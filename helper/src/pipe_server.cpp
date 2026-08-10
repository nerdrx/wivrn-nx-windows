#include "pipe_server.h"

#include <chrono>
#include <cstring>
#include <string>

#include "log.h"

namespace wivrnnx::helper
{

namespace
{

using clock = std::chrono::steady_clock;

constexpr DWORD kPipeBufferBytes = 4096;
constexpr DWORD kHandshakeTimeoutMs = 10000;
constexpr DWORD kWriteTimeoutMs = 2000;
constexpr auto kHeartbeatInterval = std::chrono::seconds(10);

// How long serve() blocks on the pipe before looking at the bridge again. The
// bridge is filled from another thread and has no signalling of its own, so this
// is the latency the helper adds to a pose on its way to vrserver. One
// millisecond, with timeBeginPeriod(1) already requested in main(), is a tenth
// of a frame at 90 Hz and costs a wakeup that does nothing most of the time.
constexpr DWORD kBridgePollMs = 1;

// kPipeName is a narrow string in the frozen contract; every W-suffix API
// needs it widened. The name is pure ASCII, so a byte-wise widen is exact.
std::wstring widen_ascii(const char * s)
{
	std::wstring w;
	for (; *s != '\0'; ++s)
		w.push_back(static_cast<wchar_t>(static_cast<unsigned char>(*s)));
	return w;
}

} // namespace

PipeServer::PipeServer(HANDLE shutdown_event, Bridge & bridge, VideoIntake & video) :
        shutdown_(shutdown_event),
        bridge_(bridge),
        video_(video)
{
}

PipeServer::~PipeServer()
{
	disconnect_client();
}

bool PipeServer::create()
{
	connect_event_.reset(CreateEventW(nullptr, TRUE, FALSE, nullptr));
	read_event_.reset(CreateEventW(nullptr, TRUE, FALSE, nullptr));
	write_event_.reset(CreateEventW(nullptr, TRUE, FALSE, nullptr));
	if (!connect_event_ || !read_event_ || !write_event_)
	{
		log_win32(GetLastError(), "CreateEventW failed");
		return false;
	}
	connect_ovl_.hEvent = connect_event_.get();
	read_ovl_.hEvent = read_event_.get();
	write_ovl_.hEvent = write_event_.get();

	const std::wstring name = widen_ascii(ipc::kPipeName);
	pipe_.reset(CreateNamedPipeW(name.c_str(),
	                             PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
	                             PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT |
	                                     PIPE_REJECT_REMOTE_CLIENTS,
	                             1, // single instance: one shim at a time
	                             kPipeBufferBytes,
	                             kPipeBufferBytes,
	                             0, // default timeout, unused with overlapped waits
	                             nullptr));
	if (!pipe_)
	{
		log_win32(GetLastError(), "CreateNamedPipeW(%s) failed", ipc::kPipeName);
		return false;
	}

	log_line("pipe %s created (protocol v%u, message mode, single instance)",
	         ipc::kPipeName,
	         ipc::kProtocolVersion);
	return true;
}

void PipeServer::run()
{
	for (;;)
	{
		if (!wait_for_client())
			break;

		poses_sent_ = 0;
		inputs_sent_ = 0;
		keepalives_seen_ = 0;
		haptics_seen_ = 0;
		unknown_seen_ = 0;
		frames_ready_seen_ = 0;
		frames_done_sent_ = 0;
		staging_configs_seen_ = 0;
		// A fresh cursor is what makes the whole state — HmdConfig first, then
		// every present device — go out again on a reconnect.
		cursor_ = Bridge::Cursor{};

		bool keep_running = true;
		if (handshake())
			keep_running = serve();

		disconnect_client();
		// Everything the shim shared with us belonged to the vrserver that has
		// just gone; the staging textures with it.
		video_.on_shim_gone();
		log_line("client disconnected (poses sent %llu, inputs sent %llu, keepalives seen %llu, "
		         "frames ready %llu, frames done %llu)",
		         static_cast<unsigned long long>(poses_sent_),
		         static_cast<unsigned long long>(inputs_sent_),
		         static_cast<unsigned long long>(keepalives_seen_),
		         static_cast<unsigned long long>(frames_ready_seen_),
		         static_cast<unsigned long long>(frames_done_sent_));

		if (!keep_running)
			break;
		if (WaitForSingleObject(shutdown_, 0) == WAIT_OBJECT_0)
			break;
	}
}

bool PipeServer::wait_for_client()
{
	log_line("waiting for shim to connect...");

	ResetEvent(connect_event_.get());
	if (ConnectNamedPipe(pipe_.get(), &connect_ovl_))
	{
		// Overlapped ConnectNamedPipe never succeeds synchronously.
		return true;
	}

	const DWORD err = GetLastError();
	if (err == ERROR_PIPE_CONNECTED)
	{
		// A client connected in the window between CreateNamedPipeW/
		// DisconnectNamedPipe and this call; the event is not signalled.
		log_line("client already connected");
		return true;
	}
	if (err != ERROR_IO_PENDING)
	{
		log_win32(err, "ConnectNamedPipe failed");
		return false;
	}

	HANDLE waits[2] = {shutdown_, connect_event_.get()};
	const DWORD w = WaitForMultipleObjects(2, waits, FALSE, INFINITE);
	if (w == WAIT_OBJECT_0)
	{
		CancelIoEx(pipe_.get(), &connect_ovl_);
		DWORD ignored = 0;
		GetOverlappedResult(pipe_.get(), &connect_ovl_, &ignored, TRUE);
		return false;
	}
	if (w != WAIT_OBJECT_0 + 1)
	{
		log_win32(GetLastError(), "WaitForMultipleObjects(connect) failed");
		return false;
	}

	DWORD transferred = 0;
	if (!GetOverlappedResult(pipe_.get(), &connect_ovl_, &transferred, FALSE))
	{
		log_win32(GetLastError(), "ConnectNamedPipe completion failed");
		return false;
	}
	return true;
}

bool PipeServer::handshake()
{
	const WaitResult r = wait_for_message(kHandshakeTimeoutMs);
	if (r == WaitResult::shutdown)
		return false;
	if (r == WaitResult::timeout)
	{
		log_line("handshake: no ShimHello within %lu ms, dropping client", kHandshakeTimeoutMs);
		return false;
	}
	if (r == WaitResult::error)
		return false;

	uint32_t raw_type = 0;
	std::memcpy(&raw_type, msg_, sizeof(raw_type));
	if (static_cast<ipc::MessageType>(raw_type) != ipc::MessageType::shim_hello ||
	    msg_size_ != sizeof(uint32_t) + sizeof(ipc::ShimHello))
	{
		log_line("handshake: expected ShimHello (type %u, %zu bytes), got type %u, %lu bytes - dropping client",
		         static_cast<unsigned>(ipc::MessageType::shim_hello),
		         sizeof(uint32_t) + sizeof(ipc::ShimHello),
		         raw_type,
		         msg_size_);
		return false;
	}

	ipc::ShimHello hello{};
	std::memcpy(&hello, msg_ + sizeof(uint32_t), sizeof(hello));
	if (hello.magic != ipc::kShimMagic)
	{
		log_line("handshake: bad magic 0x%08X (expected 0x%08X) - dropping client",
		         hello.magic,
		         ipc::kShimMagic);
		return false;
	}
	if (hello.version != ipc::kProtocolVersion)
	{
		log_line("handshake: protocol version mismatch, shim v%u vs helper v%u - dropping client",
		         hello.version,
		         ipc::kProtocolVersion);
		return false;
	}

	log_line("shim connected: vrserver pid %u, protocol v%u", hello.vrserver_pid, hello.version);

	// Every NT handle a StagingConfig will carry is a handle in that process, so
	// this has to be in place before the first one arrives.
	video_.set_vrserver_pid(hello.vrserver_pid);

	const ipc::HelperHello reply{ipc::kHelperMagic, ipc::kProtocolVersion};
	if (!send_message(ipc::MessageType::helper_hello, &reply, sizeof(reply)))
		return false;

	// The contract requires HmdConfig before anything else, and the bridge always
	// has one — a placeholder until a headset connects and tells us the real
	// geometry, at which point a refresh goes out on its own.
	if (!flush_bridge())
		return false;

	return true;
}

bool PipeServer::flush_bridge()
{
	pending_.clear();
	bridge_.collect(cursor_, pending_);

	for (const Outgoing & msg: pending_)
	{
		if (!send_message(msg.type, msg.payload, msg.size))
			return false;

		switch (msg.type)
		{
			case ipc::MessageType::pose_update:
				++poses_sent_;
				break;
			case ipc::MessageType::input_update:
				++inputs_sent_;
				break;
			case ipc::MessageType::hmd_config: {
				ipc::HmdConfig cfg{};
				std::memcpy(&cfg, msg.payload, sizeof(cfg));
				log_line("sent HmdConfig: %ux%u per eye @ %.1f Hz, ipd %.4f m, proj l/r/t/b %.3f/%.3f/%.3f/%.3f",
				         cfg.eye_width,
				         cfg.eye_height,
				         static_cast<double>(cfg.refresh_hz),
				         static_cast<double>(cfg.ipd_m),
				         static_cast<double>(cfg.proj_left[0]),
				         static_cast<double>(cfg.proj_right[0]),
				         static_cast<double>(cfg.proj_top[0]),
				         static_cast<double>(cfg.proj_bottom[0]));
				break;
			}
			case ipc::MessageType::device_add:
				log_line("sent DeviceAdd for device %u", msg.payload[0]);
				break;
			case ipc::MessageType::device_remove:
				log_line("sent DeviceRemove for device %u", msg.payload[0]);
				break;
			default:
				break;
		}
	}

	return true;
}

bool PipeServer::flush_frame_done()
{
	// Bounded by the ring: the encoder thread can never have more outstanding
	// than there are slots, and this loop drains whatever is there each pass.
	ipc::FrameDone done{};
	while (video_.pop_done(done))
	{
		if (!send_message(ipc::MessageType::frame_done, &done, sizeof(done)))
			return false;
		++frames_done_sent_;
	}
	return true;
}

bool PipeServer::serve()
{
	const auto start = clock::now();
	auto next_heartbeat = start + kHeartbeatInterval;

	for (;;)
	{
		if (!flush_bridge())
			return true; // client gone; keep the helper alive

		// Before the poll below, not after: a slot the shim has not been given
		// back is a slot it skips, so every millisecond this waits is a frame
		// the shim may drop.
		if (!flush_frame_done())
			return true;

		const auto now = clock::now();
		if (now >= next_heartbeat)
		{
			log_line("heartbeat: %llu poses sent, %llu inputs sent, %llu keepalives seen, "
			         "%llu frames ready / %llu encoded / %llu dropped, %.1f s connected",
			         static_cast<unsigned long long>(poses_sent_),
			         static_cast<unsigned long long>(inputs_sent_),
			         static_cast<unsigned long long>(keepalives_seen_),
			         static_cast<unsigned long long>(frames_ready_seen_),
			         static_cast<unsigned long long>(video_.frames_encoded()),
			         static_cast<unsigned long long>(video_.frames_dropped()),
			         std::chrono::duration<double>(now - start).count());
			next_heartbeat = now + kHeartbeatInterval;
		}

		switch (wait_for_message(kBridgePollMs))
		{
			case WaitResult::message:
				if (!handle_message())
					return true;
				break;
			case WaitResult::timeout:
				break;
			case WaitResult::shutdown:
				return false;
			case WaitResult::error:
				return true;
		}
	}
}

void PipeServer::disconnect_client()
{
	if (!pipe_)
		return;

	if (read_pending_)
	{
		CancelIoEx(pipe_.get(), &read_ovl_);
		DWORD ignored = 0;
		GetOverlappedResult(pipe_.get(), &read_ovl_, &ignored, TRUE);
		read_pending_ = false;
	}
	FlushFileBuffers(pipe_.get());
	DisconnectNamedPipe(pipe_.get());
}

bool PipeServer::post_read()
{
	ResetEvent(read_event_.get());
	if (ReadFile(pipe_.get(), msg_, static_cast<DWORD>(sizeof(msg_)), nullptr, &read_ovl_))
		return true; // completed synchronously; the event is signalled too

	const DWORD err = GetLastError();
	if (err == ERROR_IO_PENDING)
		return true;
	if (err == ERROR_BROKEN_PIPE || err == ERROR_PIPE_NOT_CONNECTED || err == ERROR_NO_DATA)
		return false;

	log_win32(err, "ReadFile failed");
	return false;
}

PipeServer::WaitResult PipeServer::wait_for_message(DWORD timeout_ms)
{
	if (!read_pending_)
	{
		if (!post_read())
			return WaitResult::error;
		read_pending_ = true;
	}

	HANDLE waits[2] = {shutdown_, read_event_.get()};
	const DWORD w = WaitForMultipleObjects(2, waits, FALSE, timeout_ms);
	if (w == WAIT_OBJECT_0)
		return WaitResult::shutdown;
	if (w == WAIT_TIMEOUT)
		return WaitResult::timeout;
	if (w != WAIT_OBJECT_0 + 1)
	{
		log_win32(GetLastError(), "WaitForMultipleObjects(read) failed");
		return WaitResult::error;
	}

	DWORD transferred = 0;
	if (!GetOverlappedResult(pipe_.get(), &read_ovl_, &transferred, FALSE))
	{
		const DWORD err = GetLastError();
		if (err == ERROR_IO_INCOMPLETE)
			return WaitResult::timeout;

		read_pending_ = false;
		if (err == ERROR_BROKEN_PIPE || err == ERROR_PIPE_NOT_CONNECTED)
			log_line("shim closed the pipe");
		else
			log_win32(err, "read completion failed");
		return WaitResult::error;
	}

	read_pending_ = false;
	msg_size_ = transferred;
	if (msg_size_ < sizeof(uint32_t))
	{
		log_line("malformed message: %lu bytes, need at least %zu - dropping client",
		         msg_size_,
		         sizeof(uint32_t));
		return WaitResult::error;
	}
	return WaitResult::message;
}

bool PipeServer::handle_message()
{
	uint32_t raw_type = 0;
	std::memcpy(&raw_type, msg_, sizeof(raw_type));
	const DWORD payload_size = msg_size_ - static_cast<DWORD>(sizeof(uint32_t));

	switch (static_cast<ipc::MessageType>(raw_type))
	{
		case ipc::MessageType::keep_alive:
			if (payload_size != sizeof(ipc::KeepAlive))
			{
				log_line("KeepAlive with %lu byte payload (expected %zu) - dropping client",
				         payload_size,
				         sizeof(ipc::KeepAlive));
				return false;
			}
			++keepalives_seen_;
			return true;

		case ipc::MessageType::haptic:
			if (payload_size != sizeof(ipc::Haptic))
			{
				log_line("Haptic with %lu byte payload (expected %zu) - dropping client",
				         payload_size,
				         sizeof(ipc::Haptic));
				return false;
			}
			else
			{
				ipc::Haptic haptic{};
				std::memcpy(&haptic, msg_ + sizeof(uint32_t), sizeof(haptic));
				bridge_.push_haptic(haptic);
				++haptics_seen_;
			}
			return true;

		case ipc::MessageType::staging_config:
			if (payload_size != sizeof(ipc::StagingConfig))
			{
				log_line("StagingConfig with %lu byte payload (expected %zu) - dropping client",
				         payload_size,
				         sizeof(ipc::StagingConfig));
				return false;
			}
			else
			{
				ipc::StagingConfig config{};
				std::memcpy(&config, msg_ + sizeof(uint32_t), sizeof(config));
				video_.on_staging_config(config);
				++staging_configs_seen_;
			}
			return true;

		case ipc::MessageType::frame_ready:
			if (payload_size != sizeof(ipc::FrameReady))
			{
				log_line("FrameReady with %lu byte payload (expected %zu) - dropping client",
				         payload_size,
				         sizeof(ipc::FrameReady));
				return false;
			}
			else
			{
				ipc::FrameReady frame{};
				std::memcpy(&frame, msg_ + sizeof(uint32_t), sizeof(frame));
				// Hands over and returns; the encode happens on the intake's own
				// thread. This loop is the one carrying poses and must not wait
				// on a GPU for any reason.
				video_.on_frame_ready(frame);
				++frames_ready_seen_;
			}
			return true;

		case ipc::MessageType::shim_hello:
			log_line("unexpected second ShimHello - dropping client");
			return false;

		default:
			// Anything the contract does not define at this version: ignore,
			// but say so once so a protocol drift does not stay silent.
			if (unknown_seen_ == 0)
				log_line("ignoring unexpected message type %u (%lu byte payload)",
				         raw_type,
				         payload_size);
			++unknown_seen_;
			return true;
	}
}

bool PipeServer::send_message(ipc::MessageType type, const void * payload, size_t payload_size)
{
	unsigned char buf[ipc::kMaxMessageSize];
	const size_t total = sizeof(uint32_t) + payload_size;
	if (total > sizeof(buf))
	{
		log_line("internal error: message type %u is %zu bytes, over kMaxMessageSize",
		         static_cast<unsigned>(type),
		         total);
		return false;
	}

	const uint32_t raw_type = static_cast<uint32_t>(type);
	std::memcpy(buf, &raw_type, sizeof(raw_type));
	std::memcpy(buf + sizeof(raw_type), payload, payload_size);

	ResetEvent(write_event_.get());
	DWORD written = 0;
	if (!WriteFile(pipe_.get(), buf, static_cast<DWORD>(total), &written, &write_ovl_))
	{
		const DWORD err = GetLastError();
		if (err != ERROR_IO_PENDING)
		{
			if (err == ERROR_BROKEN_PIPE || err == ERROR_NO_DATA || err == ERROR_PIPE_NOT_CONNECTED)
				log_line("shim closed the pipe while writing message type %u", raw_type);
			else
				log_win32(err, "WriteFile(type %u) failed", raw_type);
			return false;
		}

		HANDLE waits[2] = {shutdown_, write_event_.get()};
		const DWORD w = WaitForMultipleObjects(2, waits, FALSE, kWriteTimeoutMs);
		if (w != WAIT_OBJECT_0 + 1)
		{
			if (w == WAIT_TIMEOUT)
				log_line("write of message type %u stalled for %lu ms - dropping client",
				         raw_type,
				         kWriteTimeoutMs);
			CancelIoEx(pipe_.get(), &write_ovl_);
			DWORD ignored = 0;
			GetOverlappedResult(pipe_.get(), &write_ovl_, &ignored, TRUE);
			return false;
		}

		if (!GetOverlappedResult(pipe_.get(), &write_ovl_, &written, FALSE))
		{
			log_win32(GetLastError(), "write completion (type %u) failed", raw_type);
			return false;
		}
	}

	if (written != total)
	{
		log_line("short write: %lu of %zu bytes for message type %u", written, total, raw_type);
		return false;
	}
	return true;
}

} // namespace wivrnnx::helper
