#include "ipc_client.h"

#include "driverlog.h"

#include <chrono>
#include <cstring>
#include <string>

namespace wivrnnx
{
namespace
{

constexpr DWORD kReconnectBackoffMs = 1000;
// Time budget for HelperHello + HmdConfig to arrive after we send ShimHello.
constexpr DWORD kHandshakeTimeoutMs = 10000;
// Read wait slice; also the resolution at which we notice the keep-alive is due.
constexpr DWORD kReadSliceMs = 100;
constexpr DWORD kWriteTimeoutMs = 2000;
constexpr auto kKeepAliveInterval = std::chrono::seconds(1);

// The contract header spells the pipe name as narrow ASCII; widen it here so
// every Win32 call in the shim can stay on the W entry points without keeping a
// second copy of the name around.
std::wstring pipe_name_wide()
{
	std::wstring wide;
	for (const char * p = ipc::kPipeName; *p != '\0'; ++p)
		wide.push_back(static_cast<wchar_t>(static_cast<unsigned char>(*p)));
	return wide;
}

uint64_t query_qpc() noexcept
{
	LARGE_INTEGER counter{};
	::QueryPerformanceCounter(&counter);
	return static_cast<uint64_t>(counter.QuadPart);
}

} // namespace

IpcClient::~IpcClient()
{
	stop();
}

bool IpcClient::start(IpcCallbacks callbacks) noexcept
{
	if (running_.load(std::memory_order_acquire))
		return true;

	callbacks_ = std::move(callbacks);

	stop_event_.reset(::CreateEventW(nullptr, TRUE, FALSE, nullptr));
	read_event_.reset(::CreateEventW(nullptr, TRUE, FALSE, nullptr));
	write_event_.reset(::CreateEventW(nullptr, TRUE, FALSE, nullptr));
	// Auto-reset: it only ever means "the outgoing queue is non-empty, look".
	outgoing_event_.reset(::CreateEventW(nullptr, FALSE, FALSE, nullptr));
	if (!stop_event_.valid() || !read_event_.valid() || !write_event_.valid() || !outgoing_event_.valid())
	{
		WNX_LOG("IPC: failed to create sync events (err %lu)", ::GetLastError());
		return false;
	}

	running_.store(true, std::memory_order_release);
	try
	{
		thread_ = std::thread([this] { thread_main(); });
	}
	catch (...)
	{
		running_.store(false, std::memory_order_release);
		WNX_LOG("IPC: failed to start client thread");
		return false;
	}
	return true;
}

void IpcClient::stop() noexcept
{
	if (!running_.exchange(false, std::memory_order_acq_rel))
		return;

	if (stop_event_.valid())
		::SetEvent(stop_event_.get());

	if (thread_.joinable())
	{
		try
		{
			thread_.join();
		}
		catch (...)
		{
			// Nothing sensible to do; never let it escape into vrserver.
		}
	}

	close_pipe();
	drop_pending_messages();
	stop_event_.reset();
	read_event_.reset();
	write_event_.reset();
	outgoing_event_.reset();
}

bool IpcClient::queue_message(ipc::MessageType type, const void * payload, size_t size, bool evictable) noexcept
{
	if (!connected_.load(std::memory_order_acquire))
		return false;
	if (payload == nullptr || size == 0 || size > kMaxPendingPayload)
		return false;

	{
		std::lock_guard<std::mutex> lock(outgoing_mutex_);
		if (pending_.size() >= kMaxPendingMessages)
		{
			if (evictable)
				return false;
			// A message that is not evictable (StagingConfig) is the one thing
			// the helper cannot recover from missing: without it every later
			// FrameReady names a slot it knows nothing about. Make room by
			// discarding the oldest queued frame instead.
			pending_.pop_front();
		}

		PendingMessage message;
		message.type = type;
		message.size = static_cast<uint32_t>(size);
		std::memcpy(message.payload, payload, size);
		try
		{
			pending_.push_back(message);
		}
		catch (...)
		{
			return false; // Never let bad_alloc escape into vrserver.
		}
	}

	if (outgoing_event_.valid())
		::SetEvent(outgoing_event_.get());
	return true;
}

bool IpcClient::queue_haptic(const ipc::Haptic & haptic) noexcept
{
	return queue_message(ipc::MessageType::haptic, &haptic, sizeof(haptic), true);
}

bool IpcClient::queue_staging_config(const ipc::StagingConfig & config) noexcept
{
	return queue_message(ipc::MessageType::staging_config, &config, sizeof(config), false);
}

bool IpcClient::queue_frame_ready(const ipc::FrameReady & frame) noexcept
{
	return queue_message(ipc::MessageType::frame_ready, &frame, sizeof(frame), true);
}

void IpcClient::drop_pending_messages() noexcept
{
	std::lock_guard<std::mutex> lock(outgoing_mutex_);
	pending_.clear();
}

bool IpcClient::flush_outgoing() noexcept
{
	for (;;)
	{
		PendingMessage message;
		{
			std::lock_guard<std::mutex> lock(outgoing_mutex_);
			if (pending_.empty())
				return true;
			message = pending_.front();
			pending_.pop_front();
		}

		if (!send_message(message.type, message.payload, message.size))
		{
			WNX_LOG("IPC: write of message type %u failed, helper is gone",
			        static_cast<unsigned>(message.type));
			return false;
		}
	}
}

bool IpcClient::wait_or_stop(DWORD timeout_ms) noexcept
{
	return ::WaitForSingleObject(stop_event_.get(), timeout_ms) != WAIT_OBJECT_0;
}

void IpcClient::thread_main() noexcept
{
	WNX_LOG("IPC: client thread started, pipe %s", ipc::kPipeName);

	bool logged_wait = false;
	while (running_.load(std::memory_order_acquire))
	{
		if (!open_pipe())
		{
			if (!logged_wait)
			{
				WNX_LOG("IPC: helper not available, retrying every %lu ms", kReconnectBackoffMs);
				logged_wait = true;
			}
			if (!wait_or_stop(kReconnectBackoffMs))
				break;
			continue;
		}

		logged_wait = false;
		WNX_LOG("IPC: connected to helper");
		run_session();
		close_pipe();

		connected_.store(false, std::memory_order_release);
		// Anything queued against the dead session must not leak into the next
		// one: by the time the helper is back the buzz is meaningless, and a
		// FrameReady would name a slot of a ring that no longer exists.
		drop_pending_messages();
		if (callbacks_.on_disconnect)
			callbacks_.on_disconnect();

		if (!running_.load(std::memory_order_acquire))
			break;

		WNX_LOG("IPC: helper session ended, reconnecting");
		if (!wait_or_stop(kReconnectBackoffMs))
			break;
	}

	WNX_LOG("IPC: client thread exiting");
}

bool IpcClient::open_pipe() noexcept
{
	const std::wstring name = pipe_name_wide();

	HANDLE handle = ::CreateFileW(name.c_str(),
	                              GENERIC_READ | GENERIC_WRITE,
	                              0,
	                              nullptr,
	                              OPEN_EXISTING,
	                              FILE_FLAG_OVERLAPPED,
	                              nullptr);
	if (handle == INVALID_HANDLE_VALUE)
		return false;

	// The contract requires message framing in both directions; the server side
	// sets PIPE_TYPE_MESSAGE, the client has to opt into message reads.
	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!::SetNamedPipeHandleState(handle, &mode, nullptr, nullptr))
	{
		WNX_LOG("IPC: SetNamedPipeHandleState(PIPE_READMODE_MESSAGE) failed (err %lu)", ::GetLastError());
		::CloseHandle(handle);
		return false;
	}

	pipe_.reset(handle);
	read_pending_ = false;
	return true;
}

void IpcClient::close_pipe() noexcept
{
	cancel_pending_read();
	pipe_.reset();
}

void IpcClient::cancel_pending_read() noexcept
{
	if (!read_pending_)
		return;

	if (pipe_.valid())
	{
		::CancelIoEx(pipe_.get(), &read_overlapped_);
		DWORD ignored = 0;
		::GetOverlappedResult(pipe_.get(), &read_overlapped_, &ignored, TRUE);
	}
	read_pending_ = false;
}

bool IpcClient::send_message(ipc::MessageType type, const void * payload, size_t payload_size) noexcept
{
	if (!pipe_.valid())
		return false;
	if (sizeof(uint32_t) + payload_size > ipc::kMaxMessageSize)
		return false;

	uint8_t buffer[ipc::kMaxMessageSize];
	const uint32_t raw_type = static_cast<uint32_t>(type);
	std::memcpy(buffer, &raw_type, sizeof(raw_type));
	if (payload_size != 0)
		std::memcpy(buffer + sizeof(raw_type), payload, payload_size);

	const DWORD total = static_cast<DWORD>(sizeof(raw_type) + payload_size);

	OVERLAPPED overlapped{};
	overlapped.hEvent = write_event_.get();
	::ResetEvent(overlapped.hEvent);

	DWORD written = 0;
	if (::WriteFile(pipe_.get(), buffer, total, &written, &overlapped))
		return written == total;

	if (::GetLastError() != ERROR_IO_PENDING)
		return false;

	HANDLE waits[2] = {stop_event_.get(), overlapped.hEvent};
	const DWORD wait = ::WaitForMultipleObjects(2, waits, FALSE, kWriteTimeoutMs);
	if (wait != WAIT_OBJECT_0 + 1)
	{
		::CancelIoEx(pipe_.get(), &overlapped);
		::GetOverlappedResult(pipe_.get(), &overlapped, &written, TRUE);
		return false;
	}
	if (!::GetOverlappedResult(pipe_.get(), &overlapped, &written, FALSE))
		return false;
	return written == total;
}

IpcClient::ReadResult IpcClient::pump_read(DWORD timeout_ms, DWORD & out_bytes) noexcept
{
	out_bytes = 0;

	if (!read_pending_)
	{
		std::memset(&read_overlapped_, 0, sizeof(read_overlapped_));
		read_overlapped_.hEvent = read_event_.get();
		::ResetEvent(read_event_.get());

		DWORD bytes = 0;
		if (::ReadFile(pipe_.get(), read_buffer_, sizeof(read_buffer_), &bytes, &read_overlapped_))
		{
			out_bytes = bytes;
			return ReadResult::message;
		}
		if (::GetLastError() != ERROR_IO_PENDING)
			return ReadResult::error;
		read_pending_ = true;
	}

	// The outgoing event is in the wait set so a pulse queued by vrserver's
	// RunFrame thread -- or a FrameReady queued by the compositor's present
	// thread, which is far more latency-critical -- does not sit here for a
	// whole read slice.
	HANDLE waits[3] = {stop_event_.get(), read_event_.get(), outgoing_event_.get()};
	const DWORD wait = ::WaitForMultipleObjects(3, waits, FALSE, timeout_ms);
	if (wait == WAIT_OBJECT_0)
		return ReadResult::stopped;
	if (wait == WAIT_TIMEOUT)
		return ReadResult::timeout;
	if (wait == WAIT_OBJECT_0 + 2)
		return ReadResult::interrupted; // read stays pending, deliberately
	if (wait != WAIT_OBJECT_0 + 1)
		return ReadResult::error;

	read_pending_ = false;
	DWORD bytes = 0;
	if (!::GetOverlappedResult(pipe_.get(), &read_overlapped_, &bytes, FALSE))
		return ReadResult::error;

	out_bytes = bytes;
	return ReadResult::message;
}

bool IpcClient::handle_message(const uint8_t * data, uint32_t size, bool & got_helper_hello, bool & got_config) noexcept
{
	if (size < sizeof(uint32_t))
	{
		WNX_LOG("IPC: runt message (%u bytes)", size);
		return false;
	}

	uint32_t raw_type = 0;
	std::memcpy(&raw_type, data, sizeof(raw_type));
	const uint8_t * payload = data + sizeof(raw_type);
	const uint32_t payload_size = size - static_cast<uint32_t>(sizeof(raw_type));

	switch (static_cast<ipc::MessageType>(raw_type))
	{
		case ipc::MessageType::helper_hello:
		{
			if (payload_size != sizeof(ipc::HelperHello))
			{
				WNX_LOG("IPC: HelperHello has wrong size (%u)", payload_size);
				return false;
			}
			ipc::HelperHello hello{};
			std::memcpy(&hello, payload, sizeof(hello));
			if (hello.magic != ipc::kHelperMagic)
			{
				WNX_LOG("IPC: bad helper magic 0x%08x", hello.magic);
				return false;
			}
			if (hello.version != ipc::kProtocolVersion)
			{
				WNX_LOG("IPC: protocol mismatch (helper %u, shim %u)", hello.version, ipc::kProtocolVersion);
				return false;
			}
			got_helper_hello = true;
			WNX_LOG("IPC: handshake accepted (protocol v%u)", hello.version);
			return true;
		}

		case ipc::MessageType::hmd_config:
		{
			if (!got_helper_hello)
			{
				WNX_LOG("IPC: HmdConfig before HelperHello, dropping session");
				return false;
			}
			if (payload_size != sizeof(ipc::HmdConfig))
			{
				WNX_LOG("IPC: HmdConfig has wrong size (%u)", payload_size);
				return false;
			}
			ipc::HmdConfig config{};
			std::memcpy(&config, payload, sizeof(config));
			got_config = true;
			if (callbacks_.on_config)
				callbacks_.on_config(config);
			return true;
		}

		case ipc::MessageType::pose_update:
		{
			if (!got_config)
				return true; // Nothing to drive yet; harmless to drop.
			if (payload_size != sizeof(ipc::PoseUpdate))
			{
				WNX_LOG("IPC: PoseUpdate has wrong size (%u)", payload_size);
				return false;
			}
			ipc::PoseUpdate pose{};
			std::memcpy(&pose, payload, sizeof(pose));
			if (callbacks_.on_pose)
				callbacks_.on_pose(pose);
			return true;
		}

		case ipc::MessageType::device_add:
		{
			if (!got_config)
				return true;
			if (payload_size != sizeof(ipc::DeviceAdd))
			{
				WNX_LOG("IPC: DeviceAdd has wrong size (%u)", payload_size);
				return false;
			}
			ipc::DeviceAdd add{};
			std::memcpy(&add, payload, sizeof(add));
			if (callbacks_.on_device_add)
				callbacks_.on_device_add(add);
			return true;
		}

		case ipc::MessageType::device_remove:
		{
			if (!got_config)
				return true;
			if (payload_size != sizeof(ipc::DeviceRemove))
			{
				WNX_LOG("IPC: DeviceRemove has wrong size (%u)", payload_size);
				return false;
			}
			ipc::DeviceRemove remove{};
			std::memcpy(&remove, payload, sizeof(remove));
			if (callbacks_.on_device_remove)
				callbacks_.on_device_remove(remove);
			return true;
		}

		case ipc::MessageType::input_update:
		{
			if (!got_config)
				return true;
			if (payload_size != sizeof(ipc::InputUpdate))
			{
				WNX_LOG("IPC: InputUpdate has wrong size (%u)", payload_size);
				return false;
			}
			ipc::InputUpdate input{};
			std::memcpy(&input, payload, sizeof(input));
			if (callbacks_.on_input)
				callbacks_.on_input(input);
			return true;
		}

		case ipc::MessageType::frame_done:
		{
			if (!got_config)
				return true; // No session, so no ring, so nothing to free.
			if (payload_size != sizeof(ipc::FrameDone))
			{
				WNX_LOG("IPC: FrameDone has wrong size (%u)", payload_size);
				return false;
			}
			ipc::FrameDone done{};
			std::memcpy(&done, payload, sizeof(done));
			if (callbacks_.on_frame_done)
				callbacks_.on_frame_done(done);
			return true;
		}

		default:
			// Anything unknown: ignore, stay connected.
			return true;
	}
}

void IpcClient::run_session() noexcept
{
	const ipc::ShimHello hello{
	        ipc::kShimMagic,
	        ipc::kProtocolVersion,
	        static_cast<uint32_t>(::GetCurrentProcessId()),
	};
	if (!send_message(ipc::MessageType::shim_hello, &hello, sizeof(hello)))
	{
		WNX_LOG("IPC: failed to send ShimHello (err %lu)", ::GetLastError());
		return;
	}

	bool got_helper_hello = false;
	bool got_config = false;
	const auto session_start = std::chrono::steady_clock::now();
	auto next_keep_alive = session_start + kKeepAliveInterval;

	while (running_.load(std::memory_order_acquire))
	{
		const auto now = std::chrono::steady_clock::now();
		if (got_helper_hello && now >= next_keep_alive)
		{
			const ipc::KeepAlive keep_alive{query_qpc()};
			if (!send_message(ipc::MessageType::keep_alive, &keep_alive, sizeof(keep_alive)))
			{
				WNX_LOG("IPC: KeepAlive write failed, helper is gone");
				return;
			}
			next_keep_alive = now + kKeepAliveInterval;
		}

		// Haptics, StagingConfig and FrameReady all go out from here, so the
		// pipe only ever has one writer.
		if (!flush_outgoing())
			return;

		DWORD bytes = 0;
		switch (pump_read(kReadSliceMs, bytes))
		{
			case ReadResult::message:
				if (!handle_message(read_buffer_, static_cast<uint32_t>(bytes), got_helper_hello, got_config))
					return;
				if (got_config && !connected_.load(std::memory_order_acquire))
				{
					connected_.store(true, std::memory_order_release);
					// Only now will queue_* accept anything, so this is the
					// earliest point at which the video path can re-announce a
					// ring that survived the previous session. The callback must
					// not block: it runs on this thread, which is the only
					// thread that can drain what it queues.
					if (callbacks_.on_session_ready)
						callbacks_.on_session_ready();
				}
				break;

			case ReadResult::timeout:
				if (!got_config)
				{
					const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
					        std::chrono::steady_clock::now() - session_start);
					if (elapsed.count() > kHandshakeTimeoutMs)
					{
						WNX_LOG("IPC: handshake timed out (hello=%d config=%d)",
						        static_cast<int>(got_helper_hello),
						        static_cast<int>(got_config));
						return;
					}
				}
				break;

			case ReadResult::interrupted:
				// A haptic pulse showed up; loop round and flush it.
				break;

			case ReadResult::stopped:
				return;

			case ReadResult::error:
				WNX_LOG("IPC: pipe read failed (err %lu)", ::GetLastError());
				return;
		}
	}
}

} // namespace wivrnnx
