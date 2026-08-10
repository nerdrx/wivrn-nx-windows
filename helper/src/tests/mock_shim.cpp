// A shim that is not a SteamVR driver.
//
// Speaks protocol v3 on \\.\pipe\wivrnnx-driver against a real, running
// wivrnnx-helper.exe: handshake, then a staging ring and a stream of FrameReady,
// and checks what comes back. It exists because the frame *intake* — message
// sizes, generation handling, the FrameDone that frees a slot, and the promise
// that none of it delays a pose — is the half of the video path that has nothing
// to do with a GPU and can therefore be proven under Wine.
//
// It cannot open a shared texture: cross-process D3D11 sharing does not work in
// a Wine prefix and there is no vrserver to duplicate a handle out of. The
// handles it sends are deliberate nonsense, which is exactly the case the helper
// has to survive - and with no WiVRn client connected it never gets as far as
// trying, because the contract says a FrameReady with nobody to send the frame
// to is answered immediately with FrameDone flags=1.
//
// Usage:  wine wivrnnx-helper.exe --fake &
//         wine mock_shim.exe [frames]

#include <windows.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "wivrnnx_ipc.h"

using namespace wivrnnx::ipc;

namespace
{

int checks = 0;
int failures = 0;

#define CHECK(...)                                                                          \
	do                                                                                  \
	{                                                                                   \
		++checks;                                                                   \
		if (not(__VA_ARGS__))                                                       \
		{                                                                           \
			++failures;                                                         \
			std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__); \
		}                                                                           \
	} while (0)

HANDLE g_pipe = INVALID_HANDLE_VALUE;

bool connect_pipe(int timeout_ms)
{
	const DWORD deadline = GetTickCount() + static_cast<DWORD>(timeout_ms);
	for (;;)
	{
		g_pipe = CreateFileW(kPipeNameW, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
		if (g_pipe != INVALID_HANDLE_VALUE)
			break;
		if (GetTickCount() > deadline)
		{
			std::printf("could not open %s (error %lu)\n", kPipeName, GetLastError());
			return false;
		}
		Sleep(50);
	}

	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!SetNamedPipeHandleState(g_pipe, &mode, nullptr, nullptr))
	{
		std::printf("SetNamedPipeHandleState failed (%lu)\n", GetLastError());
		return false;
	}
	return true;
}

bool send_message(MessageType type, const void * payload, size_t size)
{
	unsigned char buf[kMaxMessageSize];
	const size_t total = sizeof(uint32_t) + size;
	const uint32_t raw = static_cast<uint32_t>(type);
	std::memcpy(buf, &raw, sizeof(raw));
	std::memcpy(buf + sizeof(raw), payload, size);

	DWORD written = 0;
	return WriteFile(g_pipe, buf, static_cast<DWORD>(total), &written, nullptr) && written == total;
}

struct Message
{
	MessageType type{};
	unsigned char payload[kMaxMessageSize]{};
	DWORD payload_size = 0;
};

// Reads one message, waiting up to `timeout_ms` in 5 ms slices. The pipe is
// opened in blocking mode, so PeekNamedPipe is what keeps this from parking on a
// helper that has nothing to say.
bool receive_message(Message & out, int timeout_ms)
{
	const DWORD deadline = GetTickCount() + static_cast<DWORD>(timeout_ms);
	for (;;)
	{
		DWORD available = 0;
		if (!PeekNamedPipe(g_pipe, nullptr, 0, nullptr, &available, nullptr))
			return false;

		if (available > 0)
		{
			unsigned char buf[kMaxMessageSize];
			DWORD read = 0;
			if (!ReadFile(g_pipe, buf, sizeof(buf), &read, nullptr) || read < sizeof(uint32_t))
				return false;

			uint32_t raw = 0;
			std::memcpy(&raw, buf, sizeof(raw));
			out.type = static_cast<MessageType>(raw);
			out.payload_size = read - static_cast<DWORD>(sizeof(uint32_t));
			std::memcpy(out.payload, buf + sizeof(uint32_t), out.payload_size);
			return true;
		}

		if (GetTickCount() > deadline)
			return false;
		Sleep(5);
	}
}

// Reads until a message of `type` arrives, counting everything else on the way.
// Poses and the rest keep flowing while frames are in flight, which is the point
// of the counters the caller reads back.
bool receive_until(MessageType type, Message & out, int timeout_ms, int * poses = nullptr)
{
	const DWORD deadline = GetTickCount() + static_cast<DWORD>(timeout_ms);
	for (;;)
	{
		const int left = static_cast<int>(deadline - GetTickCount());
		if (left <= 0)
			return false;
		if (!receive_message(out, left))
			return false;
		if (out.type == type)
			return true;
		if (poses != nullptr && out.type == MessageType::pose_update)
			++*poses;
	}
}

StagingConfig make_staging(uint32_t generation)
{
	StagingConfig config{};
	config.generation = generation;
	config.width = 3200; // 2 x 1600 per eye
	config.height = 1760;
	config.dxgi_format = 28; // DXGI_FORMAT_R8G8B8A8_UNORM
	config.count = 3;
	// Not handles. Nothing in the no-client path ever looks at them, and if the
	// helper ever did try to duplicate one it would fail loudly rather than
	// silently succeed - which is the behaviour worth having here.
	config.handles[0] = 0xDEAD0001;
	config.handles[1] = 0xDEAD0002;
	config.handles[2] = 0xDEAD0003;
	return config;
}

FrameReady make_frame(uint64_t id, uint32_t generation, uint32_t slot)
{
	FrameReady frame{};
	frame.frame_id = id;
	LARGE_INTEGER qpc{};
	QueryPerformanceCounter(&qpc);
	frame.sample_time_qpc = static_cast<uint64_t>(qpc.QuadPart);
	frame.generation = generation;
	frame.staging_index = slot;
	frame.qw = 1.f;
	frame.predict_s = 0.011f;
	return frame;
}

} // namespace

int main(int argc, char ** argv)
{
	const int frame_count = argc > 1 ? std::atoi(argv[1]) : 60;

	std::printf("mock_shim: protocol v%u frame intake against a live helper\n\n", kProtocolVersion);

	if (!connect_pipe(10000))
		return 1;

	// --- handshake -------------------------------------------------------
	{
		ShimHello hello{kShimMagic, kProtocolVersion, GetCurrentProcessId()};
		CHECK(send_message(MessageType::shim_hello, &hello, sizeof(hello)));

		Message msg;
		CHECK(receive_message(msg, 5000));
		CHECK(msg.type == MessageType::helper_hello);
		CHECK(msg.payload_size == sizeof(HelperHello));

		HelperHello reply{};
		std::memcpy(&reply, msg.payload, sizeof(reply));
		CHECK(reply.magic == kHelperMagic);
		CHECK(reply.version == kProtocolVersion);

		CHECK(receive_until(MessageType::hmd_config, msg, 5000));
		CHECK(msg.payload_size == sizeof(HmdConfig));

		HmdConfig config{};
		std::memcpy(&config, msg.payload, sizeof(config));
		std::printf("  handshake ok: %ux%u per eye @ %.1f Hz\n",
		            config.eye_width,
		            config.eye_height,
		            static_cast<double>(config.refresh_hz));
		CHECK(config.eye_width > 0 && config.eye_height > 0);
	}

	// --- the staging ring ------------------------------------------------
	const uint32_t generation = 1;
	{
		const StagingConfig config = make_staging(generation);
		CHECK(send_message(MessageType::staging_config, &config, sizeof(config)));
	}

	// --- frames ----------------------------------------------------------
	//
	// One in flight at a time, which is what a shim with a three-slot ring does
	// when the encoder keeps up. With no WiVRn client connected every one of them
	// must come back as FrameDone flags=1 (dropped), immediately.
	{
		int poses = 0;
		int done = 0;
		int dropped = 0;
		DWORD worst_ms = 0;

		for (int i = 0; i < frame_count; ++i)
		{
			const FrameReady frame = make_frame(1000 + i, generation, i % 3);
			CHECK(send_message(MessageType::frame_ready, &frame, sizeof(frame)));

			const DWORD sent_at = GetTickCount();
			Message msg;
			if (!receive_until(MessageType::frame_done, msg, 2000, &poses))
			{
				++failures;
				++checks;
				std::printf("  FAIL no FrameDone for frame %llu\n",
				            static_cast<unsigned long long>(frame.frame_id));
				break;
			}
			worst_ms = (GetTickCount() - sent_at) > worst_ms ? (GetTickCount() - sent_at) : worst_ms;

			CHECK(msg.payload_size == sizeof(FrameDone));
			FrameDone reply{};
			std::memcpy(&reply, msg.payload, sizeof(reply));
			CHECK(reply.frame_id == frame.frame_id);
			CHECK(reply.staging_index == frame.staging_index);
			++done;
			if (reply.flags == 1)
				++dropped;

			// The shim's own cadence; also what gives the pose stream a chance
			// to interleave.
			Sleep(11);
		}

		std::printf("  %d frames, %d FrameDone, %d dropped (no client), worst turnaround %lu ms\n",
		            frame_count,
		            done,
		            dropped,
		            worst_ms);
		CHECK(done == frame_count);
		// Requirement 5 of the frame path: with no WiVRn client there is no
		// encode, and every frame is handed straight back.
		CHECK(dropped == frame_count);

		// And the pose stream did not stop while all that was going on. --fake
		// runs at its own rate; anything above zero proves the two paths are not
		// serialised behind each other.
		std::printf("  %d PoseUpdate messages arrived alongside the frames\n", poses);
		CHECK(poses > 0);
	}

	// --- a stale generation ----------------------------------------------
	//
	// The contract says a FrameReady whose generation is stale must be ignored -
	// but the slot still has to be handed back, or the shim waits on it forever.
	{
		const FrameReady frame = make_frame(9000, generation + 7, 0);
		CHECK(send_message(MessageType::frame_ready, &frame, sizeof(frame)));

		Message msg;
		CHECK(receive_until(MessageType::frame_done, msg, 2000));
		FrameDone reply{};
		std::memcpy(&reply, msg.payload, sizeof(reply));
		CHECK(reply.frame_id == 9000);
		CHECK(reply.flags == 1);
	}

	// --- an out-of-range slot --------------------------------------------
	{
		const FrameReady frame = make_frame(9001, generation, 99);
		CHECK(send_message(MessageType::frame_ready, &frame, sizeof(frame)));

		Message msg;
		CHECK(receive_until(MessageType::frame_done, msg, 2000));
		FrameDone reply{};
		std::memcpy(&reply, msg.payload, sizeof(reply));
		CHECK(reply.frame_id == 9001);
		CHECK(reply.flags == 1);
	}

	// --- a new ring generation -------------------------------------------
	{
		const StagingConfig config = make_staging(generation + 1);
		CHECK(send_message(MessageType::staging_config, &config, sizeof(config)));

		// A frame for the old ring is now the stale one.
		const FrameReady old_frame = make_frame(9002, generation, 0);
		CHECK(send_message(MessageType::frame_ready, &old_frame, sizeof(old_frame)));

		Message msg;
		CHECK(receive_until(MessageType::frame_done, msg, 2000));
		FrameDone reply{};
		std::memcpy(&reply, msg.payload, sizeof(reply));
		CHECK(reply.frame_id == 9002);
		CHECK(reply.flags == 1);

		// And one for the new ring is accepted.
		const FrameReady new_frame = make_frame(9003, generation + 1, 2);
		CHECK(send_message(MessageType::frame_ready, &new_frame, sizeof(new_frame)));
		CHECK(receive_until(MessageType::frame_done, msg, 2000));
		std::memcpy(&reply, msg.payload, sizeof(reply));
		CHECK(reply.frame_id == 9003);
		CHECK(reply.staging_index == 2);
	}

	// --- KeepAlive still works -------------------------------------------
	{
		KeepAlive alive{};
		LARGE_INTEGER qpc{};
		QueryPerformanceCounter(&qpc);
		alive.time_qpc = static_cast<uint64_t>(qpc.QuadPart);
		CHECK(send_message(MessageType::keep_alive, &alive, sizeof(alive)));

		// The helper must still be talking to us afterwards.
		Message msg;
		CHECK(receive_until(MessageType::pose_update, msg, 2000));
	}

	CloseHandle(g_pipe);

	std::printf("\n%d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
