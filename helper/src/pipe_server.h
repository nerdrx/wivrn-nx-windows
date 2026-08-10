// Named-pipe server side of the frozen ipc/wivrnnx_ipc.h contract.
#pragma once

#include <windows.h>

#include <cstdint>
#include <vector>

#include "bridge.h"
#include "video_intake.h"
#include "win_handle.h"
#include "wivrnnx_ipc.h"

namespace wivrnnx::helper
{

// Serves exactly one shim at a time on kPipeName. The pipe handle is created
// once and reconnected after every client disconnect, so the pipe name exists
// from helper startup onwards and vrserver may start before or after us.
//
// Single-threaded: overlapped I/O plus a WaitForMultipleObjects on
// {shutdown event, read completion} keeps pose writes and KeepAlive reads from
// blocking each other without a second thread.
//
// It produces nothing of its own. Everything it sends comes out of the Bridge,
// which is filled either by the WiVRn session (default mode) or by the
// synthetic tracker (--fake); when nobody is filling it, the shim simply gets
// no poses and falls back to its own static pose.
class PipeServer
{
public:
	PipeServer(HANDLE shutdown_event, Bridge & bridge, VideoIntake & video);
	~PipeServer();

	PipeServer(const PipeServer &) = delete;
	PipeServer & operator=(const PipeServer &) = delete;

	// Creates the pipe and the overlapped events. Logs and returns false on
	// failure (most commonly ERROR_PIPE_BUSY: another helper is running).
	bool create();

	// Accept/serve/disconnect forever, until the shutdown event is signalled.
	void run();

private:
	enum class WaitResult
	{
		message,  // a complete message is in msg_
		timeout,  // nothing arrived within the timeout
		shutdown, // shutdown event signalled
		error,    // pipe broken or protocol violation; drop the client
	};

	// Blocks until a client connects, the shutdown event fires, or the connect
	// fails. Returns false when we should stop.
	bool wait_for_client();

	// ShimHello in, HelperHello + HmdConfig out. False = drop this client.
	bool handshake();

	// Drain the bridge onto the pipe, drain KeepAlive/Haptic off it. Returns
	// false only on shutdown.
	bool serve();

	// Send everything the cursor has not seen yet. False = the client is gone.
	bool flush_bridge();

	// Hand back every staging slot the encoder thread has finished with. False =
	// the client is gone.
	bool flush_frame_done();

	void disconnect_client();

	bool post_read();
	WaitResult wait_for_message(DWORD timeout_ms);
	bool handle_message();

	bool send_message(ipc::MessageType type, const void * payload, size_t payload_size);

	HANDLE shutdown_ = nullptr; // not owned
	Bridge & bridge_;
	VideoIntake & video_;
	UniqueHandle pipe_;
	UniqueHandle connect_event_;
	UniqueHandle read_event_;
	UniqueHandle write_event_;
	OVERLAPPED connect_ovl_{};
	OVERLAPPED read_ovl_{};
	OVERLAPPED write_ovl_{};

	bool read_pending_ = false;
	unsigned char msg_[ipc::kMaxMessageSize]{};
	DWORD msg_size_ = 0;

	// Reset on every connect: a reconnecting shim must see the whole state
	// again, HmdConfig first.
	Bridge::Cursor cursor_{};
	std::vector<Outgoing> pending_;

	uint64_t poses_sent_ = 0;
	uint64_t inputs_sent_ = 0;
	uint64_t frames_ready_seen_ = 0;
	uint64_t frames_done_sent_ = 0;
	uint64_t staging_configs_seen_ = 0;
	uint64_t keepalives_seen_ = 0;
	uint64_t haptics_seen_ = 0;
	uint64_t unknown_seen_ = 0;
};

} // namespace wivrnnx::helper
