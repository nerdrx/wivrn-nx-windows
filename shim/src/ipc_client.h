// Named-pipe client for the wivrnnx-helper.exe process.
//
// The helper is the pipe server; we are the client and reconnect forever with
// ~1 s backoff, because vrserver may outlive or predate the helper. All pipe I/O
// happens on one dedicated thread; callbacks are invoked from that thread.
#pragma once

#include "win_handle.h"

#include <wivrnnx_ipc.h>

#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace wivrnnx
{

struct IpcCallbacks
{
	// Fired once per session, right after a successful handshake.
	std::function<void(const ipc::HmdConfig &)> on_config;
	// Fired for every PoseUpdate that arrives after the config.
	std::function<void(const ipc::PoseUpdate &)> on_pose;
	// v2: device lifecycle and controller input.
	std::function<void(const ipc::DeviceAdd &)> on_device_add;
	std::function<void(const ipc::DeviceRemove &)> on_device_remove;
	std::function<void(const ipc::InputUpdate &)> on_input;
	// v3: the helper is done with a staging slot.
	std::function<void(const ipc::FrameDone &)> on_frame_done;
	// v3: fired once per session the instant connected() turns true, i.e. after
	// the handshake completed. This is the hook the video path needs: anything
	// that must be re-announced to a *new* helper (StagingConfig) is queued
	// from here, not from on_config, because queueing is only accepted once the
	// session counts as connected.
	std::function<void()> on_session_ready;
	// Fired whenever a session ends (helper died, protocol error, shutdown).
	std::function<void()> on_disconnect;
};

class IpcClient
{
public:
	IpcClient() = default;
	~IpcClient();

	IpcClient(const IpcClient &) = delete;
	IpcClient & operator=(const IpcClient &) = delete;

	// Starts the client thread. Returns false only if the stop event or the
	// thread could not be created; a helper that is not running yet is not an
	// error, the thread simply keeps retrying.
	bool start(IpcCallbacks callbacks) noexcept;

	// Signals the thread, cancels any pending I/O and joins. Idempotent.
	void stop() noexcept;

	// Queues a haptic pulse for the helper. Safe to call from any thread (in
	// practice vrserver's RunFrame thread): all pipe writes stay on the client
	// thread, this only hands the payload over and wakes it. Returns false if
	// there is no session or the queue is saturated.
	bool queue_haptic(const ipc::Haptic & haptic) noexcept;

	// v3 video transport, queued the same way and from the compositor's present
	// thread. StagingConfig is the one message that must not be lost, so it
	// evicts queued FrameReadys rather than being refused; a FrameReady that
	// cannot be queued is simply a dropped frame.
	bool queue_staging_config(const ipc::StagingConfig & config) noexcept;
	bool queue_frame_ready(const ipc::FrameReady & frame) noexcept;

	bool connected() const noexcept
	{
		return connected_.load(std::memory_order_acquire);
	}

private:
	// Bound on the outgoing queue. Payloads are tens of bytes and the client
	// thread drains the queue every loop iteration (it wakes on the event, it
	// does not wait out the read slice first); if we ever hit this the helper is
	// wedged and dropping is the right answer.
	static constexpr size_t kMaxPendingMessages = 64;
	// Every shim->helper payload fits in this: Haptic is 20 bytes,
	// StagingConfig 48, FrameReady 56.
	static constexpr size_t kMaxPendingPayload = 64;

	struct PendingMessage
	{
		ipc::MessageType type{};
		uint32_t size = 0;
		uint8_t payload[kMaxPendingPayload]{};
	};

	// Common tail of the three queue_* entry points.
	bool queue_message(ipc::MessageType type, const void * payload, size_t size, bool evictable) noexcept;

	enum class ReadResult
	{
		message,
		timeout,
		stopped,
		error,
		// Something on our side (a queued message) wants attention; no pipe
		// message was consumed and any in-flight read stays pending.
		interrupted,
	};

	void thread_main() noexcept;
	bool open_pipe() noexcept;
	void close_pipe() noexcept;
	void run_session() noexcept;
	bool handle_message(const uint8_t * data, uint32_t size, bool & got_helper_hello, bool & got_config) noexcept;

	bool send_message(ipc::MessageType type, const void * payload, size_t payload_size) noexcept;
	ReadResult pump_read(DWORD timeout_ms, DWORD & out_bytes) noexcept;
	void cancel_pending_read() noexcept;
	// Returns false if the stop event fired while waiting.
	bool wait_or_stop(DWORD timeout_ms) noexcept;
	// Client thread only. Returns false if a write failed (session is dead).
	bool flush_outgoing() noexcept;
	void drop_pending_messages() noexcept;

	IpcCallbacks callbacks_;
	std::thread thread_;
	UniqueHandle stop_event_;
	UniqueHandle read_event_;
	UniqueHandle write_event_;
	UniqueHandle outgoing_event_;
	UniqueHandle pipe_;

	std::mutex outgoing_mutex_;
	std::deque<PendingMessage> pending_;

	OVERLAPPED read_overlapped_{};
	uint8_t read_buffer_[ipc::kMaxMessageSize]{};
	bool read_pending_ = false;

	std::atomic<bool> connected_{false};
	std::atomic<bool> running_{false};
};

} // namespace wivrnnx
