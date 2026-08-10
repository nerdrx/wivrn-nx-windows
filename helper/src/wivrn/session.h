// One headset session: the WiVRn handshake, then tracking in and haptics out.
//
// The handshake is copy-adapted from wivrn::wivrn_connection::init()
// (server/driver/wivrn_connection.cpp:313-540). The packet sequence, the
// timeouts, the SMP PIN exchange and the key derivation are unchanged — they
// have to be, the client APK is the shipping one. What is dropped is
// everything that only exists because the Linux server is split across two
// processes with a Monado compositor underneath: the wivrn_ipc socket, the
// secondary (multipath) path, the path_selector, QoS marks, video, audio and
// feedback.
//
// The session runs on its own thread and writes into the Bridge; it never
// touches the pipe.
#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "wivrn_packets.h"
#include "wivrn_sockets.h"

#include "../bridge.h"
#include "../video_bridge.h"
#include "bitrate_controller.h"
#include "clock_offset.h"
#include "idr_tracker.h"
#include "pose_sanitize.h"
#include "shard_pacer.h"
#include "video_out.h"
#include "wivrnnx_ipc.h"

namespace wivrnnx::helper
{

class incorrect_pin : public std::runtime_error
{
public:
	incorrect_pin() :
	        std::runtime_error("Incorrect PIN") {}
};

// Mirrors wivrn_connection::encryption_state.
enum class EncryptionState
{
	// No encryption, no authentication. Anyone on the network can connect.
	disabled,
	// Encrypted, but only already-paired headsets are accepted.
	enabled,
	// Encrypted, and an unknown headset may pair by entering the PIN.
	pairing,
};

struct ServerOptions
{
	// common/wivrn_config.h.in:26 - the port the client's discovery expects
	// unless the SRV record says otherwise.
	int port = 9757;
	EncryptionState encryption = EncryptionState::pairing;
	std::string pin;
	// No UDP stream socket: everything rides the control TCP connection.
	bool tcp_only = false;
};

class Session
{
public:
	// Runs the handshake; throws on failure (incorrect_pin for a wrong PIN).
	Session(wivrn::TCP && tcp,
	        const ServerOptions & options,
	        Bridge & bridge,
	        VideoBridge & video,
	        std::atomic<bool> & stop);
	~Session();

	Session(const Session &) = delete;
	Session & operator=(const Session &) = delete;

	// Receive loop. Returns when the client disconnects or `stop` is set.
	void run();

private:
	using clock = std::chrono::steady_clock;

	void handshake();

	// Blocks until one packet arrives. `allow_stream` also watches the UDP
	// socket and returns the source port, which is how the client's stream port
	// is learnt; -1 means the packet came in on the control socket.
	std::pair<wivrn::from_headset::packets, int> receive_one(std::chrono::seconds timeout, bool allow_stream);

	// Everything the client has to be told before it will send tracking.
	void send_stream_setup();
	void send_tracking_pattern();

	// Re-sends video_stream_description whenever the encoder's idea of the
	// stream changes. The client ignores a description equal to the one it
	// already has and rebuilds its decoders for any other
	// (client/scenes/stream.cpp:1522).
	void update_stream_description();

	// Tells the encoder thread what this client can decode and how fast.
	void publish_video_request();

	// Moves whatever the encoder thread produced onto the socket, a paced
	// micro-burst at a time. Never blocks: what is not due yet is left for the
	// next turn of the loop, and next_video_due_ says when that is.
	void pump_video();
	// Starts the frame at the head of the queue. Returns false when it produced
	// nothing (an empty bitstream), in which case the caller pops and retries.
	bool begin_video_frame(int64_t now_ns);
	// Books the frame that just finished: counters, the IDR tracker, and the byte
	// count the bandwidth estimator eats.
	void finish_video_frame();

	// The transport half of the client's settings: the bitrate controller's two
	// switches and its control law, packet pacing, and forward error correction.
	// Read out of headset_info/settings_changed, ANDed with the command line.
	void configure_transport();
	// A bitrate the controller decided, in bits per second on the link. Turns it
	// into what the encoder should be told to produce and hands it over.
	void apply_bitrate(std::optional<uint32_t> bitrate_bps);

	int64_t frame_period_ns() const
	{
		return refresh_hz_ > 1.f ? int64_t(1e9 / double(refresh_hz_)) : 0;
	}

	void on_tracking(const wivrn::from_headset::tracking & tracking);
	void on_inputs(const wivrn::from_headset::inputs & inputs);
	void on_headset_info(const wivrn::from_headset::headset_info_packet & info);
	void on_settings(const wivrn::from_headset::settings_changed & settings);
	void on_feedback(const wivrn::from_headset::feedback & feedback);

	// Recompute HmdConfig from whatever the client has told us so far and hand
	// it to the bridge, which only bumps its generation when it really changed.
	void publish_config();

	void pump_haptics();

	template <typename T>
	void send_control(T && packet);
	template <typename T>
	void send_stream(T && packet);

	wivrn::typed_socket<wivrn::TCP, wivrn::from_headset::packets, wivrn::to_headset::packets> control_;
	wivrn::typed_socket<wivrn::UDP, wivrn::from_headset::packets, wivrn::to_headset::packets> stream_{-1};

	const ServerOptions & options_;
	Bridge & bridge_;
	VideoBridge & video_;
	std::atomic<bool> & stop_;

	std::array<uint8_t, 16> token_{};
	wivrn::from_headset::headset_info_packet info_{};
	bool have_info_ = false;

	// Eye geometry as the client last reported it, in tracking::view form
	// (poses relative to VIEW space, i.e. eye-to-head directly).
	std::array<wivrn::from_headset::tracking::view, 2> views_{};
	bool have_views_ = false;
	float refresh_hz_ = 90.f;

	// from_headset::settings_changed::standby_freeze - the NX switch the
	// sanitizers below obey. Defaults to on, like the client's own default.
	bool standby_freeze_ = true;

	ClockOffsetEstimator clock_;

	// Indexed by ipc::DeviceId.
	std::array<PoseSanitizer, kDeviceCount> sanitizers_{};
	std::array<ipc::InputUpdate, kDeviceCount> input_state_{};

	// --- video -------------------------------------------------------------

	IdrTracker idr_;
	// The description the client currently has, and the encoder state generation
	// it was built from.
	wivrn::to_headset::video_stream_description description_{};
	bool description_sent_ = false;
	uint64_t stream_generation_ = 0;
	std::vector<EncodedFrame> video_scratch_;

	// --- transport ---------------------------------------------------------

	BitrateController bitrate_;
	// The pacing window of the socket, shared by the frames of one slot: both eyes
	// of a frame are drained by this thread, so they divide one window between them
	// rather than taking one each (shard_pacer.h:162-176).
	PacingSlot pacing_slot_;
	PacedSender sender_;
	PacedSender::Sinks sinks_;
	// Frames handed over by the encoder thread, oldest first. The one at the front
	// is the one PacedSender is working through, and its bitstream must not move
	// while it is: the shard payloads are spans into it.
	std::deque<EncodedFrame> video_queue_;
	// Deeper than VideoBridge::kMaxQueued would ever hand over at once; a pacer
	// that fell behind must still not let this grow without bound.
	static constexpr size_t kMaxVideoQueued = 6;

	bool pacing_enabled_ = true;
	float pacing_window_ = 0.4f;
	bool fec_enabled_ = false;
	// Absolute monotonic time the next micro-burst is due, 0 when nothing is
	// waiting. What the poll() timeout is shortened to.
	int64_t next_video_due_ = 0;

	uint64_t video_frames_sent_ = 0;
	uint64_t video_shards_sent_ = 0;
	uint64_t video_parity_sent_ = 0;
	uint64_t video_bytes_sent_ = 0;
	uint64_t video_send_errors_ = 0;
	uint64_t video_frames_dropped_ = 0;
	uint64_t idr_requests_ = 0;
	uint32_t feedback_logged_ = 0;

	uint64_t tracking_packets_ = 0;
	uint64_t input_packets_ = 0;
	uint64_t discarded_packets_ = 0;
	clock::time_point next_report_{};
};

} // namespace wivrnnx::helper
