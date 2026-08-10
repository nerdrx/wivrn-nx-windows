// Cutting one encoded frame into video_stream_data_shards, with forward error
// correction and packet pacing.
//
// Copy-adapted from wivrn::video_encoder::SendData
// (server/encoder/video_encoder.cpp:534-709). What is kept is the whole of what
// the headset's reassembler depends on: the shard index runs from zero, the
// first shard of a frame carries view_info and no other one does, the last shard
// carries timing_info and no other one does, and the payload budget is the FEC
// budget minus whatever view_info costs on the shard that has it. The headset's
// shard_set::complete() is exactly those three rules read backwards
// (client/decoder/shard_set.h:93-102) — there is no explicit end-of-frame flag on
// this wire, the presence of timing_info on the last shard *is* the flag.
//
// Two things upstream does inside the same loop are now here as well:
//
//   * FEC. One parity shard per fec::group_size data shards, built with
//     wivrn::fec::group_builder from ${LINUX_REPO}/common/fec.h — the same header
//     the client reconstructs with, compiled here rather than transcribed — and
//     emitted immediately after the group's last data shard, exactly as
//     video_encoder.cpp:652-658 does and for the reason given there: holding
//     them back to the end of the frame would put every parity shard in one tail
//     burst, and a hiccup that swallowed the tail would take the whole frame's
//     protection with it.
//
//     Parity is only ever built for shards that ride the UDP stream socket
//     (video_encoder.cpp:563). On TCP nothing is dropped, so a parity shard is
//     pure overhead, and an IDR — which goes out on the control socket — is
//     exactly the frame that must not be made larger.
//
//   * Pacing. Upstream sleeps its sender thread between micro-bursts
//     (video_encoder.cpp:673-674). This port has no sender thread: the shards go
//     out from the session thread, which also reads tracking off the headset, so
//     a sleep there is a pose held back. PacedSender therefore sends the bursts
//     that are due and returns when the next one is; the session shortens its
//     poll() timeout to that and comes back. Same schedule, nothing blocked.
//
// What is still dropped: the multipath spill scheduler, which has no second path
// to spill onto here.
//
// Deliberately free of Windows: this is the one file of the video path that can
// be compiled and driven natively, which is what src/tests/shard_test.cpp does.
#pragma once

#include <cstdint>
#include <functional>
#include <span>

#include "fec.h"
#include "shard_pacer.h"
#include "wivrn_packets.h"

namespace wivrnnx::helper
{

class VideoPacketizer
{
public:
	using Shard = wivrn::to_headset::video_stream_data_shard;
	using ParityShard = wivrn::to_headset::video_stream_parity_shard;

	// `prefer_control` asks for the reliable socket. The shard's payload span
	// points into the caller's buffer and is only valid for the duration of the
	// call, exactly as it is upstream.
	using SendFn = std::function<void(Shard &&, bool prefer_control)>;
	// Parity always rides the stream (UDP) socket: it only repairs what can be
	// lost, and only that socket can lose anything.
	using SendParityFn = std::function<void(ParityShard &&)>;

	struct Frame
	{
		uint8_t stream_index = 0;
		uint64_t frame_index = 0;
		// An IDR goes over the control (TCP) socket, like every WiVRn encoder
		// does with its key frames (server/encoder/ffmpeg/video_encoder_ffmpeg.cpp:100,
		// video_encoder_nvenc.cpp:618). Losing one costs the session a round trip
		// and a black screen; losing a P frame costs one frame.
		bool idr = false;
		// False for a TCP-only session, where there is nothing to fragment for.
		bool has_stream_socket = true;
		// ... unless the frame is being paced. A single 200 kB write to a TCP
		// socket is exactly the burst that wedges an access point's queue and then
		// head-of-line blocks everything behind it, and a whole-frame shard gives
		// the pacer nothing to spread. Cutting a TCP frame into shards costs the
		// per-shard header (about 2% at 1400 byte payloads) and buys a frame that
		// is handed to the kernel over milliseconds instead of at once. Upstream
		// never has to make this trade: it does not pace a TCP path at all
		// (video_encoder.cpp:130-134), because on Linux the stream socket is
		// always there and TCP is only ever the failover.
		bool fragment_on_control = false;
		// Ask for parity shards. Only honoured for a frame that actually rides
		// the stream socket, see fec_active().
		bool fec = false;

		Shard::view_info_t view_info{};
		Shard::timing_info_t timing_info{};

		// video_encoder.cpp:563. Parity is worth something only on the lossy path.
		bool fec_active() const
		{
			return fec and has_stream_socket and not idr;
		}
	};

	// Sends a whole frame at once, with no pacing. Returns the number of data
	// shards sent. Kept for the callers (and the tests) that have no clock.
	static uint16_t send(const Frame & frame, std::span<uint8_t> data, const SendFn & send_one);

	// The payload budget one shard of this frame gets, before view_info is taken
	// out of it. Exposed for the test.
	static size_t payload_budget(const Frame & frame);
};

// One frame being handed to the sockets a micro-burst at a time.
//
// Owns no memory: `data` must outlive the send, which is what the session's queue
// of EncodedFrames guarantees.
class PacedSender
{
public:
	using Frame = VideoPacketizer::Frame;
	using Shard = VideoPacketizer::Shard;
	using ParityShard = VideoPacketizer::ParityShard;

	struct Sinks
	{
		VideoPacketizer::SendFn data;
		// May be empty: no parity is then built even for a frame that asked
		// for it.
		VideoPacketizer::SendParityFn parity;
		// Called on the last shard of the frame, just before it goes out, so that
		// send_end names the moment the frame really finished leaving rather than
		// the moment the first shard did — pacing spreads those over milliseconds
		// (video_encoder.cpp:589-596 takes the timestamp in the same place and for
		// the same reason). May be empty.
		std::function<void(Shard::timing_info_t &)> stamp;
	};

	// Starts a frame. `pacer` is the schedule its bytes go out on; a default
	// constructed one means "all at once", which is what an IDR on the control
	// socket and a frame smaller than one micro-burst get.
	void begin(const Frame & frame, std::span<uint8_t> data, const ShardPacer & pacer);

	// A frame is loaded and not finished.
	bool active() const
	{
		return active_;
	}

	// Hands over every shard whose deadline has passed, and always at least one
	// micro-burst, so a caller with a stopped clock still makes progress.
	//
	// Returns the absolute time the next burst is due, or 0 when the frame is
	// finished (which is also when active() goes false) — the caller then starts
	// the next frame or goes back to waiting.
	int64_t pump(int64_t now_ns, const Sinks & sinks);

	// The socket threw mid-frame: the rest of this frame is worthless (the headset
	// can never complete it) and the next pump() must not continue it.
	void abort()
	{
		active_ = false;
	}

	// Counters for the frame that has just finished.
	uint16_t shards() const
	{
		return shards_;
	}
	uint16_t parity_shards() const
	{
		return parity_shards_;
	}
	// Bytes actually put on the wire for this frame, payloads and parity payloads,
	// which is the unit BitrateController::on_frame_bytes is expressed in
	// (video_encoder.cpp:697).
	uint32_t wire_bytes() const
	{
		return wire_bytes_;
	}

private:
	void send_parity(const Sinks & sinks);

	Frame frame_{};
	std::span<uint8_t> data_{};
	ShardPacer pacer_{};
	wivrn::fec::group_builder group_;

	bool active_ = false;
	bool fec_active_ = false;
	size_t offset_ = 0;
	Shard shard_{};
	uint16_t shards_ = 0;
	uint16_t parity_shards_ = 0;
	uint32_t wire_bytes_ = 0;
};

// head ∘ eye: the eye's pose in the tracking space, from the HMD pose the frame
// was rendered with and the eye-to-head transform the client reported. This is
// what the Linux compositor gets for free from the projection layer's per-view
// pose (server/compositor/compositor.cpp:693); FrameReady only carries the head,
// because that is all SteamVR hands the shim at SubmitLayer.
inline XrPosef compose_pose(const XrPosef & head, const XrPosef & eye)
{
	const XrQuaternionf & a = head.orientation;
	const XrQuaternionf & b = eye.orientation;

	XrPosef out{};
	out.orientation.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	out.orientation.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	out.orientation.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	out.orientation.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

	// a * v * a^-1, written out rather than through a matrix.
	const float vx = eye.position.x;
	const float vy = eye.position.y;
	const float vz = eye.position.z;
	const float tx = 2.f * (a.y * vz - a.z * vy);
	const float ty = 2.f * (a.z * vx - a.x * vz);
	const float tz = 2.f * (a.x * vy - a.y * vx);

	out.position.x = head.position.x + vx + a.w * tx + (a.y * tz - a.z * ty);
	out.position.y = head.position.y + vy + a.w * ty + (a.z * tx - a.x * tz);
	out.position.z = head.position.z + vz + a.w * tz + (a.x * ty - a.y * tx);
	return out;
}

// The neutral foveation map for an encoded image of this size: one run of
// `width` output pixels at ratio 1, and the same for the height.
//
// Not optional. The headset sizes its reprojection swapchain from
// count_pixels() over these vectors and asserts the run count is odd
// (client/scenes/stream_defoveator.cpp:512-513, :653-664), so an empty parameter
// is a zero-sized swapchain and a failed assertion, not "no foveation". A single
// run is the identity: n_ratio is 0, every run's ratio is |0 - 0| + 1, and the
// defoveation pass emits one quad covering the whole image.
inline wivrn::to_headset::foveation_parameter identity_foveation(uint16_t width, uint16_t height)
{
	wivrn::to_headset::foveation_parameter p;
	p.x = {width};
	p.y = {height};
	return p;
}

} // namespace wivrnnx::helper
