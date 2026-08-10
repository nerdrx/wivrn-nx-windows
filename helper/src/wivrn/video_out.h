// Cutting one encoded frame into video_stream_data_shards.
//
// Copy-adapted from wivrn::video_encoder::SendData
// (server/encoder/video_encoder.cpp:534-706). What is kept is the whole of what
// the headset's reassembler depends on: the shard index runs from zero, the
// first shard of a frame carries view_info and no other one does, the last shard
// carries timing_info and no other one does, and the payload budget is the FEC
// budget minus whatever view_info costs on the shard that has it. The headset's
// shard_set::complete() is exactly those three rules read backwards
// (client/decoder/shard_set.h:93-102) — there is no explicit end-of-frame flag on
// this wire, the presence of timing_info on the last shard *is* the flag.
//
// What is dropped, deliberately and for this phase only: forward error
// correction (the parity group builder and the 64-byte payload reserve it costs),
// the packet pacer, and the multipath spill scheduler. The headset tolerates a
// stream with no parity — it only ever reconstructs from parity shards it
// actually receives (client/decoder/shard_set.h:127-152) — and none of the three
// changes where a shard boundary falls, only when and over which socket it goes
// out.
//
// Deliberately free of Windows: this is the one file of the video path that can
// be compiled and driven natively, which is what src/tests/shard_test.cpp does.
#pragma once

#include <cstdint>
#include <functional>
#include <span>

#include "wivrn_packets.h"

namespace wivrnnx::helper
{

class VideoPacketizer
{
public:
	using Shard = wivrn::to_headset::video_stream_data_shard;

	// `prefer_control` asks for the reliable socket. The shard's payload span
	// points into the caller's buffer and is only valid for the duration of the
	// call, exactly as it is upstream.
	using SendFn = std::function<void(Shard &&, bool prefer_control)>;

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

		Shard::view_info_t view_info{};
		Shard::timing_info_t timing_info{};
	};

	// Returns the number of shards sent.
	static uint16_t send(const Frame & frame, std::span<uint8_t> data, const SendFn & send_one);

	// The payload budget one shard of this frame gets, before view_info is taken
	// out of it. Exposed for the test.
	static size_t payload_budget(const Frame & frame);
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
