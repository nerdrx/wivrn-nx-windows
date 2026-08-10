// The video path's own vocabulary: what an encoded frame is, what the encoder is
// asked for, and the interface the frame intake drives.
//
// Nothing here knows about D3D11, AMF, winsock or wivrn_packets. That is what
// lets both halves of the helper include it: pipe_server.cpp (a plain Win32
// translation unit) fills these structs, session.cpp (which lives behind
// winsock2.h and the POSIX shadow headers) reads them, and neither has to see
// the other's headers.
#pragma once

#include <cstdint>
#include <vector>

#include "wivrnnx_ipc.h"

namespace wivrnnx::helper
{

// The two codecs this phase can produce. Mapped onto wivrn::video_codec in
// wivrn/video_out.cpp, which is the only place that enum may be named.
enum class VideoCodec : uint8_t
{
	h264,
	h265,
};

const char * video_codec_name(VideoCodec codec);

// One encoded picture of one eye: annex-B, whole frame, ready to be cut into
// shards. The pose travels with it because the headset reprojects against the
// pose the frame was *rendered* with, not against the newest one.
struct EncodedFrame
{
	uint64_t frame_id = 0;
	// FrameReady.sample_time_qpc: when Present ran, on vrserver's QPC, which is
	// the same clock the tracking path stamps poses with.
	uint64_t sample_time_qpc = 0;
	// SteamVR's own prediction interval for this frame, seconds.
	float predict_s = 0.f;
	// HMD pose at Present, from FrameReady. Orientation is (w, x, y, z).
	float pose_q[4] = {1.f, 0.f, 0.f, 0.f};
	float pose_p[3] = {0.f, 0.f, 0.f};

	uint8_t stream_index = 0; // 0 left eye, 1 right eye
	bool idr = false;
	std::vector<uint8_t> data;
};

struct EncoderConfig
{
	float refresh_hz = 90.f;
	// Bits per second on the wire for the whole stream, i.e. both eyes together.
	// The encoder splits it; the bitrate controller and the command line both talk
	// about the link, not about one component.
	uint32_t bitrate_bps = 50'000'000;
	bool allow_h265 = true;
	bool allow_h264 = true;
};

// What the encoder ended up producing, which is what the client has to be told
// in to_headset::video_stream_description. Per eye.
struct EncoderStreamInfo
{
	uint32_t width = 0;
	uint32_t height = 0;
	VideoCodec codec = VideoCodec::h265;

	bool operator==(const EncoderStreamInfo &) const = default;
};

enum class EncodeResult
{
	ok,      // frames appended to `out`
	dropped, // this frame did not make it; the session sees nothing
	fatal,   // the encoder has to be torn down and rebuilt
};

// One staging ring's worth of encoding. Rebuilt from scratch on every
// StagingConfig generation.
class IVideoEncoder
{
public:
	virtual ~IVideoEncoder();

	// `vrserver_pid` comes from ShimHello and is what the NT handles in
	// `staging` are valid in.
	virtual bool configure(const ipc::StagingConfig & staging,
	                       uint32_t vrserver_pid,
	                       const EncoderConfig & config) = 0;
	virtual void shutdown() = 0;

	// A new target for a running encoder, whole stream (both eyes), bits per
	// second. Must not rebuild anything: this is called whenever the automatic
	// bitrate control moves, which on a bad link is once a second.
	virtual void set_bitrate(uint32_t bitrate_bps) = 0;

	virtual EncoderStreamInfo stream_info() const = 0;

	// Acquires the staging slot, encodes both eyes, releases it. `out` is
	// appended to, one EncodedFrame per eye, in stream order.
	virtual EncodeResult encode(const ipc::FrameReady & frame,
	                            bool force_idr,
	                            std::vector<EncodedFrame> & out) = 0;
};

} // namespace wivrnnx::helper
