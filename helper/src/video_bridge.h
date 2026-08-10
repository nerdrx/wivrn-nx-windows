// The video seam, the mirror image of bridge.h.
//
// Poses run WiVRn session -> pipe. Frames run the other way: the shim produces
// them, the encoder thread turns them into bitstreams, the WiVRn session cuts
// them into shards and puts them on the socket. Both directions have to cross
// the same wall, because pipe_server.cpp must not see winsock2.h and session.cpp
// must not see the raw Win32 pipe. So this header, like bridge.h, pulls in
// nothing but the standard library, the frozen contract, and the video path's
// own vocabulary.
//
// Frames are queued, not latest-value: half a frame is worth nothing and the
// headset joins the two eye streams on a common frame index, so a pair either
// goes out whole or is dropped whole. The queue is short and drops the oldest
// pair on overflow, which is the same rule the Linux sender uses
// (server/encoder/video_encoder.cpp:180-187).
#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "encoder/video_encoder.h"

namespace wivrnnx::helper
{

// What the WiVRn session asks the encoder for. Read by the encoder thread on
// every frame, written by the session thread when a client comes and goes.
struct VideoRequest
{
	// No client, no encoding: FrameReady is answered with FrameDone flags=1 and
	// nothing touches the GPU.
	bool active = false;
	float refresh_hz = 90.f;
	// What the encoder should be producing right now. With the automatic bitrate
	// on this moves under the session's feet, which is what bitrate_generation is
	// for: the encoder thread applies it to the live AMF components without
	// rebuilding them (a rebuild would cost an IDR and a stream description).
	uint32_t bitrate_bps = 50'000'000;
	uint64_t bitrate_generation = 0;
	bool allow_h265 = true;
	bool allow_h264 = true;

	// Bumped by the session whenever the next frame must be an IDR: a client
	// just connected, or the headset's feedback says it lost one.
	uint64_t idr_generation = 0;

	// Bumped whenever the encoder must be rebuilt from scratch even though the
	// staging ring has not changed — a new client with a different refresh rate.
	uint64_t config_generation = 0;
};

class VideoBridge
{
public:
	// --- WiVRn session side ------------------------------------------------

	// A client connected. Anything that changes the encoder's parameters bumps
	// config_generation, so the encoder thread rebuilds on its own schedule.
	void set_client(float refresh_hz, bool allow_h265, bool allow_h264);
	void clear_client();

	// The next encoded frame of every stream must be an IDR.
	void request_idr(const char * reason);

	VideoRequest request() const;

	// Process-wide overrides from the command line, applied on top of whatever
	// the client negotiates in set_client(). codec: 0 = auto, 1 = force H.264,
	// 2 = force HEVC. `adaptive` is --no-adaptive read the right way up. Call
	// once at startup, before any client connects.
	void set_prefs(uint32_t bitrate_bps, int codec, bool adaptive);

	// What the command line asked for. The bitrate is a *ceiling*: the automatic
	// control works below it and never above.
	struct Prefs
	{
		uint32_t ceiling_bps = 50'000'000;
		int codec = 0;
		bool adaptive = true;
	};
	Prefs prefs() const;

	// The bitrate the controller decided, in bits per second on the wire for both
	// eyes together. Applied to the running encoder, no rebuild.
	void set_bitrate(uint32_t bitrate_bps);

	// Drains everything the encoder has produced. Appends; returns how many.
	size_t take_frames(std::vector<EncodedFrame> & out);

	// What the encoder is actually producing, for video_stream_description.
	// `generation` moves whenever it changes, so the session can tell a fresh
	// description from a redundant one without comparing structs it does not own.
	struct StreamState
	{
		EncoderStreamInfo info{};
		bool valid = false;
		uint64_t generation = 0;
	};
	StreamState stream_state() const;

	// --- encoder thread side -----------------------------------------------

	void publish_stream(const EncoderStreamInfo & info, bool valid);
	void push_frames(std::vector<EncodedFrame> && frames);

	// --- counters ----------------------------------------------------------

	uint64_t frames_queued() const;
	uint64_t frames_dropped_in_queue() const;

private:
	// Two eyes of two frames. Deeper than that is latency nobody asked for: the
	// shim already drops frames when the helper is behind, and a queue here would
	// only hide that.
	static constexpr size_t kMaxQueued = 4;

	mutable std::mutex mutex_;
	VideoRequest request_{};
	uint32_t pref_bitrate_bps_ = 50'000'000;
	int pref_codec_ = 0;
	bool pref_adaptive_ = true;
	StreamState stream_{};
	std::deque<EncodedFrame> frames_;
	uint64_t frames_queued_ = 0;
	uint64_t frames_dropped_ = 0;
};

} // namespace wivrnnx::helper
