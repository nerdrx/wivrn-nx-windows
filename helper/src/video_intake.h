// Frame intake: the StagingConfig/FrameReady/FrameDone half of protocol v3.
//
// The pipe read loop calls in here and must never wait on anything: it is the
// same loop that carries poses to vrserver, and a pose that arrives late is a
// pose that made the headset judder. So everything expensive — opening the
// shared textures, building the AMF components, the encode itself — happens on
// the thread this owns, and the pipe loop only ever hands over a struct and
// picks up whatever FrameDone messages are waiting.
//
// Queue depth is one frame. A FrameReady that arrives while another is still
// being encoded replaces it, and the one it replaced is answered with
// FrameDone flags=1 straight away, which is what tells the shim it may reuse
// that slot. Nothing is ever queued behind the encoder: the shim already skips
// slots it has not got back, and a queue here would only add latency to a link
// that is already behind.
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "encoder/video_encoder.h"
#include "video_bridge.h"
#include "wivrnnx_ipc.h"

namespace wivrnnx::helper
{

class VideoIntake
{
public:
	// A null encoder is the "this build cannot encode" case (no WiVRn half): every
	// FrameReady is then answered with FrameDone flags=1, which is exactly what
	// --fake wants.
	VideoIntake(VideoBridge & bridge, std::unique_ptr<IVideoEncoder> encoder);
	~VideoIntake();

	VideoIntake(const VideoIntake &) = delete;
	VideoIntake & operator=(const VideoIntake &) = delete;

	void start();
	void stop();

	// --- called from the pipe read loop ------------------------------------

	// From ShimHello. The NT handles in StagingConfig are only meaningful with it.
	void set_vrserver_pid(uint32_t pid);

	void on_staging_config(const ipc::StagingConfig & config);
	void on_frame_ready(const ipc::FrameReady & frame);

	// The shim went away: SteamVR restarted, or vrserver died. Every texture we
	// opened out of it is gone with it.
	void on_shim_gone();

	// FrameDone messages waiting to go back to the shim. Non-blocking.
	bool pop_done(ipc::FrameDone & out);

	// --- counters, for the pipe server's heartbeat -------------------------

	uint64_t frames_seen() const;
	uint64_t frames_encoded() const;
	uint64_t frames_dropped() const;

private:
	void run();
	// Everything under `mutex_` is released before this is called.
	void encode_one(const ipc::FrameReady & frame);
	void ensure_encoder(const ipc::StagingConfig & config, uint32_t vrserver_pid, const VideoRequest & request);
	void push_done(uint64_t frame_id, uint32_t staging_index, uint32_t flags);

	VideoBridge & bridge_;
	std::unique_ptr<IVideoEncoder> encoder_;

	mutable std::mutex mutex_;
	std::condition_variable cv_;
	bool stop_ = false;

	uint32_t vrserver_pid_ = 0;
	std::optional<ipc::StagingConfig> staging_;   // latest from the shim
	bool staging_changed_ = false;                // not applied to the encoder yet
	std::optional<ipc::FrameReady> pending_;      // at most one
	std::deque<ipc::FrameDone> done_;

	// Owned by the encoder thread only.
	bool encoder_ready_ = false;
	uint32_t encoder_generation_ = 0;
	uint64_t encoder_config_generation_ = 0;
	uint64_t idr_seen_ = 0;
	// Last VideoRequest::bitrate_generation applied to the live encoder.
	uint64_t bitrate_seen_ = 0;

	uint64_t frames_seen_ = 0;
	uint64_t frames_encoded_ = 0;
	uint64_t frames_dropped_ = 0;

	std::thread thread_;
};

} // namespace wivrnnx::helper
