#include "amf_video_encoder.h"

#include "../log.h"

namespace wivrnnx::helper
{

AmfVideoEncoder::~AmfVideoEncoder()
{
	shutdown();
}

bool AmfVideoEncoder::configure(const ipc::StagingConfig & staging,
                                uint32_t vrserver_pid,
                                const EncoderConfig & config)
{
	shutdown();

	const int surface_format = amf_surface_format_for_dxgi(staging.dxgi_format);
	if (surface_format == 0)
	{
		log_line("encoder: staging textures are DXGI format %u, which this phase cannot encode",
		         staging.dxgi_format);
		return false;
	}

	// Chroma is subsampled 2:1 in both directions, so both dimensions have to be
	// even. A per-eye width is half of an already even ring width and is even by
	// construction; an odd render height is possible and costs one row, which is
	// a great deal better than no video at all. Whatever comes out of here is
	// what the client is told to size its decoders to, so the two cannot drift.
	// 16-align both dimensions: AMF's AVC encoder rejects non-macroblock-
	// aligned Init (seen live: 1188x1188 -> AMF_RESULT 3 on the RX 580), and
	// HEVC merely tolerated 1456 because it happened to be 16-aligned. The
	// crop loses at most 15 rows/columns; the stream description follows the
	// encoded size automatically.
	const uint32_t eye_width = (staging.width / 2u) & ~15u;
	const uint32_t eye_height = staging.height & ~15u;
	if (eye_height != staging.height)
		log_line("encoder: staging height %u is odd, encoding %u rows", staging.height, eye_height);
	if (eye_width == 0 || eye_height == 0)
	{
		log_line("encoder: staging ring %ux%u leaves nothing to encode", staging.width, staging.height);
		return false;
	}

	if (not stage_.create_device())
		return false;
	if (not stage_.open_ring(staging, vrserver_pid))
		return false;
	if (not context_.init(stage_.device()))
		return false;

	AmfEncodeParams params{};
	params.width = eye_width;
	params.height = eye_height;
	params.refresh_hz = config.refresh_hz;
	params.bitrate_bps = config.bitrate_bps;
	params.surface_format = surface_format;
	params.allow_h265 = config.allow_h265;
	params.allow_h264 = config.allow_h264;

	static const char * const labels[kStreamCount] = {"left", "right"};
	for (uint32_t i = 0; i < kStreamCount; ++i)
	{
		if (not encoders_[i].open(context_, params, labels[i]))
		{
			shutdown();
			return false;
		}
	}

	// The two components are created from the same parameters, so they agree by
	// construction — but the fallback to H.264 is decided per component, and a
	// stream pair with different codecs is not something the description can
	// express (one video_codec per item, but the headset would then need two
	// different decoders where the picture must match).
	if (encoders_[0].codec() != encoders_[1].codec())
	{
		log_line("encoder: eyes ended up on different codecs (%s and %s), refusing",
		         video_codec_name(encoders_[0].codec()),
		         video_codec_name(encoders_[1].codec()));
		shutdown();
		return false;
	}

	info_ = EncoderStreamInfo{eye_width, eye_height, encoders_[0].codec()};
	ready_ = true;
	frames_encoded_ = 0;
	frames_dropped_ = 0;
	return true;
}

void AmfVideoEncoder::shutdown()
{
	for (AmfStreamEncoder & e: encoders_)
		e.close();
	context_.terminate();
	stage_.close_ring();
	stage_.destroy_device();
	info_ = EncoderStreamInfo{};
	ready_ = false;
}

EncodeResult AmfVideoEncoder::encode(const ipc::FrameReady & frame,
                                     bool force_idr,
                                     std::vector<EncodedFrame> & out)
{
	if (not ready_)
		return EncodeResult::fatal;
	if (frame.generation != stage_.generation())
		return EncodeResult::dropped;

	ID3D11Texture2D * source = stage_.acquire(frame.staging_index, kAcquireTimeoutMs);
	if (source == nullptr)
	{
		++frames_dropped_;
		return EncodeResult::dropped;
	}

	// Zero copy versus one copy, decided here.
	//
	// AMFContext::CreateSurfaceFromDX11Native would wrap the staging texture with
	// no copy at all, and that is what a single full-frame encoder would want.
	// It cannot work here for two independent reasons. The wrapped surface is the
	// whole side-by-side pair, and an AMF encoder encodes a whole surface — there
	// is no source rectangle, so an eye cannot be cut out of it. And the wrap
	// would have to outlive the encode, which means holding the slot's keyed
	// mutex for the entire encode of both eyes rather than for two texture
	// copies; the shim would then be skipping slots on every frame.
	//
	// So: one CopySubresourceRegion per eye into an AMF-allocated surface, which
	// is also what ALVR does with its single encoder (VideoEncoderAMF.cpp:744-749,
	// AllocSurface then CopyResource). Two copies of half a frame each is one
	// frame's worth of bandwidth on a GPU that is about to read the same pixels
	// through the encoder anyway.
	const uint32_t eye_width = info_.width;
	const uint32_t eye_height = info_.height;

	EncodeResult submitted[kStreamCount]{};
	for (uint32_t i = 0; i < kStreamCount; ++i)
	{
		submitted[i] = encoders_[i].submit(
		        [&](void * native) {
			        return stage_.copy_region(source,
			                                  static_cast<ID3D11Texture2D *>(native),
			                                  i * eye_width,
			                                  eye_width,
			                                  eye_height);
		        },
		        force_idr);
	}

	// Both copies are on the wire to the GPU before the slot goes back, so the
	// shim cannot start overwriting it under a copy that has not been recorded
	// yet. The keyed mutex orders the GPU work either way; this only makes sure
	// the work has actually been submitted.
	stage_.flush();
	stage_.release(frame.staging_index);

	bool any_fatal = false;
	std::vector<EncodedFrame> produced;
	produced.reserve(kStreamCount);

	for (uint32_t i = 0; i < kStreamCount; ++i)
	{
		if (submitted[i] == EncodeResult::fatal)
		{
			any_fatal = true;
			continue;
		}
		if (submitted[i] != EncodeResult::ok)
			continue;

		EncodedFrame encoded{};
		encoded.frame_id = frame.frame_id;
		encoded.sample_time_qpc = frame.sample_time_qpc;
		encoded.predict_s = frame.predict_s;
		encoded.pose_q[0] = frame.qw;
		encoded.pose_q[1] = frame.qx;
		encoded.pose_q[2] = frame.qy;
		encoded.pose_q[3] = frame.qz;
		encoded.pose_p[0] = frame.px;
		encoded.pose_p[1] = frame.py;
		encoded.pose_p[2] = frame.pz;
		encoded.stream_index = static_cast<uint8_t>(i);

		const EncodeResult got = encoders_[i].retrieve(encoded.data, encoded.idr);
		if (got == EncodeResult::fatal)
		{
			any_fatal = true;
			continue;
		}
		if (got != EncodeResult::ok)
			continue;

		produced.push_back(std::move(encoded));
	}

	if (any_fatal)
		return EncodeResult::fatal;

	// A frame with only one eye is worse than no frame: the headset joins the two
	// streams on a common frame index and would sit on the half it got until the
	// window rolled past it. Drop the pair.
	if (produced.size() < kStreamCount)
	{
		++frames_dropped_;
		return EncodeResult::dropped;
	}

	for (EncodedFrame & f: produced)
		out.push_back(std::move(f));

	if (++frames_encoded_ % 300 == 0)
		log_line("encoder: %llu frames encoded, %llu dropped (%llu of them keyed-mutex timeouts), "
		         "last encode %.1f / %.1f ms",
		         static_cast<unsigned long long>(frames_encoded_),
		         static_cast<unsigned long long>(frames_dropped_),
		         static_cast<unsigned long long>(stage_.acquire_timeouts()),
		         encoders_[0].last_encode_ms(),
		         encoders_[1].last_encode_ms());

	return EncodeResult::ok;
}

} // namespace wivrnnx::helper
