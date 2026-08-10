#include "synthetic_video.h"

#include <algorithm>
#include <chrono>

#include "../log.h"
#include "clock_offset.h"

namespace wivrnnx::helper
{

namespace
{
// Deterministic, and shaped like an annex-B stream so that anything downstream
// that ever looks at the first bytes sees something legal. Nothing decodes it.
void fill_bitstream(std::vector<uint8_t> & out, size_t bytes, bool idr, uint32_t seed)
{
	out.clear();
	out.reserve(bytes);
	out.insert(out.end(), {0x00, 0x00, 0x00, 0x01});
	out.push_back(static_cast<uint8_t>((idr ? 19 : 1) << 1));
	out.push_back(0x01);

	uint32_t x = seed * 2654435761u + 1u;
	while (out.size() < bytes)
	{
		x = x * 1664525u + 1013904223u;
		uint8_t b = static_cast<uint8_t>(x >> 24);
		if (b == 0x00)
			b = 0x11;
		out.push_back(b);
	}
}
} // namespace

SyntheticVideo::SyntheticVideo(VideoBridge & bridge, uint32_t eye_width, uint32_t eye_height) :
        bridge_(bridge),
        width_(eye_width),
        height_(eye_height)
{
}

SyntheticVideo::~SyntheticVideo()
{
	stop();
}

void SyntheticVideo::start()
{
	if (thread_.joinable())
		return;
	thread_ = std::thread([this] { run(); });
}

void SyntheticVideo::stop()
{
	stop_.store(true);
	if (thread_.joinable())
		thread_.join();
}

void SyntheticVideo::run()
{
	using clock = std::chrono::steady_clock;

	uint64_t frame_id = 0;
	uint64_t idr_seen = 0;
	bool published = false;
	auto next = clock::now();
	std::vector<uint8_t> scratch;

	log_line("synthetic video: %ux%u per eye, following the controller's bitrate", width_, height_);

	while (not stop_.load())
	{
		const VideoRequest request = bridge_.request();

		if (not request.active)
		{
			if (published)
			{
				bridge_.publish_stream(EncoderStreamInfo{}, false);
				published = false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			next = clock::now();
			continue;
		}

		if (not published)
		{
			// The session sends a video_stream_description off the back of this,
			// which is what makes the client start its decoders.
			bridge_.publish_stream(EncoderStreamInfo{width_, height_, VideoCodec::h265}, true);
			published = true;
			idr_seen = 0;
		}

		const float hz = request.refresh_hz > 1.f ? request.refresh_hz : 90.f;
		const auto period = std::chrono::nanoseconds(int64_t(1e9 / double(hz)));

		const bool idr = request.idr_generation != idr_seen;
		idr_seen = request.idr_generation;

		// Half the link budget per eye, one frame period's worth of it. This is
		// what makes the harness able to see the controller work: the frames get
		// smaller when it lowers the bitrate and bigger when it raises it, exactly
		// as a CBR encoder's would.
		size_t bytes = size_t(double(request.bitrate_bps) / 8.0 / double(hz) / 2.0);
		bytes = std::clamp<size_t>(bytes, 512, 4u * 1024 * 1024);
		// An IDR is a few times a P frame, which is what makes it the burst the
		// pacer and the IDR floor both exist to deal with.
		if (idr)
			bytes *= 4;

		const uint64_t qpc = ns_to_qpc(monotonic_ns());

		std::vector<EncodedFrame> frames;
		frames.reserve(2);
		for (uint8_t eye = 0; eye < 2; ++eye)
		{
			EncodedFrame f{};
			f.frame_id = frame_id;
			f.sample_time_qpc = qpc;
			f.predict_s = 1.f / hz;
			f.pose_q[0] = 1.f;
			f.stream_index = eye;
			f.idr = idr;
			fill_bitstream(scratch, bytes, idr, uint32_t(frame_id * 2 + eye));
			f.data = scratch;
			frames.push_back(std::move(f));
		}

		bridge_.push_frames(std::move(frames));
		++frame_id;

		next += period;
		const auto now = clock::now();
		if (next > now)
			std::this_thread::sleep_for(next - now);
		else
			next = now;
	}
}

} // namespace wivrnnx::helper
