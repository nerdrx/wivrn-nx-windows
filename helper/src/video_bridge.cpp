#include "video_bridge.h"

#include "log.h"

namespace wivrnnx::helper
{

void VideoBridge::set_prefs(uint32_t bitrate_bps, int codec, bool adaptive)
{
	std::lock_guard lock(mutex_);
	pref_bitrate_bps_ = bitrate_bps;
	pref_codec_ = codec;
	pref_adaptive_ = adaptive;
	request_.bitrate_bps = bitrate_bps;
}

VideoBridge::Prefs VideoBridge::prefs() const
{
	std::lock_guard lock(mutex_);
	return Prefs{pref_bitrate_bps_, pref_codec_, pref_adaptive_};
}

void VideoBridge::set_bitrate(uint32_t bitrate_bps)
{
	std::lock_guard lock(mutex_);
	if (not request_.active or bitrate_bps == 0 or bitrate_bps == request_.bitrate_bps)
		return;

	request_.bitrate_bps = bitrate_bps;
	// Deliberately not config_generation: a bitrate change must not rebuild the
	// encoder. AMF takes TARGET/PEAK_BITRATE and the VBV size at runtime, and a
	// rebuild would cost a parameter set, an IDR and a stream description round
	// trip every time the controller moved.
	++request_.bitrate_generation;
}

void VideoBridge::set_client(float refresh_hz, bool allow_h265, bool allow_h264)
{
	std::lock_guard lock(mutex_);

	// Command-line codec override wins over the negotiation, but never
	// enables a codec the client did not offer.
	if (pref_codec_ == 1)
		allow_h265 = false;
	else if (pref_codec_ == 2)
		allow_h264 = false;

	const bool changed = not request_.active ||
	                     request_.refresh_hz != refresh_hz ||
	                     request_.allow_h265 != allow_h265 ||
	                     request_.allow_h264 != allow_h264;

	request_.active = true;
	request_.refresh_hz = refresh_hz;
	// A new session starts at the ceiling, whatever the last one's controller had
	// walked the bitrate down to.
	if (request_.bitrate_bps != pref_bitrate_bps_)
	{
		request_.bitrate_bps = pref_bitrate_bps_;
		++request_.bitrate_generation;
	}
	request_.allow_h265 = allow_h265;
	request_.allow_h264 = allow_h264;

	if (changed)
	{
		++request_.config_generation;
		// A client that has just been given an encoder has never seen a
		// parameter set; the first frame it gets has to carry one.
		++request_.idr_generation;
	}
}

void VideoBridge::clear_client()
{
	std::lock_guard lock(mutex_);
	if (not request_.active)
		return;
	request_.active = false;
	// Whatever was encoded but not yet sent belongs to a session that is over.
	frames_.clear();
}

void VideoBridge::request_idr(const char * reason)
{
	{
		std::lock_guard lock(mutex_);
		++request_.idr_generation;
	}
	log_line("video: IDR requested (%s)", reason);
}

VideoRequest VideoBridge::request() const
{
	std::lock_guard lock(mutex_);
	return request_;
}

size_t VideoBridge::take_frames(std::vector<EncodedFrame> & out)
{
	std::lock_guard lock(mutex_);
	const size_t n = frames_.size();
	for (EncodedFrame & f: frames_)
		out.push_back(std::move(f));
	frames_.clear();
	return n;
}

VideoBridge::StreamState VideoBridge::stream_state() const
{
	std::lock_guard lock(mutex_);
	return stream_;
}

void VideoBridge::publish_stream(const EncoderStreamInfo & info, bool valid)
{
	std::lock_guard lock(mutex_);
	if (stream_.valid == valid && stream_.info == info)
		return;
	stream_.info = info;
	stream_.valid = valid;
	++stream_.generation;
}

void VideoBridge::push_frames(std::vector<EncodedFrame> && frames)
{
	if (frames.empty())
		return;

	std::lock_guard lock(mutex_);
	if (not request_.active)
		return;

	for (EncodedFrame & f: frames)
	{
		frames_.push_back(std::move(f));
		++frames_queued_;
	}

	// Oldest first, and by whole frames: dropping one eye of a pair would leave
	// the headset joining on a frame index it can never complete.
	while (frames_.size() > kMaxQueued)
	{
		const uint64_t victim = frames_.front().frame_id;
		while (not frames_.empty() && frames_.front().frame_id == victim)
		{
			frames_.pop_front();
			++frames_dropped_;
		}
	}
}

uint64_t VideoBridge::frames_queued() const
{
	std::lock_guard lock(mutex_);
	return frames_queued_;
}

uint64_t VideoBridge::frames_dropped_in_queue() const
{
	std::lock_guard lock(mutex_);
	return frames_dropped_;
}

} // namespace wivrnnx::helper
