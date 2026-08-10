#include "video_intake.h"

#include "log.h"

namespace wivrnnx::helper
{

namespace
{
// ipc::FrameDone::flags
constexpr uint32_t kDoneEncoded = 0;
constexpr uint32_t kDoneDropped = 1;

// Bounded so that a shim which has stopped reading cannot make this grow. Three
// is the whole ring; anything past that is a shim that is not coming back.
constexpr size_t kMaxDone = 8;
} // namespace

VideoIntake::VideoIntake(VideoBridge & bridge, std::unique_ptr<IVideoEncoder> encoder) :
        bridge_(bridge),
        encoder_(std::move(encoder))
{
}

VideoIntake::~VideoIntake()
{
	stop();
}

void VideoIntake::start()
{
	if (thread_.joinable())
		return;
	thread_ = std::thread([this] { run(); });
}

void VideoIntake::stop()
{
	{
		std::lock_guard lock(mutex_);
		stop_ = true;
	}
	cv_.notify_all();
	if (thread_.joinable())
		thread_.join();
}

void VideoIntake::set_vrserver_pid(uint32_t pid)
{
	std::lock_guard lock(mutex_);
	vrserver_pid_ = pid;
}

void VideoIntake::on_staging_config(const ipc::StagingConfig & config)
{
	{
		std::lock_guard lock(mutex_);
		// A frame for the ring that has just been replaced can never be encoded;
		// the shim ignores a stale FrameDone anyway, but leaving it un-answered
		// would be a slot the shim waits on forever if the generation check on
		// its side ever loosened.
		if (pending_ && pending_->generation != config.generation)
		{
			push_done(pending_->frame_id, pending_->staging_index, kDoneDropped);
			pending_.reset();
		}
		staging_ = config;
		staging_changed_ = true;
	}
	cv_.notify_all();

	log_line("staging config: generation %u, %u slots, %ux%u, DXGI format %u",
	         config.generation,
	         config.count,
	         config.width,
	         config.height,
	         config.dxgi_format);
}

void VideoIntake::on_frame_ready(const ipc::FrameReady & frame)
{
	bool wake = false;
	{
		std::lock_guard lock(mutex_);
		++frames_seen_;

		if (not staging_ || frame.generation != staging_->generation ||
		    frame.staging_index >= staging_->count)
		{
			// Nothing this frame refers to exists here. Answer it so the shim
			// does not sit on the slot.
			push_done(frame.frame_id, frame.staging_index, kDoneDropped);
			++frames_dropped_;
		}
		else if (encoder_ == nullptr || not bridge_.request().active)
		{
			// No client, or a build with no encoder at all (--fake): the cheap
			// path, no thread wakeup, no GPU.
			push_done(frame.frame_id, frame.staging_index, kDoneDropped);
			++frames_dropped_;
		}
		else
		{
			if (pending_)
			{
				push_done(pending_->frame_id, pending_->staging_index, kDoneDropped);
				++frames_dropped_;
			}
			pending_ = frame;
			wake = true;
		}
	}

	if (wake)
		cv_.notify_all();
}

void VideoIntake::on_shim_gone()
{
	std::deque<ipc::FrameDone> discarded;
	{
		std::lock_guard lock(mutex_);
		staging_.reset();
		staging_changed_ = true;
		pending_.reset();
		// FrameDone for a shim that is gone has nowhere to go, and the next one
		// to connect gets a fresh ring.
		discarded.swap(done_);
	}
	cv_.notify_all();
}

bool VideoIntake::pop_done(ipc::FrameDone & out)
{
	std::lock_guard lock(mutex_);
	if (done_.empty())
		return false;
	out = done_.front();
	done_.pop_front();
	return true;
}

// Called with mutex_ held.
void VideoIntake::push_done(uint64_t frame_id, uint32_t staging_index, uint32_t flags)
{
	if (done_.size() >= kMaxDone)
		done_.pop_front();

	ipc::FrameDone done{};
	done.frame_id = frame_id;
	done.staging_index = staging_index;
	done.flags = flags;
	done_.push_back(done);
}

void VideoIntake::run()
{
	for (;;)
	{
		ipc::FrameReady frame{};
		{
			std::unique_lock lock(mutex_);
			cv_.wait(lock, [this] { return stop_ || pending_.has_value() || staging_changed_; });
			if (stop_)
				break;

			if (staging_changed_)
			{
				staging_changed_ = false;
				// Tear the encoder down here, on this thread, rather than
				// lazily on the next frame: the textures it holds belong to a
				// process that may already be gone.
				if (encoder_ != nullptr && encoder_ready_)
				{
					lock.unlock();
					encoder_->shutdown();
					bridge_.publish_stream(EncoderStreamInfo{}, false);
					lock.lock();
					encoder_ready_ = false;
					encoder_generation_ = 0;
				}
			}

			if (not pending_)
				continue;
			frame = *pending_;
			pending_.reset();
		}

		encode_one(frame);
	}

	if (encoder_ != nullptr)
		encoder_->shutdown();
}

void VideoIntake::ensure_encoder(const ipc::StagingConfig & config, uint32_t vrserver_pid, const VideoRequest & request)
{
	if (encoder_ready_ && encoder_generation_ == config.generation &&
	    encoder_config_generation_ == request.config_generation)
		return;

	if (encoder_ready_)
	{
		encoder_->shutdown();
		encoder_ready_ = false;
		bridge_.publish_stream(EncoderStreamInfo{}, false);
	}

	EncoderConfig cfg{};
	cfg.refresh_hz = request.refresh_hz;
	cfg.bitrate_bps = request.bitrate_bps;
	cfg.allow_h265 = request.allow_h265;
	cfg.allow_h264 = request.allow_h264;

	if (not encoder_->configure(config, vrserver_pid, cfg))
	{
		// Do not retry on every frame: a missing amfrt64.dll or an unopenable
		// handle is not going to fix itself, and the log would drown. The next
		// StagingConfig (a SteamVR restart, a mode change) tries again.
		encoder_config_generation_ = request.config_generation;
		encoder_generation_ = config.generation;
		return;
	}

	encoder_ready_ = true;
	encoder_generation_ = config.generation;
	encoder_config_generation_ = request.config_generation;
	idr_seen_ = 0; // a brand new encoder owes the client a parameter set
	// configure() was given request.bitrate_bps, so the components already carry
	// this generation's bitrate.
	bitrate_seen_ = request.bitrate_generation;

	const EncoderStreamInfo info = encoder_->stream_info();
	bridge_.publish_stream(info, true);
	log_line("video: encoding %ux%u per eye as %s", info.width, info.height, video_codec_name(info.codec));
}

void VideoIntake::encode_one(const ipc::FrameReady & frame)
{
	ipc::StagingConfig config{};
	uint32_t pid = 0;
	{
		std::lock_guard lock(mutex_);
		if (not staging_)
		{
			push_done(frame.frame_id, frame.staging_index, kDoneDropped);
			return;
		}
		config = *staging_;
		pid = vrserver_pid_;
	}

	const VideoRequest request = bridge_.request();
	if (not request.active)
	{
		std::lock_guard lock(mutex_);
		push_done(frame.frame_id, frame.staging_index, kDoneDropped);
		++frames_dropped_;
		return;
	}

	ensure_encoder(config, pid, request);
	if (not encoder_ready_)
	{
		std::lock_guard lock(mutex_);
		push_done(frame.frame_id, frame.staging_index, kDoneDropped);
		++frames_dropped_;
		return;
	}

	// The automatic bitrate moved: hand the new target to the running components
	// before this frame is encoded. No rebuild, see AmfStreamEncoder::set_bitrate.
	if (request.bitrate_generation != bitrate_seen_)
	{
		bitrate_seen_ = request.bitrate_generation;
		encoder_->set_bitrate(request.bitrate_bps);
	}

	const bool force_idr = request.idr_generation != idr_seen_;

	std::vector<EncodedFrame> encoded;
	const EncodeResult result = encoder_->encode(frame, force_idr, encoded);

	if (result == EncodeResult::fatal)
	{
		log_line("video: encoder failed on frame %llu, tearing it down",
		         static_cast<unsigned long long>(frame.frame_id));
		encoder_->shutdown();
		encoder_ready_ = false;
		bridge_.publish_stream(EncoderStreamInfo{}, false);
	}

	if (result == EncodeResult::ok)
	{
		if (force_idr)
			idr_seen_ = request.idr_generation;
		bridge_.push_frames(std::move(encoded));
	}

	std::lock_guard lock(mutex_);
	if (result == EncodeResult::ok)
	{
		++frames_encoded_;
		push_done(frame.frame_id, frame.staging_index, kDoneEncoded);
	}
	else
	{
		++frames_dropped_;
		push_done(frame.frame_id, frame.staging_index, kDoneDropped);
	}
}

uint64_t VideoIntake::frames_seen() const
{
	std::lock_guard lock(mutex_);
	return frames_seen_;
}

uint64_t VideoIntake::frames_encoded() const
{
	std::lock_guard lock(mutex_);
	return frames_encoded_;
}

uint64_t VideoIntake::frames_dropped() const
{
	std::lock_guard lock(mutex_);
	return frames_dropped_;
}

} // namespace wivrnnx::helper
