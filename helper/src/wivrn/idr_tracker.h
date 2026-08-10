// When the stream needs a key frame.
//
// Port of wivrn::default_idr_handler (server/encoder/idr_handler.{h,cpp}), with
// the same four states and the same transitions. The stream carries no periodic
// IDR at all — the encoder is configured with an infinite GOP — so every key
// frame in a session is one this decided on:
//
//   * the first frame after a client connects, because a decoder that has never
//     seen a parameter set produces nothing,
//   * a frame the headset says it never finished receiving, because with no
//     forward error correction yet a single lost shard makes the frame
//     undecodable and every P frame after it references a picture the headset
//     does not have.
//
// Two things upstream has are left out. `should_skip`, which stops the encoder
// producing anything at all between an IDR and its feedback, is not worth the
// plumbing here: IDRs go out on the control (TCP) socket, so unlike a shard on
// UDP they cannot be lost, and the frames that would have been skipped reference
// an IDR that is certain to arrive. And `non_ref_frames`, which excuses a lost
// frame that nothing references, has nothing to describe: MAX_NUM_REFRAMES is 0,
// so every frame is referenced by the next one.
//
// --- IDR damping -----------------------------------------------------------
// One thing is added, and it is not upstream's. Live on the RX 580 this tracker
// fired 534 times in a few minutes: a frame arrives incomplete, the tracker asks
// for a key frame, the key frame is itself a burst of a few hundred kilobytes on
// a link that is already late, the burst loses the frame behind it, and round it
// goes. Upstream does not have that loop because it does not have the burst:
// server/encoder/shard_pacer.h spreads every frame over a fraction of a frame
// period and server/driver/bitrate_controller.cpp lowers the bitrate before the
// frames start arriving late at all. Both of those are now in this port
// (shard_pacer.h, bitrate_controller.h) and they are the actual cure.
//
// What upstream *does* have that damps the loop directly is `should_skip`: while
// it waits for the feedback of a key frame it emits nothing at all, so the link
// gets a quiet stretch to deliver that key frame in (idr_handler.cpp:68-86). The
// equivalent is not available here — the frames are produced by an encoder thread
// that is one FrameReady behind and has no way to be told "produce nothing" for a
// round trip without also dropping the shim's staging slots — so instead there is
// a floor on how often a *recovery* IDR may be asked for. A request that arrives
// inside the floor is not dropped, it is held and issued when the floor expires,
// so a genuinely undecodable stream still recovers; it just recovers at 2 Hz
// instead of at the frame rate. The first IDR of a session and the one a new
// stream description asks for do not go through here at all and are never delayed.
#pragma once

#include <chrono>
#include <cstdint>

#include "wivrn_packets.h"

namespace wivrnnx::helper
{

class IdrTracker
{
public:
	using clock = std::chrono::steady_clock;

	// Shortest interval between two recovery IDRs. Two frame periods would be the
	// physical minimum (a request cannot be answered before the encoder's next
	// frame and the feedback for it cannot come back before one round trip);
	// half a second is a deliberate multiple of that, chosen so that a link bad
	// enough to lose most frames spends its bandwidth on P frames that may get
	// through rather than on key frames that certainly will not.
	static constexpr std::chrono::milliseconds min_recovery_interval{500};

	// A new session: the client's decoders are empty.
	void reset()
	{
		state_ = State::need_idr;
		idr_id_ = 0;
		first_p_ = 0;
		pending_ = false;
		last_request_ = {};
		damped_ = 0;
	}

	// Called once per frame put on the wire, per stream. Idempotent for the two
	// eyes of one frame, which are always both IDR or both not.
	void on_frame_sent(uint64_t frame_index, bool idr)
	{
		if (idr)
		{
			state_ = State::wait_idr_feedback;
			idr_id_ = frame_index;
			// The key frame is on the wire; whatever asked for it has been served.
			pending_ = false;
		}
		else if (state_ == State::idr_received)
		{
			state_ = State::running;
			first_p_ = frame_index;
		}
	}

	// True when this feedback means the next frame has to be a key frame, and
	// enough time has passed since the last one for asking to be useful. Only ever
	// true on the transition, so a burst of feedback about the same loss asks for
	// one IDR and not twenty; a transition inside min_recovery_interval is held
	// and comes back out of poll().
	bool on_feedback(const wivrn::from_headset::feedback & f, clock::time_point now = clock::now())
	{
		switch (state_)
		{
			case State::need_idr:
			case State::idr_received:
				return false;

			case State::wait_idr_feedback:
				// idr_handler.cpp:34-47. Feedback for any other frame says
				// nothing about whether the key frame landed.
				if (f.frame_index != idr_id_)
					return false;
				if (f.sent_to_decoder)
				{
					state_ = State::idr_received;
					return false;
				}
				state_ = State::need_idr;
				return request(now);

			case State::running:
				// idr_handler.cpp:48-55.
				if (f.sent_to_decoder || f.frame_index < first_p_)
					return false;
				state_ = State::need_idr;
				return request(now);
		}
		return false;
	}

	// True when a request that was held back by min_recovery_interval may go out
	// now. Called once per session loop iteration.
	bool poll(clock::time_point now = clock::now())
	{
		if (not pending_)
			return false;
		if (now - last_request_ < min_recovery_interval)
			return false;

		pending_ = false;
		last_request_ = now;
		return true;
	}

	bool waiting_for_idr() const
	{
		return state_ == State::need_idr;
	}

	// How many requests the floor has held back, for the periodic report.
	uint64_t damped() const
	{
		return damped_;
	}

private:
	// Grants a recovery IDR, or holds it until the floor expires.
	bool request(clock::time_point now)
	{
		if (last_request_ != clock::time_point{} and now - last_request_ < min_recovery_interval)
		{
			if (not pending_)
				++damped_;
			pending_ = true;
			return false;
		}

		pending_ = false;
		last_request_ = now;
		return true;
	}

	enum class State
	{
		need_idr,
		wait_idr_feedback,
		idr_received,
		running,
	};

	State state_ = State::need_idr;
	uint64_t idr_id_ = 0;
	uint64_t first_p_ = 0;

	// A recovery IDR is owed but the floor has not expired yet.
	bool pending_ = false;
	clock::time_point last_request_{};
	uint64_t damped_ = 0;
};

} // namespace wivrnnx::helper
