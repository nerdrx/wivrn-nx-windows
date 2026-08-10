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
#pragma once

#include <cstdint>

#include "wivrn_packets.h"

namespace wivrnnx::helper
{

class IdrTracker
{
public:
	// A new session: the client's decoders are empty.
	void reset()
	{
		state_ = State::need_idr;
		idr_id_ = 0;
		first_p_ = 0;
	}

	// Called once per frame put on the wire, per stream. Idempotent for the two
	// eyes of one frame, which are always both IDR or both not.
	void on_frame_sent(uint64_t frame_index, bool idr)
	{
		if (idr)
		{
			state_ = State::wait_idr_feedback;
			idr_id_ = frame_index;
		}
		else if (state_ == State::idr_received)
		{
			state_ = State::running;
			first_p_ = frame_index;
		}
	}

	// True when this feedback means the next frame has to be a key frame. Only
	// ever true on the transition, so a burst of feedback about the same loss
	// asks for one IDR and not twenty.
	bool on_feedback(const wivrn::from_headset::feedback & f)
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
				return true;

			case State::running:
				// idr_handler.cpp:48-55.
				if (f.sent_to_decoder || f.frame_index < first_p_)
					return false;
				state_ = State::need_idr;
				return true;
		}
		return false;
	}

	bool waiting_for_idr() const
	{
		return state_ == State::need_idr;
	}

private:
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
};

} // namespace wivrnnx::helper
