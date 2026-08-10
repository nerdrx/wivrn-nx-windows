#include "video_out.h"

#include <algorithm>
#include <limits>

#include "fec.h"
#include "wivrn_serialization.h"

namespace wivrnnx::helper
{

size_t VideoPacketizer::payload_budget(const Frame & frame)
{
	// video_encoder.cpp:565. The control socket is TCP: it does its own
	// fragmentation and a shard there may be the whole frame, which is what makes
	// an IDR one datagram-shaped object instead of two hundred.
	if (not frame.has_stream_socket || frame.idr)
		return std::numeric_limits<size_t>::max();

	// FEC is off in this phase, so this is data_shard::max_payload_size (1400)
	// with nothing reserved. Asking fec:: for it rather than writing 1400 keeps
	// the two ends tied to the same constant when parity is grafted on.
	return wivrn::fec::shard_payload_budget(false);
}

uint16_t VideoPacketizer::send(const Frame & frame, std::span<uint8_t> data, const SendFn & send_one)
{
	// video_encoder.cpp:103 - the sender skips a frame with no bytes rather than
	// emitting a shard for it. A zero-length frame would also be un-completable
	// on the headset, since complete() needs a last shard to exist at all.
	if (data.empty())
		return 0;

	const size_t budget = payload_budget(frame);
	const bool control = frame.idr;

	// video_encoder.cpp:456-461: the shard is a template that is mutated as the
	// frame is walked.
	Shard shard{};
	shard.stream_item_idx = frame.stream_index;
	shard.frame_idx = frame.frame_index;
	shard.shard_idx = 0;
	shard.view_info = frame.view_info;
	shard.timing_info.reset();

	size_t offset = 0;
	while (offset < data.size())
	{
		// video_encoder.cpp:571. view_info only exists on the first shard, so from
		// the second one on this is the whole budget.
		const size_t overhead = wivrn::serialized_size(shard.view_info);
		size_t payload_size = budget > overhead ? budget - overhead : 0;
		if (payload_size == 0)
		{
			// video_encoder.cpp:574-585: unreachable with today's budgets, but a
			// view_info that grew to fill a shard would otherwise spin here
			// forever. One byte of progress beats a hang.
			payload_size = 1;
		}

		// Upstream writes std::min(end, begin + payload_size) with payload_size
		// set to uint32 max on the control path, which is an iterator computed
		// past the end. Clamping the length instead gives the same split without
		// the arithmetic.
		const size_t remaining = data.size() - offset;
		const size_t take = std::min(payload_size, remaining);
		const bool last = take == remaining;

		if (last)
		{
			// video_encoder.cpp:588-596. This, and only this, is what tells the
			// headset the frame is finished.
			shard.timing_info = frame.timing_info;
		}

		shard.payload = data.subspan(offset, take);
		send_one(Shard{shard}, control);

		++shard.shard_idx;
		// video_encoder.cpp:661.
		shard.view_info.reset();
		offset += take;
	}

	return shard.shard_idx;
}

} // namespace wivrnnx::helper
