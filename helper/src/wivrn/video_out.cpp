#include "video_out.h"

#include <algorithm>
#include <limits>

#include "wivrn_serialization.h"

namespace wivrnnx::helper
{

size_t VideoPacketizer::payload_budget(const Frame & frame)
{
	// video_encoder.cpp:565. The control socket is TCP: it does its own
	// fragmentation and a shard there may be the whole frame, which is what makes
	// an IDR one datagram-shaped object instead of two hundred. An IDR is also the
	// one frame that is never paced — the session is waiting on it — so it is
	// never cut up either.
	if (frame.idr || (not frame.has_stream_socket && not frame.fragment_on_control))
		return std::numeric_limits<size_t>::max();

	// data_shard::max_payload_size, less fec::payload_reserve when parity is on:
	// a parity shard must not be a bigger datagram than the data shards it covers
	// (fec.h:60-75).
	return wivrn::fec::shard_payload_budget(frame.fec_active());
}

void PacedSender::begin(const Frame & frame, std::span<uint8_t> data, const ShardPacer & pacer)
{
	frame_ = frame;
	data_ = data;
	pacer_ = pacer;
	offset_ = 0;
	shards_ = 0;
	parity_shards_ = 0;
	wire_bytes_ = 0;
	fec_active_ = frame.fec_active();

	// video_encoder.cpp:456-461: the shard is a template that is mutated as the
	// frame is walked.
	shard_ = Shard{};
	shard_.stream_item_idx = frame.stream_index;
	shard_.frame_idx = frame.frame_index;
	shard_.shard_idx = 0;
	shard_.view_info = frame.view_info;
	shard_.timing_info.reset();

	group_.reset(frame.stream_index, frame.frame_index);

	// video_encoder.cpp:103 - the sender skips a frame with no bytes rather than
	// emitting a shard for it. A zero-length frame would also be un-completable on
	// the headset, since complete() needs a last shard to exist at all.
	active_ = not data.empty();
}

void PacedSender::send_parity(const Sinks & sinks)
{
	// The group is always closed, whether or not its parity is worth sending: the
	// builder's next group has to start at the next shard either way
	// (video_encoder.cpp:499-507).
	auto parity = group_.take();
	if (not parity or not sinks.parity)
		return;

	wire_bytes_ += uint32_t(parity->payload.size());
	++parity_shards_;

	// video_encoder.cpp:524-531: a network error costs this frame its protection
	// and nothing else. The session's own poll loop is what decides a connection
	// is over.
	try
	{
		sinks.parity(std::move(*parity));
	}
	catch (const std::exception &)
	{
	}
}

int64_t PacedSender::pump(int64_t now_ns, const Sinks & sinks)
{
	if (not active_)
		return 0;

	const size_t budget = VideoPacketizer::payload_budget(frame_);
	const bool control = frame_.idr;

	while (offset_ < data_.size())
	{
		// video_encoder.cpp:571. view_info only exists on the first shard, so from
		// the second one on this is the whole budget.
		const size_t overhead = wivrn::serialized_size(shard_.view_info);
		size_t payload_size = budget > overhead ? budget - overhead : 0;
		if (payload_size == 0)
		{
			// video_encoder.cpp:574-585: unreachable with today's budgets, but a
			// view_info that grew to fill a shard would otherwise spin here
			// forever. One byte of progress beats a hang.
			payload_size = 1;
		}

		// Upstream writes std::min(end, begin + payload_size) with payload_size set
		// to uint32 max on the control path, which is an iterator computed past the
		// end. Clamping the length instead gives the same split without the
		// arithmetic.
		const size_t remaining = data_.size() - offset_;
		const size_t take = std::min(payload_size, remaining);
		const bool last = take == remaining;

		if (last)
		{
			// video_encoder.cpp:588-596. This, and only this, is what tells the
			// headset the frame is finished.
			shard_.timing_info = frame_.timing_info;
			if (sinks.stamp)
				sinks.stamp(*shard_.timing_info);
		}

		shard_.payload = data_.subspan(offset_, take);
		wire_bytes_ += uint32_t(take);

		// The parity builder sees the shard before it is handed to the socket:
		// send() consumes the payload span in place (the socket encrypts through
		// it), so the blob has to be taken while it still holds plaintext.
		if (fec_active_)
			group_.add(shard_);

		sinks.data(Shard{shard_}, control);

		++shard_.shard_idx;
		++shards_;
		// video_encoder.cpp:661.
		shard_.view_info.reset();
		offset_ += take;

		// video_encoder.cpp:652-658. The parity shard of a group goes out
		// immediately after the group's last data shard, so it travels in (or right
		// at the edge of) the same pacing micro-burst and reaches the headset while
		// the group is still open there.
		//
		// block_full() was full() before common/fec.h gained the interleaved,
		// adaptive layout. This builder is never handed a set_layout(), so its depth
		// stays at the default of 1 and a block is exactly one group of
		// fec::group_size shards — the same point this fired at before.
		if (fec_active_ and group_.block_full())
			send_parity(sinks);

		// Leaky bucket: hold the next micro-burst back until the frame's schedule
		// says it may go out. Never past the frame's budget, and the budget is a
		// fraction of a frame period, so this can never push the end of the frame
		// into the next one.
		//
		// Parity bytes are not counted into the schedule: the frame's own bytes
		// still take exactly the window they were given, and the parity rides
		// alongside. The bitrate accounting already took the overhead out of the
		// encoder (see fec::data_share), so the link sees the same rate either way.
		if (offset_ < data_.size())
		{
			if (auto at = pacer_.wait_until(offset_, now_ns))
				return *at;
		}
	}

	// Last group of the frame, usually a partial one. Emitted even for a group of
	// a single shard: a one-shard frame is cheap to duplicate in absolute bytes and
	// losing it costs exactly as much as losing a big one — a frame plus the IDR
	// round trip it triggers (video_encoder.cpp:677-682).
	if (fec_active_)
		send_parity(sinks);

	active_ = false;
	return 0;
}

uint16_t VideoPacketizer::send(const Frame & frame, std::span<uint8_t> data, const SendFn & send_one)
{
	PacedSender sender;
	sender.begin(frame, data, ShardPacer{});

	PacedSender::Sinks sinks{
	        .data = send_one,
	        // No parity and no re-stamped send_end: this overload has no clock and
	        // no socket to put a parity shard on.
	        .parity = {},
	        .stamp = {},
	};

	// An unpaced sender finishes in one call by construction: ShardPacer{} is
	// inactive and wait_until() never returns a deadline.
	sender.pump(0, sinks);
	return sender.shards();
}

} // namespace wivrnnx::helper
