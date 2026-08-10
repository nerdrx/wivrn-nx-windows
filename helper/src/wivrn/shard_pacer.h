// Leaky-bucket schedule for one frame's worth of video shards.
//
// Copy-adapted from wivrn::shard_pacer and wivrn::pacing_slot
// (server/encoder/shard_pacer.h:45-204). The arithmetic is unchanged, constant
// for constant: group_bytes, min_sleep_ns, max_window, deadline(), wait_until()
// and pacing_slot::begin_frame() are the upstream ones, so a frame is cut into
// the same micro-bursts here as on Linux and tests/pacing_test.cpp's numbers
// still describe it.
//
// What is dropped is spill_scheduler, upstream's multipath striping split:
// there is one path in this port and nothing to spill onto.
//
// What is different is who sleeps. Upstream owns a sender thread per socket and
// blocks it in clock_nanosleep between two micro-bursts. Here the shards go out
// from the session thread, which is also the thread that reads tracking off the
// headset and feeds the Bridge the pipe server drains — sleeping on it would
// hold poses back by exactly the time pacing spreads a frame over. So nothing
// in this port ever sleeps for pacing: PacedSender (video_out.h) hands out the
// bursts that are due and reports when the next one is, and the session's poll()
// timeout is shortened to that. Same schedule, no blocked thread.
//
// Pure arithmetic, no clock and no syscalls, exactly as upstream: the caller
// supplies the current time. src/tests/shard_test.cpp drives it on a virtual one.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace wivrnnx::helper
{

class ShardPacer
{
public:
	// shard_pacer.h:56. Bytes handed to the kernel back to back between two
	// pauses: 8-9 datagrams, small enough to fit one Wi-Fi TXOP, large enough
	// that the wakeup rate stays sane and the driver can still aggregate.
	static constexpr size_t group_bytes = 12 * 1024;

	// shard_pacer.h:63. Below this a pause is not worth taking: it is under what
	// a timer wakeup costs and under the slack a non-realtime thread gets anyway.
	static constexpr int64_t min_sleep_ns = 150'000;

	// shard_pacer.h:73. Hard ceiling on the configured window fraction. The
	// bitrate controller reads link utilisation as the fraction of a frame period
	// a frame took to arrive, so pacing over a fraction w puts a floor of w under
	// every utilisation sample; its "spare capacity, probe upwards" threshold is
	// 0.60 (bitrate_controller.h:212) and a window near it would park the
	// controller in its hysteresis band forever.
	static constexpr float max_window = 0.5f;

	// An inactive pacer: never pauses.
	ShardPacer() = default;

	ShardPacer(int64_t start_ns, int64_t budget_ns, size_t total_bytes) :
	        start(start_ns),
	        budget(std::max<int64_t>(budget_ns, 0)),
	        total(total_bytes)
	{}

	// False when there is nothing to spread: no budget left (a late frame, or a
	// frame slot whose window is already spent), or a frame that fits in a single
	// group and is harmless as it is.
	bool active() const
	{
		return budget > 0 and total > group_bytes;
	}

	int64_t window_ns() const
	{
		return active() ? budget : 0;
	}

	// Absolute time at which the byte at `offset` may go out. Monotone
	// non-decreasing in offset, equal to the start time at offset 0, and never
	// later than start + budget.
	int64_t deadline(size_t offset) const
	{
		if (not active())
			return start;
		if (offset >= total)
			return start + budget;
		return start + int64_t(budget * uint64_t(offset) / total);
	}

	// Number of micro-bursts the frame is split into. There is one pause before
	// each of them but the first.
	size_t group_count() const
	{
		if (not active())
			return 1;
		return (total + group_bytes - 1) / group_bytes;
	}

	// Deadline of the first byte of group `index`.
	int64_t group_deadline(size_t index) const
	{
		return deadline(std::min(index * group_bytes, total));
	}

	// Called by the sender after every shard, with the running count of bytes
	// already handed to the kernel and the current time. Returns the absolute
	// time the next shard may go out at, or nothing at all to keep sending back
	// to back.
	std::optional<int64_t> wait_until(size_t sent_bytes, int64_t now)
	{
		if (not active() or sent_bytes < next_group or sent_bytes >= total)
			return {};

		// Skip whole groups rather than pausing once per group that was crossed:
		// a single shard never spans a group, but a caller handing over larger
		// chunks must not be paced several times over.
		next_group = (sent_bytes / group_bytes + 1) * group_bytes;

		const int64_t at = deadline(sent_bytes);
		if (at - now < min_sleep_ns)
			return {};

		return at;
	}

private:
	int64_t start = 0;
	int64_t budget = 0;
	size_t total = 0;
	size_t next_group = group_bytes;
};

// Pacing state of one socket, i.e. of one sender.
//
// shard_pacer.h:162-204, unchanged. The window is a property of the frame slot,
// not of a single frame: both eyes are pushed into the same queue and drained by
// the same thread, so two streams each paced over 40% of the frame period would
// take 80% of it. Instead the slot owns one window and the frames in it share
// what is left.
class PacingSlot
{
public:
	// Budget for a frame that starts going out at `now`, with `queued` further
	// frames already waiting behind it.
	//
	// The first frame of a slot opens it and gets the whole window. Frames that
	// follow within the same slot get an equal share of whatever is left of it,
	// and once the window is spent they get nothing and are blasted out. A frame
	// that arrives a whole period after the slot opened starts a new one.
	int64_t begin_frame(int64_t now, int64_t frame_period_ns, float window, size_t queued)
	{
		if (frame_period_ns <= 0 or window <= 0)
			return 0;

		const int64_t full = int64_t(frame_period_ns * std::clamp(window, 0.f, ShardPacer::max_window));

		if (not slot_open or now - slot_start >= frame_period_ns)
		{
			slot_start = now;
			slot_open = true;
		}

		const int64_t left = slot_start + full - now;
		if (left <= 0)
			return 0;

		return left / int64_t(queued + 1);
	}

	void reset()
	{
		slot_open = false;
	}

private:
	// Monotonic time the current frame slot opened
	int64_t slot_start = 0;
	bool slot_open = false;
};

} // namespace wivrnnx::helper
