// Client <-> server clock synchronisation.
//
// Copy-adapted from server/driver/clock_offset.{h,cpp}. The estimator, the
// sample window and the linear regression are unchanged; what differs is the
// server clock underneath it. Monado's os_monotonic_get_ns() becomes a
// QueryPerformanceCounter converted to nanoseconds, so that the result can be
// turned straight back into the QPC ticks the IPC contract stamps into every
// PoseUpdate — vrserver wants QPC, and a round trip through any other clock
// would add an offset of its own on top of the one we are measuring.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

#include "wivrn_packets.h"

namespace wivrnnx::helper
{

// Nanoseconds since an arbitrary origin, derived from QueryPerformanceCounter.
int64_t monotonic_ns();

// The inverse: server nanoseconds back to raw QPC ticks. Exact, because both
// directions go through the same performance-counter frequency.
uint64_t ns_to_qpc(int64_t ns);

// And the other way, for the timestamps the video path gets handed. FrameReady
// carries vrserver's QueryPerformanceCounter at Present; the shard's view_info
// wants headset nanoseconds, and this is the first half of that conversion (the
// second is ClockOffset::to_headset). Same clock, same frequency, so a pose and
// a frame stamped in the same millisecond stay in the same millisecond.
int64_t qpc_to_ns(uint64_t qpc);

// y: headset time, x: server time, y = x + b.
struct ClockOffset
{
	int64_t b = 0;
	bool stable = false;

	explicit operator bool() const
	{
		return stable;
	}

	int64_t from_headset(int64_t headset_ns) const
	{
		return headset_ns - b;
	}

	int64_t to_headset(int64_t server_ns) const
	{
		return server_ns + b;
	}
};

class ClockOffsetEstimator
{
public:
	// True when it is time to send another to_headset::timesync_query.
	bool should_sample(std::chrono::steady_clock::time_point now);

	void add_sample(const wivrn::from_headset::timesync_response & sample);

	ClockOffset get_offset() const;

	void reset();

private:
	struct Sample : wivrn::from_headset::timesync_response
	{
		int64_t received;
	};

	static constexpr size_t kNumSamples = 100;

	mutable std::mutex mutex_;
	std::vector<Sample> samples_;
	size_t sample_index_ = 0;
	// Least significant bit doubles as the "stable" flag, exactly as upstream
	// does, so the whole offset can be read with one atomic load.
	std::atomic<int64_t> b_ = 0;

	std::chrono::steady_clock::time_point next_sample_{};
	std::chrono::milliseconds sample_interval_{10};
};

} // namespace wivrnnx::helper
