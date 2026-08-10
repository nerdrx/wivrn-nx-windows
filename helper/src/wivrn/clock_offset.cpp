#include "clock_offset.h"

#include "win_net.h" // winsock2 before anything reaches windows.h

#include <windows.h>

#include <cmath>

namespace wivrnnx::helper
{

namespace
{

int64_t qpc_frequency()
{
	static const int64_t freq = [] {
		LARGE_INTEGER f{};
		QueryPerformanceFrequency(&f);
		return f.QuadPart != 0 ? f.QuadPart : 10'000'000;
	}();
	return freq;
}

} // namespace

int64_t monotonic_ns()
{
	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	// 128-bit intermediate: at the usual 10 MHz the product overflows 64 bits
	// after about three minutes of uptime.
	return static_cast<int64_t>((static_cast<__int128>(now.QuadPart) * 1'000'000'000) / qpc_frequency());
}

uint64_t ns_to_qpc(int64_t ns)
{
	if (ns < 0)
		return 0;
	return static_cast<uint64_t>((static_cast<__int128>(ns) * qpc_frequency()) / 1'000'000'000);
}

int64_t qpc_to_ns(uint64_t qpc)
{
	return static_cast<int64_t>((static_cast<__int128>(qpc) * 1'000'000'000) / qpc_frequency());
}

bool ClockOffsetEstimator::should_sample(std::chrono::steady_clock::time_point now)
{
	std::lock_guard lock(mutex_);
	if (now < next_sample_)
		return false;
	next_sample_ = now + sample_interval_;
	return true;
}

void ClockOffsetEstimator::reset()
{
	std::lock_guard lock(mutex_);
	samples_.clear();
	sample_index_ = 0;
	b_ = 0;
	next_sample_ = {};
	sample_interval_ = std::chrono::milliseconds(10);
}

void ClockOffsetEstimator::add_sample(const wivrn::from_headset::timesync_response & base_sample)
{
	const int64_t now = monotonic_ns();
	Sample sample{base_sample, now};

	std::lock_guard lock(mutex_);

	if (samples_.size() < kNumSamples)
	{
		samples_.push_back(sample);
	}
	else
	{
		sample_interval_ = std::chrono::milliseconds(100);

		int64_t latency = 0;
		for (const Sample & s: samples_)
			latency += s.received - s.query;
		latency /= static_cast<int64_t>(samples_.size());

		// A round trip three times the average is almost certainly a
		// retransmission; it would drag the regression with it.
		if (sample.received - sample.query > 3 * latency)
			return;

		samples_[sample_index_] = sample;
		sample_index_ = (sample_index_ + 1) % kNumSamples;
	}

	// Linear regression of headset time against server time, on values
	// recentred about their means so the fit keeps its precision.
	const size_t n = samples_.size();
	const double inv_n = 1.0 / static_cast<double>(n);
	double x0 = 0;
	double y0 = 0;
	for (const Sample & s: samples_)
	{
		x0 += static_cast<double>(s.query + s.received) * 0.5;
		y0 += static_cast<double>(s.response);
	}
	x0 *= inv_n;
	y0 *= inv_n;

	if (samples_.size() < kNumSamples)
	{
		int64_t b = static_cast<int64_t>(y0 - x0);
		b &= ~static_cast<int64_t>(1);
		b_ = b;
		return;
	}

	double sum_x = 0;
	double sum_y = 0;
	for (const Sample & s: samples_)
	{
		// Symmetrical latency assumed, as upstream does.
		sum_x += static_cast<double>(s.query + s.received) * 0.5 - x0;
		sum_y += static_cast<double>(s.response) - y0;
	}

	const double mean_x = sum_x * inv_n;
	const double mean_y = sum_y * inv_n;
	const double b = y0 + (mean_y - mean_x) - x0;

	const bool stable = std::abs(b - static_cast<double>(b_.load())) < 20'000'000;

	int64_t new_b = static_cast<int64_t>(b);
	if (stable)
		new_b |= 1;
	else
		new_b &= ~static_cast<int64_t>(1);
	b_ = new_b;
}

ClockOffset ClockOffsetEstimator::get_offset() const
{
	const int64_t b = b_.load();
	return ClockOffset{b, (b & 1) != 0};
}

} // namespace wivrnnx::helper
