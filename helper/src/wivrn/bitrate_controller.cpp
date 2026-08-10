// Transcription of server/driver/bitrate_controller.cpp. Line references in the
// comments are into that file; U_LOG_I becomes log_line, and the multipath
// entry points (set_path_ceiling, set_combined) and the transport_status
// snapshot are dropped — see the header.
#include "bitrate_controller.h"

#include "../log.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace wivrnnx::helper
{

namespace
{
constexpr double to_mbits = 1e-6;

const char * mode_name(BitrateController::mode m)
{
	return m == BitrateController::mode::bbr ? "bandwidth estimation (v2)" : "AIMD (v1)";
}
} // namespace

void BitrateController::configure(const config & c, uint32_t ceiling_bps, bool client_enabled_, bool radio_aware_, std::optional<mode> client_mode_)
{
	std::lock_guard lock(mutex);

	conf = c;
	client_enabled = client_enabled_;
	radio_aware = radio_aware_;
	client_mode = client_mode_;
	ceiling = ceiling_bps;
	// A ceiling below the configured minimum wins: the ceiling is always honoured.
	min_bitrate = std::min(conf.min_bitrate_bps, ceiling ? ceiling : conf.min_bitrate_bps);
	bitrate = ceiling;
	recovery_target = ceiling;
	st = state::steady;
	first_recovery_step = true;
	frames = {};
	has_frames = false;
	stale_before = 0;
	flush();
	flush_radio();
	flush_estimator();

	if (conf.enabled and client_enabled and ceiling)
		log_line("video: automatic bitrate enabled, %s, ceiling %.1f Mbit/s, floor %.1f Mbit/s, radio trend %s",
		         mode_name(mode_locked()),
		         ceiling * to_mbits,
		         min_bitrate * to_mbits,
		         radio_aware ? "on" : "off");
	else if (ceiling)
		log_line("video: automatic bitrate disabled (%s), fixed %.1f Mbit/s",
		         conf.enabled ? "headset switch" : "--no-adaptive",
		         ceiling * to_mbits);
}

bool BitrateController::enabled() const
{
	std::lock_guard lock(mutex);
	return conf.enabled and client_enabled;
}

BitrateController::mode BitrateController::mode_locked() const
{
	// The headset's selector wins whenever it expressed one; the configuration is the default
	// for a headset that never touched it.
	return client_mode.value_or(conf.control);
}

BitrateController::mode BitrateController::active_mode() const
{
	std::lock_guard lock(mutex);
	return mode_locked();
}

BitrateController::status BitrateController::snapshot() const
{
	std::lock_guard lock(mutex);

	status s{
	        .bitrate_bps = bitrate,
	        .ceiling_bps = ceiling,
	        .control = mode_locked(),
	};

	if (mode_locked() == mode::bbr and bandwidth.valid() and bandwidth_samples >= estimator_min_samples)
		s.estimate_bps = uint32_t(std::min<double>(bandwidth.get(), std::numeric_limits<uint32_t>::max()));

	if (not(conf.enabled and client_enabled))
	{
		// Nothing is controlling anything: the bitrate is the one the operator set, and
		// neither state machine has any meaning to report.
		s.state = state_name::off;
		return s;
	}

	if (s.control == mode::bbr)
	{
		switch (bbr_st)
		{
			case bbr_state::startup:
				s.state = state_name::startup;
				break;
			case bbr_state::steady:
				s.state = state_name::steady;
				break;
			case bbr_state::probe:
				s.state = state_name::probe;
				break;
		}
	}
	else
	{
		s.state = st == state::recovering ? state_name::recovering : state_name::steady;
	}

	// The hold is a latch that outlives the switch being turned off.
	s.radio_hold = radio_hold and radio_aware;

	return s;
}

std::optional<uint32_t> BitrateController::set_client_mode(std::optional<mode> m)
{
	std::lock_guard lock(mutex);

	if (m == client_mode)
		return {};

	const mode before = mode_locked();
	client_mode = m;
	const mode after = mode_locked();

	if (before == after)
		return {};

	log_line("video: automatic bitrate control law is now %s", mode_name(after));

	// The two laws do not measure the same thing. Start over, from the ceiling, both ways.
	return reset_locked();
}

void BitrateController::set_pacing_window(float window)
{
	std::lock_guard lock(mutex);
	pacing_window = std::clamp(window, 0.f, 1.f);
}

std::optional<uint32_t> BitrateController::set_client_enabled(bool enabled_)
{
	std::lock_guard lock(mutex);

	if (enabled_ == client_enabled)
		return {};

	client_enabled = enabled_;

	log_line("video: automatic bitrate %s on the headset", client_enabled ? "enabled" : "disabled");

	// Measurements taken under the other setting say nothing about it now. Starting over also
	// puts the bitrate back to the ceiling, which is what switching off must do.
	return reset_locked();
}

void BitrateController::set_radio_aware(bool enabled_)
{
	std::lock_guard lock(mutex);

	if (enabled_ == radio_aware)
		return;

	radio_aware = enabled_;

	log_line("video: radio-aware bitrate %s on the headset", radio_aware ? "enabled" : "disabled");

	// Never touches the bitrate: the AIMD probes back up on its own if the step was
	// unnecessary.
	flush_radio();
}

uint32_t BitrateController::current() const
{
	std::lock_guard lock(mutex);
	return bitrate;
}

uint32_t BitrateController::clamp(uint64_t value) const
{
	return uint32_t(std::clamp<uint64_t>(value, min_bitrate, ceiling));
}

void BitrateController::flush()
{
	window.clear();
	healthy_since.reset();
	last_evaluation = {};

	// bitrate_controller.cpp:226-232. Every caller of this is a bitrate that just changed.
	// Emptying the window is not enough on its own: the frames already in flight were sent at
	// the old bitrate and would refill it with exactly the samples that were just thrown away.
	// Their delivery rates are still honest measurements of the link and are kept, only their
	// utilisation is misattributed.
	if (has_frames)
		stale_before = newest_frame + 1;
}

void BitrateController::flush_radio()
{
	radio_window.clear();
	radio_ema.reset();
	last_radio_sample = {};
	last_radio_step = {};
	radio_hold = false;
	radio_stable_since.reset();
}

void BitrateController::flush_estimator()
{
	bandwidth.reset();
	bandwidth_samples = 0;
	last_bandwidth_sample = {};
	bbr_st = bbr_state::startup;
	startup_mark = 0;
	startup_stalled = 0;
	round_started = {};
	last_probe = {};
	probe_until = {};
	last_bbr_change = {};
}

int64_t BitrateController::min_loaded_wire_ns() const
{
	if (frame_period <= 0)
		return 0;

	// Relative to the paced window when the shards are paced: that window is the shortest time
	// a frame of any size can take. With pacing off there is no such floor and the line is a
	// fraction of the frame period instead.
	const double fraction = pacing_window > 0
	                                ? app_limited_wire_fraction * double(pacing_window)
	                                : unpaced_wire_fraction;

	return int64_t(fraction * double(frame_period));
}

std::optional<uint32_t> BitrateController::set_ceiling(uint32_t ceiling_bps)
{
	std::lock_guard lock(mutex);

	if (not ceiling_bps or ceiling_bps == ceiling)
		return {};

	ceiling = ceiling_bps;
	min_bitrate = std::min(conf.min_bitrate_bps, ceiling);
	return reset_locked();
}

uint32_t BitrateController::bandwidth_estimate() const
{
	std::lock_guard lock(mutex);

	if (mode_locked() != mode::bbr or not bandwidth.valid() or bandwidth_samples < estimator_min_samples)
		return 0;

	return uint32_t(std::min<double>(bandwidth.get(), std::numeric_limits<uint32_t>::max()));
}

std::optional<uint32_t> BitrateController::reset()
{
	std::lock_guard lock(mutex);
	return reset_locked();
}

std::optional<uint32_t> BitrateController::reset_locked()
{
	if (not ceiling)
		return {};

	bitrate = ceiling;
	recovery_target = bitrate;
	st = state::steady;
	first_recovery_step = true;
	frames = {};
	has_frames = false;
	stale_before = 0;
	flush();

	// A bandwidth estimate is a property of the link, but every caller of this is a change that
	// invalidates it as well. Starting the ramp over costs a second and cannot be wrong.
	flush_estimator();

	// The radio samples are a property of the radio and survive a reset; the hold does not, the
	// bitrate is back at the ceiling and probing is allowed again.
	radio_hold = false;
	radio_stable_since.reset();
	last_radio_step = {};

	return bitrate;
}

void BitrateController::close_frame(frame_state & frame, clock::time_point now)
{
	if (frame.index == uint64_t(-1))
		return;

	const bool fresh = frame.index >= stale_before;

	if (frame.lost)
	{
		if (fresh)
			window.push_back({.when = now, .lost = true, .late = frame.late});
	}
	else if (frame.valid and frame.first and frame.last >= frame.first and frame_period > 0)
	{
		const int64_t wire_ns = int64_t(frame.last - frame.first);

		double rate = 0;
		// Only a frame that actually loaded the link says anything about how much the link
		// can carry. See app_limited_wire_fraction.
		if (mode_locked() == mode::bbr and wire_ns > 0 and frame.bytes and wire_ns >= min_loaded_wire_ns())
		{
			rate = 8e9 * double(frame.bytes) / double(wire_ns);
			bandwidth.update(rate, now, estimator_window);
			++bandwidth_samples;
			last_bandwidth_sample = now;
		}

		if (fresh)
			window.push_back({
			        .when = now,
			        .utilisation = float(double(wire_ns) / double(frame_period)),
			        .late = frame.late,
			        .rate = rate,
			});
	}

	frame = {};
}

void BitrateController::on_frame_bytes(uint64_t frame_index, uint8_t stream_index, uint32_t bytes, clock::time_point now)
{
	std::lock_guard lock(mutex);

	// v1 never looks at the byte counts, and letting them touch the frame ring would move the
	// instant a frame becomes a sample for no benefit at all.
	if (mode_locked() != mode::bbr or not conf.enabled or not client_enabled or not ceiling)
		return;

	if (stream_index >= video_stream_count or not bytes)
		return;

	if (not has_frames or frame_index > newest_frame)
	{
		newest_frame = frame_index;
		has_frames = true;
	}
	// The feedback for this frame has already been turned into a sample. Can happen after a
	// stall: the send path is normally a frame or two *ahead* of the feedback.
	else if (frame_index + frame_ring_size <= newest_frame)
		return;

	auto & frame = frames[frame_index % frames.size()];
	if (frame.index != frame_index)
	{
		close_frame(frame, now);
		frame.index = frame_index;
	}

	frame.bytes += bytes;
}

BitrateController::stats BitrateController::analyse(clock::time_point now)
{
	while (not window.empty() and now - window.front().when > window_duration)
		window.pop_front();

	stats res{.count = window.size()};

	std::vector<float> utilisations;
	std::vector<double> rates;
	utilisations.reserve(window.size());
	for (const auto & s: window)
	{
		if (s.lost)
			++res.lost;
		else
			utilisations.push_back(s.utilisation);
		if (s.late)
			++res.late;
		if (s.rate > 0)
			rates.push_back(s.rate);
	}

	if (not utilisations.empty())
	{
		size_t n = std::min(utilisations.size() - 1,
		                    size_t(utilisation_percentile * utilisations.size()));
		std::nth_element(utilisations.begin(), utilisations.begin() + n, utilisations.end());
		res.utilisation = utilisations[n];
	}

	// Same high percentile, for the same reason: one slow frame is not the link slowing down.
	res.rate_count = rates.size();
	if (not rates.empty())
	{
		size_t n = std::min(rates.size() - 1, size_t(utilisation_percentile * rates.size()));
		std::nth_element(rates.begin(), rates.begin() + n, rates.end());
		res.rate = rates[n];
	}

	// A frame that never arrived is at least as bad as a fully saturated one.
	if (res.lost)
		res.utilisation = std::max(res.utilisation, double(utilisation_severe));

	return res;
}

BitrateController::radio_trend BitrateController::analyse_radio(clock::time_point now)
{
	while (not radio_window.empty() and now - radio_window.front().when > radio_trend_window)
		radio_window.pop_front();

	radio_trend res;
	if (radio_window.size() < radio_min_samples)
		return res;

	const auto & oldest = radio_window.front();
	const auto & newest = radio_window.back();

	res.span_s = std::chrono::duration<double>(newest.when - oldest.when).count();
	if (res.span_s <= 0)
		return res;

	// Least squares slope of the smoothed level against time, in dB/s.
	double mean_t = 0;
	double mean_y = 0;
	for (const auto & s: radio_window)
	{
		mean_t += std::chrono::duration<double>(s.when - oldest.when).count();
		mean_y += s.rssi_dbm;
	}
	mean_t /= double(radio_window.size());
	mean_y /= double(radio_window.size());

	double cov = 0;
	double var = 0;
	for (const auto & s: radio_window)
	{
		double dt = std::chrono::duration<double>(s.when - oldest.when).count() - mean_t;
		cov += dt * (s.rssi_dbm - mean_y);
		var += dt * dt;
	}
	if (var <= 0)
		return res;

	res.usable = true;
	res.rssi_dbm = newest.rssi_dbm;
	res.slope_db_per_s = cov / var;
	res.delta_db = res.slope_db_per_s * res.span_s;
	res.link_speed_mbps = newest.link_speed_mbps;
	for (const auto & s: radio_window)
		res.link_speed_peak_mbps = std::max(res.link_speed_peak_mbps, s.link_speed_mbps);

	return res;
}

std::optional<uint32_t> BitrateController::on_wifi_state(int rssi_dbm, int link_speed_mbps, clock::time_point when)
{
	std::lock_guard lock(mutex);

	// Both switches, plus the radio one, plus a stream to control at all.
	if (not conf.enabled or not client_enabled or not radio_aware or not ceiling)
		return {};

	// Sentinels and nonsense. Android answers -127 when it will not say, and no real Wi-Fi link
	// sits at 0 dBm or below -110 dBm.
	if (rssi_dbm >= 0 or rssi_dbm <= -110)
		return {};

	// A gap in the reports means the trend across it is meaningless: start a new one.
	if (not radio_window.empty() and when - last_radio_sample > radio_max_age)
	{
		radio_window.clear();
		radio_ema.reset();
		radio_hold = false;
		radio_stable_since.reset();
	}
	last_radio_sample = when;

	radio_ema = radio_ema
	                    ? radio_ema_alpha * rssi_dbm + (1 - radio_ema_alpha) * *radio_ema
	                    : double(rssi_dbm);
	radio_window.push_back({
	        .when = when,
	        .rssi_dbm = *radio_ema,
	        .link_speed_mbps = std::max(0, link_speed_mbps),
	});

	auto trend = analyse_radio(when);
	if (not trend.usable)
		return {};

	// The deep drop and its rebound are a closed loop of their own; a guess from the radio on
	// top of it would only make the two fight. v2 has no such regime.
	if (mode_locked() == mode::aimd and st == state::recovering)
		return {};

	// The signal is coming back: let the normal probing upwards resume at once.
	if (radio_hold and trend.delta_db > radio_rise_db and trend.rssi_dbm > radio_low_rssi_dbm)
	{
		radio_hold = false;
		log_line("video: radio signal recovering (%+.1f dB over %.1f s, now %.0f dBm), probing allowed again",
		         trend.delta_db,
		         trend.span_s,
		         trend.rssi_dbm);
	}

	// Trigger 1: a real fall, and low enough that the fall has nothing left to eat into.
	const bool falling = trend.delta_db < -radio_fall_db and trend.rssi_dbm < radio_low_rssi_dbm;
	// Trigger 2: the radio's own rate adaptation already gave up half of the PHY rate, and what
	// is left no longer has room for what is being sent.
	const bool starved = trend.link_speed_mbps > 0 and
	                     trend.link_speed_peak_mbps > 0 and
	                     double(trend.link_speed_mbps) <= radio_link_speed_collapse * double(trend.link_speed_peak_mbps) and
	                     double(trend.link_speed_mbps) * 1e6 < radio_link_speed_headroom * double(bitrate);

	// A hold taken on a fall must also let go when the fall simply *stops*: the user walked to a
	// new spot and settled there. Without this the hold would latch forever and block every
	// probe.
	if (radio_hold and not falling and not starved and
	    std::abs(trend.slope_db_per_s) < radio_stable_slope)
	{
		if (not radio_stable_since)
			radio_stable_since = when;
		else if (when - *radio_stable_since >= radio_stable_hold)
		{
			radio_hold = false;
			radio_stable_since.reset();
			log_line("video: radio signal stable at %.0f dBm (%.2f dB/s over %.1f s), probing allowed again",
			         trend.rssi_dbm,
			         trend.slope_db_per_s,
			         trend.span_s);
		}
	}
	else
		radio_stable_since.reset();

	if (not falling and not starved)
		return {};

	// Same cooldowns as any other decrease, plus one of its own: a preemptive step is a guess
	// and the frame timings must be given time to confirm or deny it.
	if (when - last_decrease < decrease_cooldown or when - last_radio_step < radio_step_interval)
		return {};

	const uint32_t previous = bitrate;

	// v2 expresses the same preemptive step as its own gain: it is the estimate that is about to
	// be wrong, and the radio-degrading gain is what the next evaluation would apply anyway.
	if (mode_locked() == mode::bbr and bandwidth.valid())
	{
		bbr_st = bbr_state::steady;
		// Never upwards, whatever the estimate says: same rule as v1.
		const uint32_t target = clamp(uint64_t(gain_radio * bandwidth.get()));
		if (target < bitrate)
		{
			bitrate = target;
			last_bbr_change = when;
		}
	}
	else
		bitrate = clamp(uint64_t(bitrate * decrease_factor));

	// Hold the probing back up until the radio says the degradation is over, whether or not the
	// bitrate could actually move (it may already be on the floor).
	radio_hold = true;
	last_decrease = when;
	last_radio_step = when;

	if (bitrate == previous)
		return {};

	// The utilisation samples were taken at the old bitrate and say nothing about the new one.
	flush();

	log_line("video: bitrate %.1f -> %.1f Mbit/s, radio degrading (%s: RSSI %.0f dBm, %+.1f dB over %.1f s, %.1f dB/s, link %d Mbit/s, peak %d Mbit/s)",
	         previous * to_mbits,
	         bitrate * to_mbits,
	         falling ? "falling signal" : "PHY rate collapse",
	         trend.rssi_dbm,
	         trend.delta_db,
	         trend.span_s,
	         trend.slope_db_per_s,
	         trend.link_speed_mbps,
	         trend.link_speed_peak_mbps);

	return bitrate;
}

std::optional<uint32_t> BitrateController::on_feedback(const wivrn::from_headset::feedback & feedback, int64_t period_ns, bool streaming, clock::time_point now)
{
	std::lock_guard lock(mutex);

	if (not conf.enabled or not client_enabled or not streaming or not ceiling or period_ns <= 0)
		return {};

	// Only the video streams (one per encoder) report frame delivery timings.
	if (feedback.stream_index >= video_stream_count)
		return {};

	frame_period = period_ns;

	if (not has_frames or feedback.frame_index > newest_frame)
	{
		newest_frame = feedback.frame_index;
		has_frames = true;
	}
	// Feedback for a frame that has already been turned into a sample (the client sends several
	// packets per frame, the last one only after it was displayed).
	else if (feedback.frame_index + frame_ring_size <= newest_frame)
		return {};

	auto & frame = frames[feedback.frame_index % frames.size()];
	if (frame.index != feedback.frame_index)
	{
		close_frame(frame, now);
		frame.index = feedback.frame_index;
	}

	if (not feedback.sent_to_decoder)
	{
		// The frame was given up on before it was complete: packets were lost or arrived far
		// too late. Its received_first_packet is the time it was abandoned, not a wire time.
		frame.lost = true;
	}
	else
	{
		if (feedback.received_first_packet)
			frame.first = frame.first ? std::min(frame.first, feedback.received_first_packet) : feedback.received_first_packet;
		if (feedback.received_last_packet)
			frame.last = std::max(frame.last, feedback.received_last_packet);
		frame.valid = true;
	}

	// Decoded, then evicted by a newer frame without ever being shown. times_displayed == 0 on
	// its own means nothing: it is also the state of the feedback sent as soon as a frame is
	// handed to the decoder.
	if (feedback.received_from_decoder and not feedback.blitted and feedback.times_displayed == 0)
		frame.late = true;

	return evaluate(now);
}

std::optional<uint32_t> BitrateController::evaluate(clock::time_point now)
{
	if (now - last_evaluation < evaluation_interval)
		return {};
	last_evaluation = now;

	// The headset stopped reporting its radio: a hold taken on data this old would last forever.
	if (radio_hold and now - last_radio_sample > radio_max_age)
	{
		radio_hold = false;
		log_line("video: no Wi-Fi report for %d ms, releasing the radio hold",
		         int(radio_max_age.count()));
	}

	auto s = analyse(now);
	if (s.count < min_samples)
		return {};

	return mode_locked() == mode::bbr ? evaluate_bbr(now, s) : evaluate_aimd(now, s);
}

std::optional<uint32_t> BitrateController::evaluate_aimd(clock::time_point now, const stats & s)
{
	const bool severe = s.utilisation > utilisation_severe or
	                    s.lost >= lost_frames_severe or
	                    s.late >= late_frames_severe;
	const bool degraded = severe or
	                      s.utilisation > utilisation_decrease or
	                      s.lost >= lost_frames_decrease or
	                      s.late >= late_frames_decrease;
	const bool healthy = s.utilisation < utilisation_increase and s.lost == 0 and s.late == 0;

	const uint32_t previous = bitrate;
	const char * reason = nullptr;

	if (degraded)
	{
		healthy_since.reset();

		if (now - last_decrease < decrease_cooldown)
			return {};
		last_decrease = now;

		if (st == state::recovering)
		{
			// Congestion returned while rebounding: the pre-drop bitrate was too
			// optimistic, aim lower next time.
			recovery_target = clamp(uint64_t(recovery_target * recovery_target_backoff));
		}

		if (severe)
		{
			if (st != state::recovering)
				recovery_target = bitrate;
			bitrate = clamp(uint64_t(bitrate * deep_decrease_factor));
			st = state::recovering;
			first_recovery_step = true;
			reason = "acute congestion";
			// The deep drop and its rebound are in charge from here.
			radio_hold = false;
		}
		else
		{
			bitrate = clamp(uint64_t(bitrate * decrease_factor));
			reason = "congestion";
		}

		if (st == state::recovering and recovery_target <= bitrate)
			st = state::steady;

		flush();
	}
	else if (healthy)
	{
		if (not healthy_since)
			healthy_since = now;

		auto held = now - *healthy_since;

		if (st == state::recovering)
		{
			if (held < (first_recovery_step ? recovery_confirm : recovery_step_interval))
				return {};

			bitrate = std::min(recovery_target, clamp(uint64_t(bitrate * recovery_factor)));
			first_recovery_step = false;
			reason = "link healthy again, rebounding";

			if (bitrate >= recovery_target)
				st = state::steady;
		}
		else
		{
			// Frame timings look fine, but the radio says the signal is still on the way
			// down: probing back up now only walks into the degradation again.
			if (radio_hold)
				return {};

			if (held < increase_hold or bitrate >= ceiling)
				return {};

			uint32_t step = std::max<uint32_t>(increase_step_min, uint32_t(ceiling * increase_step_ratio));
			bitrate = clamp(uint64_t(bitrate) + step);
			recovery_target = bitrate;
			reason = "spare capacity";
		}

		flush();
	}
	else
	{
		// Between the two thresholds: hysteresis band, hold the current bitrate.
		healthy_since.reset();
		return {};
	}

	if (bitrate == previous)
		return {};

	log_line("video: bitrate %.1f -> %.1f Mbit/s, %s (p%d utilisation %.2f, %zu lost, %zu late over %zu frames)",
	         previous * to_mbits,
	         bitrate * to_mbits,
	         reason,
	         int(utilisation_percentile * 100),
	         s.utilisation,
	         s.lost,
	         s.late,
	         s.count);

	return bitrate;
}

std::optional<uint32_t> BitrateController::evaluate_bbr(clock::time_point now, const stats & s)
{
	// The measurements have not caught up with the last change yet, see change_settle.
	if (last_bbr_change != clock::time_point{} and now - last_bbr_change < change_settle)
		return {};

	// Congestion signal: the link handing the same bytes over more slowly than the best it has
	// managed in the last ten seconds. Not applied during the startup ramp.
	const double slowdown = (bandwidth.valid() and s.rate > 0) ? bandwidth.get() / s.rate : 1;

	// A probe is a deliberate overshoot: reading that back as congestion would turn every probe
	// into a backoff. Frames actually lost or dropped still count.
	const bool overshooting = bbr_st == bbr_state::probe;
	const bool acute = s.lost >= lost_frames_decrease or
	                   s.late >= late_frames_decrease or
	                   (not overshooting and
	                    (s.utilisation > utilisation_severe or
	                     (bbr_st != bbr_state::startup and slowdown > slowdown_backoff)));

	// An estimate no loaded frame has refreshed for a whole window is not a bottleneck any more.
	if (bandwidth_samples and now - last_bandwidth_sample > estimator_window)
	{
		bandwidth.reset();
		bandwidth_samples = 0;
		log_line("video: no loaded frame for %d ms, dropping the bandwidth estimate",
		         int(estimator_window.count()));
	}

	if (bandwidth_samples < estimator_min_samples)
	{
		// Nothing measured. On a link with capacity to spare every frame is over before it
		// loaded anything and there is simply nothing to estimate. If the link measures
		// healthy and the bitrate is below the ceiling, walk it back up: v1's blind additive
		// probe is exactly the right fallback, hold and all.
		if (not acute)
		{
			const bool healthy = s.utilisation < utilisation_increase and s.lost == 0 and s.late == 0;
			if (not healthy or radio_hold)
			{
				healthy_since.reset();
				return {};
			}

			if (not healthy_since)
				healthy_since = now;

			if (now - *healthy_since < increase_hold or bitrate >= ceiling)
				return {};

			const uint32_t before = bitrate;
			const uint32_t step = std::max<uint32_t>(increase_step_min, uint32_t(ceiling * increase_step_ratio));
			bitrate = clamp(uint64_t(bitrate) + step);
			flush();

			if (bitrate == before)
				return {};

			log_line("video: bitrate %.1f -> %.1f Mbit/s, no bottleneck measured (p%d utilisation %.2f over %zu frames)",
			         before * to_mbits,
			         bitrate * to_mbits,
			         int(utilisation_percentile * 100),
			         s.utilisation,
			         s.count);
			return bitrate;
		}

		// ... unless the link just failed, which is a measurement of its own: whatever is
		// being sent is more than it can carry. Seed the filter with it.
		bandwidth.reset();
		bandwidth.update(double(bitrate), now, estimator_window);
		bandwidth_samples = estimator_min_samples;
		last_bandwidth_sample = now;
	}

	const double bw = bandwidth.get();
	const uint32_t previous = bitrate;
	const char * reason = nullptr;
	double gain = gain_steady;
	// A state change the bitrate has to follow at once, whatever the steady rate limiting would
	// otherwise say
	bool forced = false;

	if (acute)
	{
		if (now - last_decrease < decrease_cooldown)
			return {};
		last_decrease = now;

		// A maximum filter remembers for ten seconds, and an acute failure is proof that what
		// it remembers was never really deliverable.
		bandwidth.cap(s.rate > 0 ? s.rate : backoff_factor * bw);

		bbr_st = bbr_state::steady;
		startup_stalled = startup_stall_rounds;
		last_probe = now;
		last_bbr_change = now;
		probe_until = {};

		bitrate = clamp(uint64_t(backoff_factor * bandwidth.get()));
		gain = backoff_factor;
		reason = "backing off";

		// Samples taken at the old bitrate say nothing about the new one, same as v1.
		flush();
	}
	else
	{
		// The radio is the only leading indicator there is: it says the estimate is about to
		// be too optimistic. Leave the ramp and the probing alone until it recovers.
		if (radio_hold and bbr_st != bbr_state::steady)
		{
			bbr_st = bbr_state::steady;
			probe_until = {};
			last_probe = now;
		}

		switch (bbr_st)
		{
			case bbr_state::startup: {
				if (round_started == clock::time_point{})
				{
					round_started = now;
					startup_mark = bw;
				}
				else if (now - round_started >= round_duration)
				{
					startup_stalled = bw > startup_growth * startup_mark ? 0 : startup_stalled + 1;
					startup_mark = bw;
					round_started = now;
				}

				if (startup_stalled >= startup_stall_rounds)
				{
					bbr_st = bbr_state::steady;
					last_probe = now;
					log_line("video: bandwidth startup done, plateaued at %.1f Mbit/s after %zu flat rounds",
					         bw * to_mbits,
					         startup_stalled);
				}
				break;
			}

			case bbr_state::probe:
				if (now >= probe_until)
				{
					bbr_st = bbr_state::steady;
					last_probe = now;
					// The samples in the window were taken at the raised gain; the
					// drain back to the steady one must not wait for them to age out.
					flush();
					forced = true;
				}
				break;

			case bbr_state::steady:
				// One probe every probe_interval, to rediscover capacity that came back.
				// Never into a falling radio.
				if (not radio_hold and now - last_probe >= probe_interval and bitrate < ceiling)
				{
					bbr_st = bbr_state::probe;
					probe_until = now + probe_duration;
					log_line("video: probing at gain %.2f, estimate %.1f Mbit/s",
					         gain_probe,
					         bw * to_mbits);
				}
				break;
		}

		switch (bbr_st)
		{
			case bbr_state::startup:
				gain = gain_startup;
				reason = "startup";
				break;
			case bbr_state::probe:
				gain = gain_probe;
				reason = "probing";
				break;
			case bbr_state::steady:
				gain = radio_hold ? gain_radio : gain_steady;
				reason = radio_hold ? "steady, radio degrading" : "steady";
				break;
		}

		const uint32_t target = clamp(uint64_t(gain * bw));

		// Do not chase the few percent the estimate wobbles by, and do not re-encode at a new
		// bitrate more often than once a second, once out of the startup ramp.
		if (bbr_st != bbr_state::startup and not forced)
		{
			if (now - last_bbr_change < steady_interval)
				return {};

			const double delta = std::abs(double(target) - double(bitrate));
			if (bitrate and delta < steady_change_threshold * double(bitrate))
				return {};
		}

		bitrate = target;
		last_bbr_change = now;

		// The bandwidth filter is deliberately not flushed: it is the one thing that has to
		// survive a change of bitrate.
		if (bitrate != previous)
			flush();
	}

	if (bitrate == previous)
		return {};

	log_line("video: bitrate %.1f -> %.1f Mbit/s, %s (estimate %.1f Mbit/s, gain %.2f, recent %.1f Mbit/s over %zu samples, slowdown x%.2f, p%d utilisation %.2f, %zu lost, %zu late over %zu frames)",
	         previous * to_mbits,
	         bitrate * to_mbits,
	         reason,
	         bandwidth.get() * to_mbits,
	         gain,
	         s.rate * to_mbits,
	         s.rate_count,
	         slowdown,
	         int(utilisation_percentile * 100),
	         s.utilisation,
	         s.lost,
	         s.late,
	         s.count);

	return bitrate;
}

} // namespace wivrnnx::helper
