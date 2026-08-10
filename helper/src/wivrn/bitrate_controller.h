// Automatic bitrate control.
//
// Copy-adapted from wivrn::bitrate_controller
// (server/driver/bitrate_controller.{h,cpp}), the NX AIMD + delivered-bandwidth
// controller. Every threshold, gain, cooldown and window below is upstream's,
// value for value, and the two evaluators are transcriptions of
// evaluate_aimd()/evaluate_bbr(); the commentary that explains why each number is
// what it is is upstream's too and is kept, because the numbers are worth nothing
// without it.
//
// What is dropped: the multipath half (set_path_ceiling / set_combined /
// effective_ceiling's path clamp), which exists to describe a session that failed
// over onto a USB tunnel or that stripes video over two links at once. This port
// has one path. `to_headset::transport_status` reporting is dropped with it —
// nothing here draws the headset's Transport page — and the snapshot it fed is
// reduced to what the log line wants.
//
// What is added: nothing. In particular the controller still takes no clock
// offset (see "The queueing signal, and why it is not an RTT" below): every time
// it uses is either its own steady_clock or a difference of two headset
// timestamps, which is what makes it usable in the first seconds of a session.
//
// ======================================================================================
//
// The client acknowledges every video frame with a from_headset::feedback packet that carries,
// among others, the time the first and the last packet of that frame were received (both in the
// client clock, so their difference needs no clock offset). The fraction of a frame period spent
// receiving a frame is a direct measure of how much of the link capacity the stream is using:
//
//     utilisation = (received_last_packet - received_first_packet) / frame_period
//
// A utilisation close to (or above) 1 means the wireless link cannot deliver a frame within the
// time budget of a frame, which is exactly what "the connection lags" feels like. Frames that
// never arrive completely (no sent_to_decoder, same test the IDR tracker uses) and frames that are
// decoded but dropped before ever being shown are counted as delivery failures.
//
// Two regimes coexist:
//   * gradual degradation (walking away from the router): gentle multiplicative decrease followed
//     by slow additive probing back up;
//   * acute lag spike: a deep drop, which in practice also flushes whatever queue got wedged, then
//     a fast slow-start style rebound back to the pre-drop bitrate. If congestion returns during
//     the rebound the rebound target itself is lowered (the classic ssthresh idea) so the two do
//     not oscillate.
//
// All decisions are taken from a sliding window of per-frame samples, using a high percentile so
// that a single unlucky frame does not move the bitrate. The window is flushed after every change
// so that stale samples cannot trigger a second change.
//
// On top of that, the headset reports its Wi-Fi radio state about once a second
// (from_headset::wifi_state). Frame timings are a *lagging* indicator: by the time the utilisation
// rises the packets are already late. The radio is a *leading* one — the signal starts falling a
// second or two before the rate adaptation gives up and the first packet is lost. The controller
// therefore steps down preemptively on a falling signal, while the frame timings still look fine.
// Only the trend is used: absolute dBm says nothing portable, a fall of several dB over a few
// seconds says the user is walking away from the access point. Radio input can only ever *lower*
// the bitrate; it never raises one, and it never runs while the deep-drop recovery is in charge.
//
// ======================================================================================
// Control law v2: delivered-bandwidth estimation (bitrate_mode::bbr)
// ======================================================================================
//
// Everything above is a *congestion* controller: it reacts to the link being full. It never
// learns how big the link is, so after a decrease it has to walk back up blind, one additive
// step every five seconds. v2 measures the link instead, the way BBR does, and derives the
// bitrate from that estimate. It is selected per session and runs inside this same object: the
// ceiling, the floor, the switches, the frame ring that joins the per-stream feedback into one
// frame, the acute lost/late detection and the radio trend are all shared and all still apply —
// only the function that turns a window of samples into a bitrate differs.
//
// --- The delivery rate sample ---------------------------------------------------------
// The headset says, per frame, when the first and the last packet of that frame arrived. The
// server knows how many bytes it put on the wire for that frame (see on_frame_bytes: the
// send path reports them, parity shards included, which is exactly the unit the bitrate is
// expressed in). One frame therefore yields
//
//     delivery_rate = 8 * frame_bytes / (received_last - received_first)
//
// which is a *lower bound* on the capacity of the bottleneck: those bytes really did get
// through in that time. Both timestamps are in the client clock, so no clock offset is needed.
//
// --- App-limited samples --------------------------------------------------------------
// A lower bound is only useful if the sender was actually trying. BBR calls a sample
// "app-limited" when the application had nothing more to send, and refuses to let such a
// sample lower its estimate. Here the analogous case is a frame that is small compared to what
// the link could have carried in that window: a nearly static scene produces a 5 kB P-frame
// that goes out in one micro-burst and lands in a few hundred microseconds, and 5 kB over
// 300 us reads as 130 Mbit/s of "capacity" that is really one Wi-Fi TXOP measured with a
// stopwatch. So a sample is only admitted when the frame occupied the link for a meaningful
// stretch of the frame period: see app_limited_wire_fraction.
//
// Packet pacing (see shard_pacer.h) is the other half of the same story, and the reason the
// threshold is expressed relative to the *paced* window rather than to a frame period. Pacing
// deliberately spreads a frame over ~40% of a frame period whatever its size, so with pacing on
// the measured delivery rate saturates at about bitrate / window: the sender, not the link, is
// what limits it. That is a genuine app-limited regime and it is handled the way BBR handles
// its own: the sample is still a valid lower bound, the estimate climbs to it, and the
// controller keeps raising the bitrate until something else stops it — a ceiling, or the link
// actually filling up, at which point the receive span stretches past the paced window and the
// samples become real capacity measurements again.
//
// --- The queueing signal, and why it is not an RTT --------------------------------------
// BBR's second state variable is min_rtt, and the ratio of the current RTT to it is how it
// notices a queue building before anything is lost. A real RTT is not available here without
// a clock offset: send_begin is in the server clock and received_first_packet in the client's,
// and while this port does run a clock offset estimator (wivrn/clock_offset.h), it needs
// seconds to converge and its residual error is of the same order as the queueing delay that
// would have to be detected. Depending on it would make the controller useless exactly when
// adaptation matters most — the first seconds of a session. So no absolute delay is used.
//
// The obvious offset-free substitute is the wire span itself, received_last - received_first,
// compared against its own windowed minimum. That is wrong: a wire span is a *transmission*
// time, not a propagation delay. It is proportional to the number of bytes in the frame, so it
// grows whenever the bitrate is raised or the scene gets busy, with no queue anywhere. A min
// filter on it would read every increase as congestion.
//
// Dividing by the bytes removes exactly that dependency, and what is left is the delivery rate
// the estimator already computes. So the queueing signal is
//
//     slowdown = windowed_max_bandwidth / (recent delivery rate)
//
// — the link handing over the same bytes more slowly than the best it has managed in the last
// ten seconds. It is scale-free, it needs no clock offset, and it is the same quantity in both
// pacing regimes. Both sides are high percentiles, so a single slow frame does not trigger it,
// and it is ignored during the startup ramp.
//
// --- The state machine -------------------------------------------------------------------
// startup: gain 1.25, doubling-ish growth until the bandwidth estimate stops improving for
//          three consecutive rounds — BBR's STARTUP and its bandwidth-plateau exit.
// steady:  gain 0.85. The gap to 1 is the headroom that keeps the bottleneck queue empty; it
//          is also what leaves room for the P-frame/I-frame variance a video encoder produces
//          around its nominal bitrate.
// probe:   once every eight seconds, one round at gain 1.10, so that capacity that came back
//          is rediscovered instead of waiting for the ten second maximum filter to age out.
//          Skipped while the radio says the link is degrading. A probe is a deliberate
//          overshoot, so while one is running its own overshoot is not read as congestion.
// backoff: on the same acute signals v1 treats as congestion — frames that never arrived,
//          frames decoded but never shown, utilisation at the ceiling — or on the slowdown
//          above, the maximum filter is first capped to what the link is measurably doing
//          right now, and the bitrate goes to 0.7 times that. The state returns to steady,
//          whose 0.85 gain brings the bitrate back up one second later: the undershoot is
//          BBR's DRAIN, there to empty the queue that built up.
// A degrading radio trend forces steady with gain 0.75 and blocks probing.
#pragma once

#include "wivrn_packets.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

namespace wivrnnx::helper
{

class BitrateController
{
public:
	using clock = std::chrono::steady_clock;
	using mode = wivrn::bitrate_mode;

	struct config
	{
		// NX default is on; --no-adaptive turns it off.
		bool enabled = true;
		// Never go below this, whatever the measurements say
		uint32_t min_bitrate_bps = 10'000'000;
		// Control law used when the headset expresses no preference of its own. The
		// headset's selector always wins over this.
		mode control = mode::aimd;
	};

	// --- Measurement window -------------------------------------------------------------
	// Duration of the sliding window of per-frame samples.
	static constexpr std::chrono::milliseconds window_duration{2000};
	// No decision is taken with fewer samples than this in the window.
	static constexpr size_t min_samples = 20;
	// Upper bound on how often the policy is evaluated.
	static constexpr std::chrono::milliseconds evaluation_interval{250};
	// Percentile of the utilisation samples used for every threshold comparison.
	static constexpr double utilisation_percentile = 0.9;
	// Number of frames tracked simultaneously; a frame is turned into a sample once this many
	// newer frames have been seen, which is enough for the late "displayed" feedback to arrive.
	static constexpr size_t frame_ring_size = 16;
	// One video stream per encoder. Upstream allows three (left, right, alpha); this port
	// produces two and the constant is left at upstream's value, since a stream index it
	// never emits simply never appears.
	static constexpr uint8_t video_stream_count = 3;

	// --- Thresholds ---------------------------------------------------------------------
	// p90 utilisation above this: the link is saturated, decrease.
	static constexpr double utilisation_decrease = 0.85;
	// p90 utilisation below this: there is spare capacity, probe upwards. The gap with
	// utilisation_decrease is the hysteresis band, and is mandatory: inside it nothing happens.
	static constexpr double utilisation_increase = 0.60;
	// p90 utilisation above this: acute congestion, a frame no longer fits in a frame period.
	static constexpr double utilisation_severe = 1.00;
	// Frames that never arrived completely, counted over the window.
	static constexpr size_t lost_frames_decrease = 1;
	static constexpr size_t lost_frames_severe = 3;
	// Frames decoded but dropped before being displayed, counted over the window.
	static constexpr size_t late_frames_decrease = 4;
	static constexpr size_t late_frames_severe = 10;

	// --- Decrease -----------------------------------------------------------------------
	// Gentle multiplicative decrease, for gradual degradation.
	static constexpr double decrease_factor = 0.7;
	// Deep drop on an acute lag spike. The drop itself is therapeutic: it lets whatever queue
	// piled up on the link drain, after which the same bitrate is usually fine again.
	static constexpr double deep_decrease_factor = 0.4;
	// Minimum time between two decreases, so a single bad patch cannot collapse the bitrate.
	static constexpr std::chrono::milliseconds decrease_cooldown{2000};

	// --- Slow additive increase (above the recovery target) -----------------------------
	// Increase step, whichever is larger.
	static constexpr uint32_t increase_step_min = 2'000'000;
	static constexpr double increase_step_ratio = 0.05; // of the ceiling
	// The link must measure healthy for this long before every increase. As the window is
	// flushed on every change this also acts as the increase cooldown.
	static constexpr std::chrono::milliseconds increase_hold{5000};

	// --- Fast recovery (after a deep drop, up to the pre-drop bitrate) ------------------
	static constexpr std::chrono::milliseconds recovery_confirm{2500};
	static constexpr std::chrono::milliseconds recovery_step_interval{1000};
	// Rebound steps are multiplicative, so the pre-drop level is reached in a few seconds.
	static constexpr double recovery_factor = 2.0;
	// If congestion comes back while rebounding, lower the rebound target by this factor.
	static constexpr double recovery_target_backoff = 0.75;

	// --- Radio trend (preemptive decrease) ----------------------------------------------
	static constexpr std::chrono::milliseconds radio_trend_window{4000};
	// A sample older than this is stale: the headset stopped reporting. Stale data is ignored
	// entirely and releases any hold.
	static constexpr std::chrono::milliseconds radio_max_age{5000};
	static constexpr size_t radio_min_samples = 4;
	// Smoothing of the reported RSSI. One sample per second, so a time constant of ~2 s.
	static constexpr double radio_ema_alpha = 0.4;
	// The fall, in dB over the window, that separates walking away from the access point from
	// the noise of standing still.
	static constexpr double radio_fall_db = 6.0;
	// ... but a fall only matters once the absolute level is low enough that the radio's rate
	// adaptation is about to start dropping MCS.
	static constexpr double radio_low_rssi_dbm = -65.0;
	// Second, independent trigger: the radio's own rate adaptation already halved the PHY rate
	// compared to the best it saw in the window, *and* what is left is less than this multiple
	// of the bitrate being sent.
	static constexpr double radio_link_speed_headroom = 2.0;
	static constexpr double radio_link_speed_collapse = 0.5;
	// Minimum time between two preemptive steps: a preemptive step is a guess, and the frame
	// timings must be given time to confirm or deny it.
	static constexpr std::chrono::milliseconds radio_step_interval{4000};
	// The signal coming back up by this much over the window releases the hold.
	static constexpr double radio_rise_db = 3.0;
	// A hold is also released when the signal simply stops falling.
	static constexpr double radio_stable_slope = 0.75;
	static constexpr std::chrono::milliseconds radio_stable_hold{3000};

	// --- v2: delivered-bandwidth estimator ----------------------------------------------
	static constexpr std::chrono::milliseconds estimator_window{10000};
	// A delivery rate sample only counts towards the maximum filter when the frame kept the
	// link busy for at least this fraction of the *paced* window.
	static constexpr double app_limited_wire_fraction = 0.30;
	// The same idea with pacing switched off, where there is no window to be a fraction of.
	static constexpr double unpaced_wire_fraction = 0.10;
	static constexpr size_t estimator_min_samples = 12;

	// Gains applied to the bandwidth estimate, one per state.
	static constexpr double gain_startup = 1.25;
	static constexpr double gain_steady = 0.85;
	static constexpr double gain_probe = 1.10;
	// Steady gain while the radio trend says the link is on its way down.
	static constexpr double gain_radio = 0.75;
	// Acute congestion: the maximum filter is first capped to what the link is measured to be
	// delivering right now, and the bitrate goes to this times that.
	static constexpr double backoff_factor = 0.70;

	// One "round" of the startup ramp. Not a round trip — there is no RTT here.
	static constexpr std::chrono::milliseconds round_duration{500};
	static constexpr double startup_growth = 1.25;
	static constexpr size_t startup_stall_rounds = 3;

	static constexpr std::chrono::milliseconds probe_interval{8000};
	static constexpr std::chrono::milliseconds probe_duration{500};

	// In steady state the bitrate is not allowed to move more often than this, nor at all
	// unless the new target differs from the current bitrate by more than the threshold.
	static constexpr std::chrono::milliseconds steady_interval{1000};
	static constexpr double steady_change_threshold = 0.05;
	// Nothing is evaluated for this long after a change of bitrate.
	static constexpr std::chrono::milliseconds change_settle{250};

	// How much slower than the ten second maximum the link has to be handing frames over
	// before that counts as congestion rather than as noise.
	static constexpr double slowdown_backoff = 1.60;

	BitrateController() = default;

	// Set the configuration and the initial ceiling. The ceiling is the bitrate the operator
	// asked for on the command line; the controller never goes above it. client_enabled is the
	// headset side switch: the control only runs when both it and the server configuration are
	// enabled. radio_aware is the headset side switch for the preemptive radio trend.
	// client_mode is the control law the headset asked for, empty when it asked for none and
	// config::control decides.
	void configure(const config &, uint32_t ceiling_bps, bool client_enabled, bool radio_aware, std::optional<mode> client_mode = {});

	bool enabled() const;
	uint32_t current() const;

	// The headset toggled its own switch. Starts over, as measurements taken under the other
	// setting say nothing; switching off therefore restores the full ceiling.
	std::optional<uint32_t> set_client_enabled(bool);

	// The headset toggled the radio trend switch. Never changes the bitrate on its own.
	void set_radio_aware(bool);

	// The control law the headset asked for, empty when it expresses no preference. A change
	// starts over, exactly like the headset switch.
	std::optional<uint32_t> set_client_mode(std::optional<mode>);

	// Control law actually in force.
	mode active_mode() const;

	enum class state_name
	{
		off,
		steady,
		recovering,
		startup,
		probe,
	};

	// Everything the log line says about the control, read in one go under the controller's
	// own mutex so that the fields agree with each other.
	struct status
	{
		uint32_t bitrate_bps = 0;
		uint32_t ceiling_bps = 0;
		mode control = mode::aimd;
		state_name state = state_name::off;
		bool radio_hold = false;
		// Delivered-bandwidth estimate, v2 only, 0 when there is none yet.
		uint32_t estimate_bps = 0;
	};
	status snapshot() const;

	// The pacing window currently in force, as a fraction of a frame period, or 0 when the
	// video shards are not paced. Only the v2 estimator uses it.
	void set_pacing_window(float window);

	// Bytes put on the wire for one video frame of one stream, parity shards included: the
	// unit the bitrate itself is expressed in. Allocation-free, and a no-op unless the v2
	// estimator is the one running.
	void on_frame_bytes(uint64_t frame_index, uint8_t stream_index, uint32_t bytes, clock::time_point now = clock::now());

	// A new ceiling was requested. Resets the controller to it.
	std::optional<uint32_t> set_ceiling(uint32_t ceiling_bps);

	// The delivered-bandwidth estimate in bits per second, or 0 when the v2 estimator is not
	// the law in force or has not admitted enough samples yet.
	uint32_t bandwidth_estimate() const;

	// Forget all measurements and go back to the ceiling, e.g. when a new client connects.
	std::optional<uint32_t> reset();

	// Feed one feedback packet. frame_period_ns is the current video frame period, streaming
	// tells whether the stream is up at all. Returns a new bitrate to apply, if any. now is
	// injectable so the policy can be driven on a virtual clock by the tests.
	std::optional<uint32_t> on_feedback(const wivrn::from_headset::feedback &, int64_t frame_period_ns, bool streaming, clock::time_point now = clock::now());

	// Feed one Wi-Fi radio report from the headset. Only call it for a sample the headset
	// marked valid; obviously impossible values are rejected here as well. link_speed_mbps <= 0
	// means unknown, the rest of the sample is still used. Returns a lower bitrate to apply, if
	// the trend calls for a preemptive step; never a higher one.
	std::optional<uint32_t> on_wifi_state(int rssi_dbm, int link_speed_mbps, clock::time_point when = clock::now());

private:
	// State of one video frame while its per-stream feedback packets are being collected.
	struct frame_state
	{
		uint64_t index = uint64_t(-1);
		XrTime first = 0; // earliest received_first_packet over all streams
		XrTime last = 0;  // latest received_last_packet over all streams
		bool valid = false;
		bool lost = false; // at least one stream never arrived completely
		bool late = false; // decoded but dropped before being displayed
		// Bytes put on the wire for this frame, summed over the video streams, v2 only.
		uint64_t bytes = 0;
	};

	struct sample
	{
		clock::time_point when;
		float utilisation = 0;
		bool lost = false;
		bool late = false;
		// Delivery rate this frame measured, bits per second, or 0 when it says nothing
		// about the capacity (v1, a lost frame, an app-limited one)
		double rate = 0;
	};

	// Windowed maximum of a scalar, Kathleen Nichols' three-sample filter, the same one BBR
	// uses for its bandwidth estimate. O(1) per sample, no allocation, three entries whatever
	// the frame rate. bitrate_controller.h:509-587, unchanged.
	class max_filter
	{
	public:
		bool valid() const
		{
			return count != 0;
		}
		// Meaningless when not valid()
		double get() const
		{
			return s[0].value;
		}

		void reset()
		{
			count = 0;
			s = {};
		}

		void update(double value, clock::time_point now, clock::duration window)
		{
			++count;

			// First sample, a new extremum, or everything on record is stale.
			if (count == 1 or better(value, s[0].value) or now - s[2].when > window)
			{
				s[0] = s[1] = s[2] = {now, value};
				return;
			}

			if (better(value, s[1].value))
				s[1] = s[2] = {now, value};
			else if (better(value, s[2].value))
				s[2] = {now, value};

			if (now - s[0].when > window)
			{
				// The running extremum aged out: promote the two sub-window ones.
				s[0] = s[1];
				s[1] = s[2];
				s[2] = {now, value};
				if (now - s[0].when > window)
				{
					s[0] = s[1];
					s[1] = s[2];
				}
			}
			else if (s[1].when == s[0].when and now - s[1].when > window / 4)
				s[1] = s[2] = {now, value};
			else if (s[2].when == s[1].when and now - s[2].when > window / 2)
				s[2] = {now, value};
		}

		// Force the maximum down to at most `value`: what the filter remembers has just been
		// proven wrong by an event rather than aged out.
		void cap(double value)
		{
			for (auto & e: s)
			{
				if (better(e.value, value))
					e.value = value;
			}
		}

	private:
		struct entry
		{
			clock::time_point when{};
			double value = 0;
		};

		static bool better(double a, double b)
		{
			return a >= b;
		}

		std::array<entry, 3> s{};
		size_t count = 0;
	};

	// One Wi-Fi report, after the sentinel filtering.
	struct radio_sample
	{
		clock::time_point when;
		double rssi_dbm = 0; // smoothed, not the raw report
		int link_speed_mbps = 0;
	};

	// What the window says about the radio right now.
	struct radio_trend
	{
		bool usable = false;
		double rssi_dbm = 0; // smoothed current level
		double slope_db_per_s = 0;
		double span_s = 0;
		// Change over the window, positive while the signal improves
		double delta_db = 0;
		int link_speed_mbps = 0;
		int link_speed_peak_mbps = 0;
	};

	enum class state
	{
		// Slow AIMD around the current bitrate.
		steady,
		// Below a remembered pre-drop bitrate after a deep drop, rebounding fast.
		recovering,
	};

	// v2 only, see the state machine described at the top of this file.
	enum class bbr_state
	{
		startup,
		steady,
		probe,
	};

	struct stats
	{
		double utilisation = 0; // high percentile over the window
		size_t lost = 0;
		size_t late = 0;
		size_t count = 0;
		// High percentile of the delivery rates in the window, and how many there were.
		double rate = 0;
		size_t rate_count = 0;
	};

	mutable std::mutex mutex;

	config conf;
	// Headset side switch, ANDed with conf.enabled
	bool client_enabled = true;
	// Control law the headset asked for; conf.control when it asked for none
	std::optional<mode> client_mode;
	// Headset side switch for the radio trend, ANDed with the two above
	bool radio_aware = true;
	// Ceiling requested by the operator
	uint32_t ceiling = 0;
	uint32_t bitrate = 0;
	uint32_t min_bitrate = 0;

	state st = state::steady;
	uint32_t recovery_target = 0;
	bool first_recovery_step = true;

	std::array<frame_state, frame_ring_size> frames;
	uint64_t newest_frame = 0;
	bool has_frames = false;
	// Frames older than this were sent at a bitrate that is no longer in force, and say
	// nothing about the one that is. Moved past everything in flight by every flush()
	uint64_t stale_before = 0;
	// Current video frame period in ns
	int64_t frame_period = 0;
	// Fraction of a frame period the video shards are spread over, 0 when not paced
	float pacing_window = 0;

	// --- v2 estimator state -------------------------------------------------------------
	bbr_state bbr_st = bbr_state::startup;
	// Bottleneck bandwidth estimate, bits per second, maximum over estimator_window
	max_filter bandwidth;
	// Delivery rate samples admitted into it, and when the last one was
	size_t bandwidth_samples = 0;
	clock::time_point last_bandwidth_sample{};
	// Value the bandwidth estimate had at the start of the current startup round, and how
	// many consecutive rounds it has failed to grow
	double startup_mark = 0;
	size_t startup_stalled = 0;
	clock::time_point round_started{};
	clock::time_point last_probe{};
	clock::time_point probe_until{};
	clock::time_point last_bbr_change{};

	std::deque<sample> window;
	std::optional<clock::time_point> healthy_since;
	clock::time_point last_evaluation{};
	clock::time_point last_decrease{};

	std::deque<radio_sample> radio_window;
	// Smoothed RSSI carried across samples, empty until the first one
	std::optional<double> radio_ema;
	clock::time_point last_radio_sample{};
	clock::time_point last_radio_step{};
	// A preemptive step was taken and the radio has not recovered since: hold the normal
	// probing upwards, it would only walk straight back into the degradation.
	bool radio_hold = false;
	// When the slope first read flat while a hold was in force, for the stabilisation release.
	std::optional<clock::time_point> radio_stable_since;

	// active_mode(), with the mutex already held
	mode mode_locked() const;
	// reset(), with the mutex already held
	std::optional<uint32_t> reset_locked();
	void close_frame(frame_state &, clock::time_point now);
	void flush();
	// Forget the radio trend and release the hold
	void flush_radio();
	// Forget everything the v2 estimator learned
	void flush_estimator();
	// Shortest wire span, in ns, that is not an app-limited measurement
	int64_t min_loaded_wire_ns() const;
	stats analyse(clock::time_point now);
	radio_trend analyse_radio(clock::time_point now);
	std::optional<uint32_t> evaluate(clock::time_point now);
	std::optional<uint32_t> evaluate_aimd(clock::time_point now, const stats &);
	std::optional<uint32_t> evaluate_bbr(clock::time_point now, const stats &);
	uint32_t clamp(uint64_t value) const;
};

} // namespace wivrnnx::helper
