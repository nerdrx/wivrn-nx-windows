// The automatic bitrate controller and the IDR damping, driven on a virtual
// clock with synthetic feedback traces.
//
// Everything here is a property of the *ported* controller measured against the
// thresholds the Linux one is written in terms of — the whole point of the port
// is that those numbers keep meaning what they meant, so every check below names
// the constant it is testing rather than a magic number, and the trace shapes are
// the ones the header's commentary describes:
//
//   Part A: nothing at all happens below min_samples, above the ceiling, or with
//           the control switched off (--no-adaptive, and the headset's own switch).
//   Part B: AIMD. A clean trace probes back up towards the ceiling one
//           increase_step every increase_hold; a congested one comes down by
//           decrease_factor no faster than decrease_cooldown; an acute one takes
//           the deep drop and then rebounds by recovery_factor.
//   Part C: the hysteresis band between utilisation_increase and
//           utilisation_decrease really does nothing.
//   Part D: the radio trend steps down before the frame timings notice, holds the
//           probing back, and lets go when the signal settles.
//   Part E: the v2 delivered-bandwidth law: the estimate is built from the frame
//           bytes and the receive span, the bitrate follows a gain times it, and
//           an acute failure caps the filter rather than waiting for it to age out.
//   Part F: IDR damping. The live failure this phase exists to cure — a headset
//           that reports a lost frame on every frame — must not produce more than
//           two key frame requests a second, and must still produce them.
//
// Native Linux build; see run_tests.sh next to this file.

#include "../wivrn/bitrate_controller.h"
#include "../wivrn/idr_tracker.h"

#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <optional>
#include <vector>

using namespace wivrnnx::helper;
using clock_t_ = BitrateController::clock;
using namespace std::chrono_literals;

namespace
{

int checks = 0;
int failures = 0;

#define CHECK(...)                                                                          \
	do                                                                                  \
	{                                                                                   \
		++checks;                                                                   \
		if (not(__VA_ARGS__))                                                       \
		{                                                                           \
			++failures;                                                         \
			std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__); \
		}                                                                           \
	} while (0)

constexpr uint32_t Mbit = 1'000'000;
// 90 fps, the rate the RX 580 session runs at.
constexpr int64_t period_ns = 11'111'111;

// One synthetic client, feeding one frame's feedback per stream at a fixed
// utilisation. The headset timestamps are in the client clock and the controller
// only ever takes their difference, so any origin will do.
class Trace
{
public:
	explicit Trace(BitrateController & c) :
	        ctl(c) {}

	// Frames per second the trace plays at.
	void advance(std::chrono::nanoseconds dt)
	{
		now += dt;
		headset_ns += dt.count();
	}

	// One frame of both eyes. `utilisation` is the fraction of a frame period the
	// headset spent receiving it; lost means it never completed; late means it was
	// decoded and then dropped without ever being shown.
	void frame(double utilisation, bool lost = false, bool late = false, uint32_t bytes_per_eye = 0)
	{
		for (uint8_t stream = 0; stream < 2; ++stream)
		{
			if (bytes_per_eye)
				ctl.on_frame_bytes(index, stream, bytes_per_eye, now);

			wivrn::from_headset::feedback f{};
			f.frame_index = index;
			f.stream_index = stream;
			f.received_first_packet = headset_ns;
			f.received_last_packet = headset_ns + int64_t(utilisation * double(period_ns));

			if (not lost)
			{
				f.sent_to_decoder = f.received_last_packet + 1;
				f.received_from_decoder = f.sent_to_decoder + 1;
				if (late)
				{
					// Decoded, then evicted before it was ever shown: blitted
					// and times_displayed both stay zero.
					f.times_displayed = 0;
				}
				else
				{
					f.blitted = f.received_from_decoder + 1;
					f.displayed = f.blitted + 1;
					f.times_displayed = 1;
				}
			}

			if (auto b = ctl.on_feedback(f, period_ns, true, now))
			{
				applied = *b;
				history.push_back(*b);
				++changes;
			}
		}

		++index;
		advance(std::chrono::nanoseconds(period_ns));
	}

	// `seconds` worth of frames at 90 fps.
	void play(double seconds, double utilisation, bool lost = false, bool late = false, uint32_t bytes_per_eye = 0)
	{
		const int frames = int(seconds * 1e9 / double(period_ns));
		for (int i = 0; i < frames; ++i)
			frame(utilisation, lost, late, bytes_per_eye);
	}

	BitrateController & ctl;
	clock_t_::time_point now = clock_t_::time_point{} + 1h;
	int64_t headset_ns = 500'000'000;
	uint64_t index = 1;
	uint32_t applied = 0;
	int changes = 0;
	// Every bitrate the controller handed back, in order.
	std::vector<uint32_t> history;
};

// A controller and the trace that drives it. One object because the controller
// holds a mutex and cannot be moved.
struct Fixture
{
	BitrateController c;
	Trace t;

	explicit Fixture(uint32_t ceiling,
	                 bool enabled = true,
	                 BitrateController::mode m = BitrateController::mode::aimd) :
	        t(c)
	{
		BitrateController::config cfg{};
		cfg.enabled = enabled;
		cfg.control = m;
		c.configure(cfg, ceiling, true, true, {});
	}
};

void part_a_gates()
{
	std::printf("Part A: the gates\n");

	// --no-adaptive: the controller is inert and the bitrate stays where the
	// command line put it.
	{
		Fixture fx(50 * Mbit, false);
		auto & c = fx.c;
		CHECK(not c.enabled());
		Trace t(c);
		t.play(6, 1.4, true, true);
		CHECK(t.changes == 0);
		CHECK(c.current() == 50 * Mbit);
	}

	// The headset's own switch does the same, and turning it off restores the
	// ceiling whatever the controller had walked down to.
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(6, 1.4, true, true);
		CHECK(c.current() < 50 * Mbit);
		auto restored = c.set_client_enabled(false);
		CHECK(restored.has_value() && *restored == 50 * Mbit);
		const int before = t.changes;
		t.play(6, 1.4, true, true);
		CHECK(t.changes == before);
	}

	// Fewer than min_samples in the window: no decision at all. min_samples is 20
	// and one frame of two eyes is one sample, so half a second of frames is not
	// yet enough to react to a catastrophic trace.
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		for (int i = 0; i < 15; ++i)
			t.frame(2.0, true);
		CHECK(t.changes == 0);
		CHECK(c.current() == 50 * Mbit);
	}

	// The floor and the ceiling. A ceiling below the configured minimum wins.
	{
		Fixture fx(6 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(60, 2.0, true, true);
		CHECK(c.current() == 6 * Mbit);
	}
	{
		Fixture fx(100 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(90, 2.0, true, true);
		// BitrateController::config::min_bitrate_bps
		CHECK(c.current() == 10 * Mbit);
		CHECK(t.applied == 10 * Mbit);
	}

	// Feedback for a stream that is not a video stream is ignored.
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		wivrn::from_headset::feedback f{};
		f.stream_index = BitrateController::video_stream_count;
		f.frame_index = 1;
		for (int i = 0; i < 200; ++i)
			CHECK(not c.on_feedback(f, period_ns, true, clock_t_::now()).has_value());
	}
}

void part_b_aimd()
{
	std::printf("Part B: AIMD decrease, deep drop and rebound\n");

	// --- a link that has started to saturate: one gentle multiplicative step ---
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		// p90 utilisation above utilisation_decrease (0.85) but not severe, and
		// no losses: "congestion", decrease_factor.
		t.play(1.0, 0.9);
		CHECK(t.changes == 1);
		CHECK(c.current() == uint32_t(50 * Mbit * BitrateController::decrease_factor));

		// decrease_cooldown is 2 s: another second of the same trace changes
		// nothing.
		const int after_first = t.changes;
		t.play(0.9, 0.9);
		CHECK(t.changes == after_first);

		// Past the cooldown it steps again.
		t.play(1.5, 0.9);
		CHECK(t.changes == after_first + 1);
		CHECK(c.current() < uint32_t(50 * Mbit * BitrateController::decrease_factor));
	}

	// --- the acute case: the deep drop, then the fast rebound ------------------
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;

		// Frames that never complete: lost_frames_severe is 3, and a lost frame
		// also forces the utilisation to utilisation_severe.
		t.play(1.0, 1.3, true);
		CHECK(t.changes == 1);
		const uint32_t dropped = c.current();
		CHECK(dropped == uint32_t(50 * Mbit * BitrateController::deep_decrease_factor));
		CHECK(c.snapshot().state == BitrateController::state_name::recovering);

		// The trace keeps losing frames for another second, and the two second
		// window still holds those losses for two seconds after that, so a second
		// deep drop once the cooldown expires is expected rather than surprising:
		// the controller is answering the samples it has.
		t.play(2.0, 0.2);
		CHECK(c.current() <= dropped);
		CHECK(c.snapshot().state == BitrateController::state_name::recovering);
		const uint32_t bottom = c.current();

		// A clean link from here. The window has to age the losses out (2 s) and
		// then the first rebound step waits recovery_confirm (2.5 s); the ones
		// after it wait recovery_step_interval (1 s), and each is multiplicative
		// (recovery_factor 2.0) up to the pre-drop bitrate.
		t.play(6.0, 0.2);
		CHECK(c.current() > bottom);

		t.play(40.0, 0.2);
		CHECK(c.current() == 50 * Mbit);
		CHECK(c.snapshot().state == BitrateController::state_name::steady);
	}

	// --- congestion during the rebound lowers the target (ssthresh) ------------
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(1.0, 1.3, true);
		CHECK(c.snapshot().state == BitrateController::state_name::recovering);
		const uint32_t target_before = 50 * Mbit;

		// Congestion again while rebounding.
		t.play(3.0, 0.95);
		// Then a clean link for long enough to finish rebounding.
		t.play(20.0, 0.2);
		CHECK(c.current() < target_before);
		CHECK(c.current() >= 10 * Mbit);
	}

	// --- a healthy link probes back up, one increase_step every increase_hold ---
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(1.0, 1.3, true); // deep drop
		t.play(45.0, 0.2);      // rebound, then probe all the way back up
		CHECK(c.current() == 50 * Mbit);

		// At the ceiling there is nothing left to probe for.
		const int at_ceiling = t.changes;
		t.play(30.0, 0.2);
		CHECK(t.changes == at_ceiling);
	}

	// increase_step is max(2 Mbit/s, 5% of the ceiling), and it takes
	// increase_hold (5 s) of healthy link per step.
	{
		Fixture fx(200 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(1.0, 0.9); // one gentle decrease: 200 -> 140
		CHECK(c.current() == uint32_t(200 * Mbit * BitrateController::decrease_factor));

		// A clean link from here. The window still holds the congested samples for
		// two seconds and the cooldown lets one more decrease through; what this
		// pins is the shape of the *increases* that follow, every one of which is
		// exactly max(increase_step_min, increase_step_ratio * ceiling) — 10 Mbit/s
		// at this ceiling — and never past the ceiling itself.
		const size_t before = t.history.size();
		t.play(90.0, 0.2);

		const uint32_t step = std::max<uint32_t>(BitrateController::increase_step_min,
		                                         uint32_t(200 * Mbit * BitrateController::increase_step_ratio));
		int increases = 0;
		bool steps_exact = true;
		for (size_t i = before + 1; i < t.history.size(); ++i)
		{
			if (t.history[i] <= t.history[i - 1])
				continue;
			// A rebound step is multiplicative and is not one of these; the
			// controller only reaches the additive path once it is back above
			// its recovery target, which from a gentle decrease it always is.
			++increases;
			if (t.history[i] - t.history[i - 1] != step && t.history[i] != 200 * Mbit)
				steps_exact = false;
		}
		CHECK(increases > 3);
		CHECK(steps_exact);
		CHECK(c.current() == 200 * Mbit);
	}

	// --- late frames alone are enough --------------------------------------------
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		// late_frames_decrease is 4 over the two second window.
		t.play(1.0, 0.3, false, true);
		CHECK(c.current() < 50 * Mbit);
	}
}

void part_c_hysteresis()
{
	std::printf("Part C: the hysteresis band\n");

	// Between utilisation_increase (0.60) and utilisation_decrease (0.85) nothing
	// happens at all, however long the trace runs.
	for (double u: {0.62, 0.7, 0.84})
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(20.0, u);
		CHECK(t.changes == 0);
		CHECK(c.current() == 50 * Mbit);
	}
}

void part_d_radio()
{
	std::printf("Part D: the radio trend\n");

	// A signal walking away from the access point: more than radio_fall_db (6 dB)
	// over the window and below radio_low_rssi_dbm (-65). The frame timings are
	// perfect throughout — this is the whole point, the radio is the leading
	// indicator.
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(2.0, 0.3);
		CHECK(c.current() == 50 * Mbit);

		std::optional<uint32_t> stepped;
		for (int i = 0; i < 8; ++i)
		{
			t.play(1.0, 0.3);
			// -60 dBm falling by 4 dB a second, i.e. well past the trend
			// threshold within the four second window.
			const int rssi = -60 - 4 * i;
			if (auto b = c.on_wifi_state(rssi, 400, t.now))
				stepped = b;
		}

		CHECK(stepped.has_value());
		CHECK(c.current() < 50 * Mbit);
		CHECK(c.snapshot().radio_hold);

		// While the signal keeps falling, a perfectly healthy set of frame timings
		// does not probe back up: that is the whole point of the hold. (Stop
		// feeding reports instead and the hold is released after radio_max_age,
		// which is also correct and is what evaluate() logs.)
		const uint32_t held = c.current();
		bool never_rose = true;
		for (int i = 0; i < 15; ++i)
		{
			t.play(1.0, 0.2);
			// Still falling, and still a level a radio can report: below
			// -110 dBm the sample is rejected as nonsense and the reports
			// would look like they had stopped.
			c.on_wifi_state(std::max(-105, -92 - i), 400, t.now);
			never_rose = never_rose && c.current() <= held;
		}
		CHECK(never_rose);
		CHECK(c.snapshot().radio_hold);
	}

	// The signal settling at a lower level releases the hold (radio_stable_slope
	// over radio_stable_hold), and the normal probing resumes.
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		for (int i = 0; i < 8; ++i)
		{
			t.play(1.0, 0.3);
			c.on_wifi_state(-60 - 4 * i, 400, t.now);
		}
		CHECK(c.snapshot().radio_hold);

		// Steady now, at the new low level. The smoothed level has to catch up
		// with the raw one before the slope reads flat, and then the slope has to
		// stay flat for radio_stable_hold.
		for (int i = 0; i < 15; ++i)
		{
			t.play(1.0, 0.3);
			c.on_wifi_state(-88, 400, t.now);
		}
		CHECK(not c.snapshot().radio_hold);
	}

	// Sentinels and nonsense are rejected outright: Android answers -127 when it
	// will not say.
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		for (int i = 0; i < 10; ++i)
		{
			t.play(1.0, 0.3);
			CHECK(not c.on_wifi_state(-127, 0, t.now).has_value());
			CHECK(not c.on_wifi_state(0, 0, t.now).has_value());
		}
		CHECK(not c.snapshot().radio_hold);
		CHECK(c.current() == 50 * Mbit);
	}

	// The headset switch off means no trend at all.
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		c.set_radio_aware(false);
		Trace t(c);
		for (int i = 0; i < 8; ++i)
		{
			t.play(1.0, 0.3);
			CHECK(not c.on_wifi_state(-60 - 4 * i, 400, t.now).has_value());
		}
		CHECK(c.current() == 50 * Mbit);
	}
}

void part_e_bbr()
{
	std::printf("Part E: the delivered-bandwidth law\n");

	// A link that hands 100 Mbit/s over: each eye's frame is bitrate/2 per frame
	// period, and the headset says it took `utilisation` of a frame period to
	// arrive, so the delivery rate the estimator computes is bitrate / utilisation.
	// With the shards unpaced, a frame counts as loaded above
	// unpaced_wire_fraction (10%) of a frame period.
	{
		Fixture fx(100 * Mbit, true, BitrateController::mode::bbr);
		auto & c = fx.c;
		CHECK(c.active_mode() == BitrateController::mode::bbr);
		Trace t(c);

		// 100 Mbit/s over a 90 fps frame period is 139 kB per frame, ~69 kB an eye.
		const uint32_t bytes_per_eye = 69 * 1024;
		t.play(4.0, 0.8, false, false, bytes_per_eye);

		const uint32_t estimate = c.bandwidth_estimate();
		CHECK(estimate > 0);
		// 8 * 139 kB / (0.8 * 11.1 ms) = 128 Mbit/s, and the bitrate is a gain
		// times that: 0.85 in steady, 1.25 in the startup ramp.
		CHECK(estimate > 110 * Mbit && estimate < 145 * Mbit);
		CHECK(c.current() > 90 * Mbit);
		CHECK(c.current() <= 100 * Mbit); // never above the ceiling

		// The startup ramp ends once the estimate stops growing.
		CHECK(c.snapshot().state == BitrateController::state_name::steady);
	}

	// An acute failure caps the filter to what the link is actually doing now
	// rather than waiting ten seconds for the maximum to age out, and the bitrate
	// goes to backoff_factor times that.
	{
		Fixture fx(100 * Mbit, true, BitrateController::mode::bbr);
		auto & c = fx.c;
		auto & t = fx.t;
		const uint32_t bytes_per_eye = 69 * 1024;
		t.play(4.0, 0.8, false, false, bytes_per_eye);
		const uint32_t before = c.current();
		CHECK(before > 0);

		// The link collapses: frames stop completing.
		t.play(3.0, 1.5, true, false, bytes_per_eye);
		CHECK(c.current() < before);
	}

	// No byte counts at all (the send path never reported any): there is nothing
	// to estimate, and the law falls back to v1's blind additive probe.
	{
		Fixture fx(50 * Mbit, true, BitrateController::mode::bbr);
		auto & c = fx.c;
		auto & t = fx.t;
		// Utilisation alone is not what v2 reacts to — its congestion signal is the
		// slowdown against its own estimate, and there is no estimate here. A frame
		// that never arrives is acute whatever the law.
		t.play(1.0, 1.3, true);
		CHECK(c.current() < 50 * Mbit);
		// The window keeps the losses for two seconds, so the backoff has one
		// more step in it before the link reads clean.
		t.play(4.0, 0.2);
		const uint32_t after = c.current();

		t.play(30.0, 0.2);
		CHECK(c.current() > after);
	}

	// Switching the law starts over, from the ceiling: a utilisation window says
	// nothing about a bandwidth estimate.
	{
		Fixture fx(50 * Mbit);
		auto & c = fx.c;
		auto & t = fx.t;
		t.play(1.0, 0.95);
		CHECK(c.current() < 50 * Mbit);
		auto b = c.set_client_mode(BitrateController::mode::bbr);
		CHECK(b.has_value() && *b == 50 * Mbit);
		CHECK(c.active_mode() == BitrateController::mode::bbr);
	}
}

void part_f_idr_damping()
{
	std::printf("Part F: IDR damping\n");

	// The live failure: the headset reports a lost frame on every single frame at
	// 90 fps. Without a floor that is one key frame request per frame, which is
	// what produced 534 of them in a few minutes on the RX 580.
	auto storm = [](bool damped) {
		IdrTracker idr;
		idr.reset();

		auto now = clock_t_::time_point{} + 1h;
		uint64_t frame = 0;
		int requests = 0;

		// Ten seconds of a link that loses every frame.
		for (int i = 0; i < 900; ++i)
		{
			// The session sends a frame, which is an IDR whenever one was
			// asked for and the encoder has caught up.
			const bool send_idr = idr.waiting_for_idr();
			idr.on_frame_sent(frame, send_idr);

			wivrn::from_headset::feedback f{};
			f.frame_index = frame;
			f.stream_index = 0;
			f.sent_to_decoder = 0; // never completed

			if (idr.on_feedback(f, now))
				++requests;
			if (damped && idr.poll(now))
				++requests;

			++frame;
			now += std::chrono::nanoseconds(period_ns);
		}
		return requests;
	};

	// min_recovery_interval is 500 ms, so ten seconds of total loss is at most 20
	// requests plus the one the session starts with.
	const int with_floor = storm(true);
	CHECK(with_floor <= 21);
	CHECK(with_floor >= 15);
	std::printf("  10 s of total frame loss at 90 fps: %d key frame requests (%.1f/s)\n",
	            with_floor,
	            with_floor / 10.0);

	// And the requests still come out even if the session never calls poll(): the
	// tracker only holds one back when one was granted less than the floor ago.
	CHECK(storm(false) >= 15);

	// A single loss on an otherwise healthy stream is answered at once, not after
	// half a second.
	{
		IdrTracker idr;
		idr.reset();
		auto now = clock_t_::time_point{} + 1h;

		idr.on_frame_sent(0, true);  // the session's first frame is an IDR
		idr.on_frame_sent(1, false); // ... but the tracker is waiting for feedback

		wivrn::from_headset::feedback ok{};
		ok.frame_index = 0;
		ok.sent_to_decoder = 1;
		CHECK(not idr.on_feedback(ok, now));

		idr.on_frame_sent(2, false); // now running

		wivrn::from_headset::feedback lost{};
		lost.frame_index = 3;
		lost.sent_to_decoder = 0;
		CHECK(idr.on_feedback(lost, now));
		CHECK(idr.waiting_for_idr());
		CHECK(idr.damped() == 0);

		// A second loss inside the floor is held, not dropped: it comes out of
		// poll() once the floor expires.
		idr.on_frame_sent(4, true);
		idr.on_frame_sent(5, false);
		wivrn::from_headset::feedback ok2{};
		ok2.frame_index = 4;
		ok2.sent_to_decoder = 1;
		idr.on_feedback(ok2, now);
		idr.on_frame_sent(6, false);

		wivrn::from_headset::feedback lost2{};
		lost2.frame_index = 7;
		lost2.sent_to_decoder = 0;
		CHECK(not idr.on_feedback(lost2, now + 100ms));
		CHECK(idr.damped() == 1);
		CHECK(not idr.poll(now + 200ms));
		CHECK(idr.poll(now + 600ms));
		CHECK(not idr.poll(now + 700ms)); // and only once
	}
}

} // namespace

// The controller logs through the helper's logger, which is a Windows translation
// unit. Nothing here needs a timestamp, and swallowing the lines keeps the test's
// output readable; set BITRATE_TEST_VERBOSE to see the decisions.
namespace wivrnnx::helper
{
void log_line(const char * fmt, ...)
{
	static const bool verbose = std::getenv("BITRATE_TEST_VERBOSE") != nullptr;
	if (not verbose)
		return;

	std::va_list args;
	va_start(args, fmt);
	std::printf("    | ");
	std::vprintf(fmt, args);
	std::printf("\n");
	va_end(args);
}
} // namespace wivrnnx::helper

int main()
{
	std::printf("bitrate_test: the automatic bitrate controller and the IDR floor\n\n");

	part_a_gates();
	part_b_aimd();
	part_c_hysteresis();
	part_d_radio();
	part_e_bbr();
	part_f_idr_damping();

	std::printf("\n%d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
