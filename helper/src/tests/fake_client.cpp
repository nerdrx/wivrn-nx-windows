// A synthetic WiVRn headset, for the loopback harness.
//
// The unit tests prove the pieces: the packetizer against upstream's slicing
// loop, the parity against upstream's group builder, the pacer's schedule on a
// virtual clock, the controller against synthetic feedback. What none of them
// touch is the wiring — that the session really does feed the controller, really
// does apply what it decides, really does spread a frame over the wire, and
// really does hold IDRs back. That needs a socket and a clock, so this is a
// client: it speaks the real protocol to a real wivrnnx-helper.exe running under
// Wine, receives real shards over a real socket, and answers with feedback it
// makes up.
//
// Deliberately a *native* Linux binary talking to a Windows one over loopback.
// Everything it uses is in ${LINUX_REPO}/common, which is the same code the
// helper links, so the wire format cannot drift between the two ends; and the
// client has no Windows in it at all, so a Wine problem in the helper cannot be
// hidden by the same problem in the harness.
//
// What it drives, and what it then asserts on:
//
//   (a) the bitrate reacts. Three phases: healthy, then a stretch where every
//       frame is reported as never completed, then healthy again. The synthetic
//       encoder in the helper sizes its frames from the bitrate the controller
//       decides, so the frame sizes this measures *are* the controller's output.
//   (b) pacing spreads a frame. The arrival times of the shards of one frame are
//       recorded, and their gaps have to look like ShardPacer's micro-bursts
//       rather than like one blast.
//   (c) parity only where it repairs anything: shards on a UDP session, none at
//       all on a TCP-only one.
//   (d) the IDR floor holds. The middle phase is exactly the failure that
//       produced 534 key frames in minutes on the RX 580; the key frames counted
//       here must come at about two a second, not ninety.
//   (e) the headset can actually reassemble it. Counting shards and calling a
//       frame received the moment a timing_info turns up - all this used to do -
//       passes on a stream the shipping client shows nothing at all for. So the
//       client's own shard_set and frame_window are run here, with its
//       accumulator transcribed on top (see StrictStream), and the run has to
//       come out with frames *decoded*, no frame index retired without a single
//       shard ("no shard was received"), no invalid datagram, and nothing too
//       big to cross a 1500 byte MTU - which the 64 kB loopback would hide.
//   (f) the parity is used, not merely emitted. Loopback loses nothing, so
//       fec::reconstruct never ran here; --drop-permille throws data shards away
//       inside the client so that the repair path a real Wi-Fi link exercises
//       several times a second is exercised too.
//
// Usage: fake_client [--port N] [--tcp-only] [--seconds S] [--expect-parity]
//                    [--drop-permille N]
//
// Native Linux build; see run_tests.sh next to this file.

#include "crypto.h"
#include "protocol_version.h"
#include "wivrn_packets.h"
#include "wivrn_sockets.h"

// The headset's own reassembly, verbatim: both are header only and free of
// Vulkan, so the harness runs the shipping code rather than a paraphrase of it.
#include "decoder/frame_window.h"
#include "decoder/shard_set.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <optional>
#include <system_error>
#include <poll.h>
#include <string>
#include <variant>
#include <vector>

using namespace wivrn;

namespace
{

int failures = 0;

void check(bool ok, const std::string & what)
{
	if (ok)
	{
		std::printf("  ok   %s\n", what.c_str());
		return;
	}
	++failures;
	std::printf("  FAIL %s\n", what.c_str());
}

int64_t now_ns()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
	               std::chrono::steady_clock::now().time_since_epoch())
	        .count();
}

// wivrn's TCP socket reads with MSG_DONTWAIT and throws on EAGAIN, so a receive
// is only ever valid after a poll says there is something — and even then a
// partial frame comes back as "nothing yet". Both are normal; anything else is
// not.
bool would_block(const std::system_error & e)
{
	return e.code().value() == EAGAIN || e.code().value() == EWOULDBLOCK;
}

template <typename Socket>
std::optional<to_headset::packets> wait_for_packet(Socket & sock, int timeout_ms)
{
	const int64_t deadline = now_ns() + int64_t(timeout_ms) * 1'000'000;
	for (;;)
	{
		const int64_t left = (deadline - now_ns()) / 1'000'000;
		if (left <= 0)
			return {};

		pollfd fd{};
		fd.fd = sock.get_fd();
		fd.events = POLLIN;
		if (::poll(&fd, 1, int(left)) <= 0)
			continue;

		try
		{
			if (auto packet = sock.receive())
				return packet;
		}
		catch (const std::system_error & e)
		{
			if (not would_block(e))
				throw;
		}
	}
}

template <typename... Ts>
struct overloaded : Ts...
{
	using Ts::operator()...;
};
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// One frame of one stream as it arrives.
struct Frame
{
	int64_t first_at = 0;
	int64_t last_at = 0;
	size_t shards = 0;
	size_t bytes = 0;
	bool complete = false;
	bool idr = false;
	// Arrival time of every shard, for the pacing measurement.
	std::vector<int64_t> arrivals;
};

// What one phase of the run measured.
struct Phase
{
	const char * name = "";
	size_t frames = 0;
	size_t bytes = 0;
	double mean_frame_bytes() const
	{
		return frames ? double(bytes) / double(frames) : 0;
	}
};

// --------------------------------------------------------------------------
// The headset's reassembly, transcribed
//
// Counting shards and calling a frame "arrived" the moment a timing_info shows
// up - which is all this harness used to do - is a far weaker rule than the one
// the shipping client applies, and it passes on a stream the headset would show
// nothing at all for. The real rules, and where they live:
//
//   client/scenes/stream_network.cpp:72-82   a shard whose stream_item_idx has
//        no decoder is dropped silently. decoder_count is 4 (stream.h:63).
//   client/decoder/shard_accumulator.cpp:70-101 (push_shard) files the shard by
//        frame index into a six deep window and retires whatever the window
//        pushes out, which is where "frame N was not sent because no shard was
//        received" (:44-49) comes from.
//   client/decoder/frame_window.h:96-171     the window: a frame is given up on
//        when a *complete* frame more than 3 indices newer exists, or when a
//        shard for a frame 6 newer arrives.
//   client/decoder/shard_set.h:93-124        complete() needs every index from
//        0 filled with no holes AND the last one to carry timing_info; insert()
//        drops a shard_idx >= 4096 and a duplicate.
//   client/decoder/shard_accumulator.cpp:203-258 (try_submit_front) refuses a
//        frame whose first shard has no view_info ("first shard has no
//        view_info") and only ever submits the oldest frame, in index order.
//
// Everything below is those rules with the Vulkan decoder replaced by a byte
// vector, so that a stream which would leave the headset on its lobby screen
// fails here instead.
struct StrictStream
{
	using window_t = wivrn::frame_window<wivrn::shard_set, 6, 3>;
	using data_shard = wivrn::to_headset::video_stream_data_shard;
	using parity_shard = wivrn::to_headset::video_stream_parity_shard;

	explicit StrictStream(uint8_t stream_index = 0) :
	        window(wivrn::shard_set(stream_index)) {}

	window_t window;

	// What the run came to, per stream.
	size_t frames_decoded = 0;
	// shard_accumulator.cpp:49 - the frame the headset never saw a single
	// datagram of. This is the line the Pico prints for almost every frame.
	size_t frames_no_shard = 0;
	// shard_accumulator.cpp:66 - some shards, never all of them.
	size_t frames_incomplete = 0;
	// try_submit_front, step::unusable: whole, but the pose is missing.
	size_t frames_no_view_info = 0;
	size_t shards_inserted = 0;
	size_t shards_duplicate = 0;
	size_t shards_reconstructed = 0;
	size_t bytes_decoded = 0;

	// shard_accumulator.cpp:203-258, with push_data/frame_completed replaced by
	// counting. The `submitted` bookkeeping is kept exactly: it is what stops a
	// frame revisited after a newer one moved from being fed twice.
	window_t::step try_submit_front(wivrn::shard_set & current)
	{
		using step = window_t::step;
		auto & data_shards = current.data;

		uint16_t first = current.submitted;
		uint16_t last = first;
		while (last < data_shards.size() and data_shards[last])
			++last;

		const bool frame_complete = last == data_shards.size() and
		                            not data_shards.empty() and
		                            data_shards.back()->timing_info;

		if (last > first)
		{
			if (frame_complete and not data_shards.front()->view_info)
			{
				++frames_no_view_info;
				return step::unusable;
			}

			for (size_t idx = first; idx < last; ++idx)
				bytes_decoded += data_shards[idx]->payload.size();
			current.submitted = last;
		}

		if (not frame_complete)
			return step::wait;

		if (not data_shards.front()->view_info)
		{
			++frames_no_view_info;
			return step::unusable;
		}

		++frames_decoded;
		return step::done;
	}

	// shard_accumulator.cpp:134-141
	void pump()
	{
		window.drain(
		        [this](wivrn::shard_set & set) { return try_submit_front(set); },
		        [this](wivrn::shard_set & set) { retire(set); });
	}

	// shard_accumulator.cpp:44-67, debug_why_not_sent
	void retire(const wivrn::shard_set & set)
	{
		if (set.data.empty())
			++frames_no_shard;
		else
			++frames_incomplete;
	}

	// shard_accumulator.cpp:70-101
	void push_shard(data_shard && shard, int64_t now)
	{
		const uint64_t frame_idx = shard.frame_idx;

		wivrn::shard_set * set = window.slot(frame_idx, [this](wivrn::shard_set & s) { retire(s); });
		if (not set)
			return; // older than anything still being reassembled

		if (set->insert(std::move(shard), now))
			++shards_inserted;
		else
			++shards_duplicate;

		drain_parity(*set, now);

		if (set->complete())
			window.note_complete(frame_idx);

		pump();
	}

	// shard_accumulator.cpp:105-131
	void push_parity(parity_shard && parity, int64_t now)
	{
		const uint64_t frame_idx = parity.frame_idx;
		if (frame_idx < window.front_index() or frame_idx >= window.front_index() + window_t::depth)
			return;

		wivrn::shard_set * set = window.slot(frame_idx, [](wivrn::shard_set &) {});
		if (not set or set->frame_index() != frame_idx)
			return;

		if (set->group_complete(parity) or set->parity.size() >= 64)
			return;

		set->parity.push_back(std::move(parity));
		drain_parity(*set, now);

		if (set->complete())
			window.note_complete(frame_idx);

		pump();
	}

	// shard_accumulator.cpp:143-177
	void drain_parity(wivrn::shard_set & set, int64_t now)
	{
		if (set.parity.empty())
			return;

		size_t kept = 0;
		for (size_t i = 0; i < set.parity.size(); ++i)
		{
			parity_shard & p = set.parity[i];
			if (set.group_complete(p))
				continue;
			if (set.reconstruct(p, now))
			{
				++shards_reconstructed;
				continue;
			}
			if (kept != i)
				set.parity[kept] = std::move(p);
			++kept;
		}
		set.parity.resize(kept);
	}
};

from_headset::headset_info_packet make_info()
{
	from_headset::headset_info_packet info{};
	info.render_eye_width = 1600;
	info.render_eye_height = 1760;
	info.stream_eye_width = 1600;
	info.stream_eye_height = 1760;
	info.available_refresh_rates = {72.f, 90.f};
	info.settings.preferred_refresh_rate = 90.f;
	info.settings.minimum_refresh_rate = 72.f;
	info.settings.fps_divider = 1;
	// The headset's half of the transport switches. All on: the server's own
	// switch (--no-adaptive) is what the harness varies.
	info.settings.bitrate_bps = 50'000'000;
	info.settings.bitrate_auto = true;
	info.settings.radio_aware = true;
	info.settings.smooth_pacing = true;
	info.settings.fec = true;

	for (int eye = 0; eye < 2; ++eye)
	{
		info.fov[eye] = XrFovf{-0.9f, 0.9f, 0.9f, -0.9f};
	}
	info.supported_codecs = {video_codec::h265, video_codec::h264};
	info.system_name = "fake-client";
	info.language = "en";
	info.country = "US";
	return info;
}

} // namespace

int main(int argc, char ** argv)
{
	int port = 9757;
	double seconds = 24;
	bool tcp_only = false;
	bool expect_parity = false;
	// Datagram loss, in per mille, applied to the strict reassembler only. A
	// loopback link never loses anything, so the parity shards the helper emits
	// are never actually *used* here: reconstruct() is dead code in this harness
	// unless something is taken away. Wi-Fi takes it away for real.
	int drop_permille = 0;

	for (int i = 1; i < argc; ++i)
	{
		const std::string arg = argv[i];
		if (arg == "--port" && i + 1 < argc)
			port = std::atoi(argv[++i]);
		else if (arg == "--seconds" && i + 1 < argc)
			seconds = std::atof(argv[++i]);
		else if (arg == "--tcp-only")
			tcp_only = true;
		else if (arg == "--expect-parity")
			expect_parity = true;
		else if (arg == "--drop-permille" && i + 1 < argc)
			drop_permille = std::atoi(argv[++i]);
		else
		{
			std::printf("unknown argument %s\n", arg.c_str());
			return 2;
		}
	}

	std::printf("fake_client: %s session on port %d for %.0f s\n\n",
	            tcp_only ? "TCP-only" : "UDP",
	            port,
	            seconds);

	// --- connect and handshake ---------------------------------------------
	in_addr loopback{};
	loopback.s_addr = htonl(INADDR_LOOPBACK);

	typed_socket<TCP, to_headset::packets, from_headset::packets> control(TCP{loopback, port});

	crypto::key key = crypto::key::generate_x448_keypair();

	control.send(from_headset::crypto_handshake{
	        .protocol_version = protocol_version,
	        .public_key = key.public_key(),
	        .name = "fake-client",
	});

	{
		auto packet = wait_for_packet(control, 10000);
		if (not packet)
		{
			std::printf("no crypto handshake from the server\n");
			return 1;
		}
		auto * hs = std::get_if<to_headset::crypto_handshake>(&*packet);
		if (hs == nullptr || hs->state != to_headset::crypto_handshake::crypto_state::encryption_disabled)
		{
			std::printf("the server wants encryption; run it with --no-encryption\n");
			return 1;
		}
	}

	// The client's "my sockets are switched over" confirmation.
	control.send(from_headset::crypto_handshake{
	        .protocol_version = protocol_version,
	        .public_key = key.public_key(),
	        .name = "fake-client",
	});

	int stream_port = -1;
	{
		auto packet = wait_for_packet(control, 10000);
		auto * hs = packet ? std::get_if<to_headset::handshake>(&*packet) : nullptr;
		if (hs == nullptr)
		{
			std::printf("no handshake from the server\n");
			return 1;
		}
		stream_port = hs->stream_port;
	}

	typed_socket<UDP, to_headset::packets, from_headset::packets> stream{-1};
	if (stream_port >= 0 && not tcp_only)
	{
		stream = decltype(stream)();
		sockaddr_in6 any{};
		any.sin6_family = AF_INET6;
		stream.bind(any);
		stream.connect(loopback, stream_port);
		// Sending the handshake on the stream socket is what tells the server the
		// client's UDP port; sending it on TCP would ask for a TCP-only session.
		stream.send(from_headset::handshake{});
	}
	else
	{
		control.send(from_headset::handshake{});
	}

	// The second handshake, sent once the server's stream socket exists.
	{
		auto packet = wait_for_packet(control, 10000);
		if (not packet || not std::holds_alternative<to_headset::handshake>(*packet))
		{
			std::printf("no second handshake from the server\n");
			return 1;
		}
	}

	control.send(make_info());
	std::printf("handshake complete, stream port %d\n\n", stream_port);

	// --- the session -------------------------------------------------------

	// Phases, in seconds from the start of the stream: a healthy stretch, then a
	// stretch where every frame is reported as never completed (the RX 580
	// failure), then healthy again.
	const double bad_from = seconds * 0.30;
	const double bad_to = seconds * 0.62;

	std::map<std::pair<uint8_t, uint64_t>, Frame> frames;
	Phase before{"healthy"};
	Phase bad{"reporting every frame lost"};
	Phase after{"healthy again"};

	size_t data_shards = 0;
	size_t parity_shards = 0;
	// The headset's own reassembler, one per stream. decoder_count on the client
	// is 4 (client/scenes/stream.h:63) and a shard for anything past that is
	// dropped without a word (stream_network.cpp:76-80), so count those too.
	std::array<StrictStream, 4> strict{StrictStream(0), StrictStream(1), StrictStream(2), StrictStream(3)};
	size_t shards_unknown_stream = 0;
	// Deterministic, so a failure is reproducible. Data shards only: a parity
	// shard the link ate is simply protection that was not there, which is not
	// what this is measuring.
	std::mt19937 dropper{0x5eed};
	size_t shards_dropped = 0;
	auto drop_this_one = [&] {
		if (drop_permille <= 0)
			return false;
		if (int(dropper() % 1000) >= drop_permille)
			return false;
		++shards_dropped;
		return true;
	};
	// Every datagram that came off the stream socket, by wire size, and every one
	// the deserializer refused. The refusals are what the shipping client prints
	// as "Dropped N invalid datagram(s) on the stream socket" (wivrn_client.h:389).
	std::atomic<uint64_t> stream_bytes = 0;
	size_t stream_datagrams = 0;
	size_t max_datagram = 0;
	// An IPv4 datagram that still fits a 1500 byte MTU without fragmenting. The
	// loopback this harness runs over has a 64 kB MTU and would hide an
	// over-large datagram completely; a real link would not.
	static constexpr size_t mtu_safe_payload = 1500 - 20 - 8;
	size_t oversize_datagrams = 0;
	// Frames that arrived on the control socket, which is where every key frame
	// goes: on a UDP session that is exactly the IDR count.
	size_t control_frames = 0;
	size_t control_frames_while_losing = 0;
	// Which socket the packet being dispatched came off.
	bool from_control = false;
	size_t descriptions = 0;
	// Arrival gaps within a frame, over the whole run.
	std::vector<int64_t> intra_frame_gaps;
	std::vector<int64_t> frame_spans;
	// The same, but only for the first phase. Pacing is a property of a frame big
	// enough to be worth pacing (ShardPacer::group_bytes), and the middle of the
	// run deliberately drives the bitrate — and with it the frame size — into the
	// floor, where a frame is one micro-burst and is correctly not paced at all.
	std::vector<int64_t> healthy_frame_spans;
	// Frame sizes, second by second, for the log.
	std::map<int, std::pair<size_t, size_t>> per_second;

	const int64_t started = now_ns();
	const int64_t deadline = started + int64_t(seconds * 1e9);
	int64_t last_wifi = started;

	auto elapsed = [&] { return double(now_ns() - started) / 1e9; };

	auto send_feedback = [&](uint8_t stream_index, uint64_t frame_index, const Frame & f, bool lost) {
		from_headset::feedback fb{};
		fb.frame_index = frame_index;
		fb.stream_index = stream_index;
		fb.received_first_packet = f.first_at;
		fb.received_last_packet = f.last_at;
		if (not lost)
		{
			fb.sent_to_decoder = f.last_at + 100'000;
			fb.received_from_decoder = fb.sent_to_decoder + 2'000'000;
			fb.blitted = fb.received_from_decoder + 500'000;
			fb.displayed = fb.blitted + 500'000;
			fb.times_displayed = 1;
		}
		// A frame the headset gave up on: no sent_to_decoder at all, which is the
		// same test the server's IDR tracker and its bitrate controller both use.
		control.send(std::move(fb));
	};

	auto on_shard = [&](to_headset::video_stream_data_shard && shard) {
		const int64_t at = now_ns();
		++data_shards;
		const bool control_shard = from_control;

		Frame & f = frames[{shard.stream_item_idx, shard.frame_idx}];
		if (f.shards == 0)
		{
			f.first_at = at;
			f.idr = shard.view_info.has_value() && shard.shard_idx == 0;
		}
		++f.shards;
		f.bytes += shard.payload.size();
		if (not f.arrivals.empty() && at > f.arrivals.back())
			intra_frame_gaps.push_back(at - f.arrivals.back());
		f.arrivals.push_back(at);
		f.last_at = at;

		if (not shard.timing_info)
			return;

		// The last shard of the frame: the headset's own completion rule.
		f.complete = true;
		const double t = elapsed();
		const bool lost = t >= bad_from && t < bad_to;

		if (control_shard && stream)
		{
			++control_frames;
			if (lost)
				++control_frames_while_losing;
		}

		Phase & phase = t < bad_from ? before : (lost ? bad : after);
		++phase.frames;
		phase.bytes += f.bytes;

		auto & second = per_second[int(t)];
		second.first += 1;
		second.second += f.bytes;

		if (f.shards > 1)
		{
			frame_spans.push_back(f.last_at - f.first_at);
			if (t < bad_from)
				healthy_frame_spans.push_back(f.last_at - f.first_at);
		}

		send_feedback(shard.stream_item_idx, shard.frame_idx, f, lost);
		frames.erase({shard.stream_item_idx, shard.frame_idx});
	};

	auto visitor = overloaded{
	        [&](to_headset::video_stream_data_shard && shard) {
		        // stream_network.cpp:72-82: the headset files the shard by
		        // stream_item_idx and drops it if no decoder answers to that
		        // index. Feed the real reassembler a copy before the loose
		        // accounting below consumes the shard.
		        const uint8_t idx = shard.stream_item_idx;
		        if (idx >= strict.size())
			        ++shards_unknown_stream;
		        else if (from_control or not drop_this_one())
			        strict[idx].push_shard(to_headset::video_stream_data_shard{shard}, now_ns());
		        on_shard(std::move(shard));
	        },
	        [&](to_headset::video_stream_parity_shard && parity) {
		        ++parity_shards;
		        const uint8_t idx = parity.stream_item_idx;
		        if (idx < strict.size())
			        strict[idx].push_parity(std::move(parity), now_ns());
		        else
			        ++shards_unknown_stream;
	        },
	        [&](to_headset::video_stream_description && desc) {
		        ++descriptions;
		        std::printf("  stream description: %ux%u per eye, %.0f Hz\n",
		                    desc.width,
		                    desc.height,
		                    double(desc.frame_rate));
	        },
	        [&](to_headset::timesync_query && q) {
		        control.send(from_headset::timesync_response{.query = q.query, .response = now_ns()});
	        },
	        [&](auto &&) {},
	};

	while (now_ns() < deadline)
	{
		pollfd fds[2]{};
		fds[0].fd = stream ? stream.get_fd() : -1;
		fds[0].events = POLLIN;
		fds[1].fd = control.get_fd();
		fds[1].events = POLLIN;

		if (::poll(fds, 2, 5) < 0)
			break;

		// One datagram's worth of wire bytes, so that the size distribution of
		// what the helper actually put on the socket can be asserted on.
		auto note_datagram = [&](uint64_t before) {
			const uint64_t size = stream_bytes.load() - before;
			if (size == 0)
				return;
			++stream_datagrams;
			max_datagram = std::max<size_t>(max_datagram, size_t(size));
			if (size > mtu_safe_payload)
				++oversize_datagrams;
		};

		try
		{
			from_control = false;
			while (stream)
			{
				const uint64_t before = stream_bytes.load();
				auto packet = stream.receive_pending_lossy(&stream_bytes);
				if (not packet)
					break;
				note_datagram(before);
				std::visit(visitor, std::move(*packet));
			}

			from_control = true;
			while (auto packet = control.receive_pending())
				std::visit(visitor, std::move(*packet));

			if (fds[0].fd >= 0 && (fds[0].revents & POLLIN))
			{
				from_control = false;
				const uint64_t before = stream_bytes.load();
				if (auto packet = stream.receive_lossy(&stream_bytes))
				{
					note_datagram(before);
					std::visit(visitor, std::move(*packet));
				}
			}
			if (fds[1].revents & POLLIN)
			{
				from_control = true;
				if (auto packet = control.receive())
					std::visit(visitor, std::move(*packet));
			}
		}
		catch (const std::system_error & e)
		{
			if (not would_block(e))
			{
				std::printf("session ended: %s\n", e.what());
				break;
			}
		}
		catch (const std::exception & e)
		{
			std::printf("session ended: %s\n", e.what());
			break;
		}

		// A Wi-Fi report a second, at a level and a PHY rate that say nothing is
		// wrong: the radio path must not step in and confuse (a).
		if (now_ns() - last_wifi > 1'000'000'000)
		{
			last_wifi = now_ns();
			control.send(from_headset::wifi_state{
			        .valid = true,
			        .rssi_dbm = -45,
			        .link_speed_mbps = 866,
			        .timestamp = now_ns(),
			});
		}
	}

	// --- what happened -----------------------------------------------------

	std::printf("\n  per second (eye-frames, mean kB per frame):\n");
	for (const auto & [second, counts]: per_second)
	{
		const char * phase = second < int(bad_from) ? "healthy"
		                     : second < int(bad_to) ? "LOSING FRAMES"
		                                            : "healthy";
		std::printf("    %2d s  %3zu frames  %6.1f kB  %s\n",
		            second,
		            counts.first,
		            counts.second / 1024.0 / double(counts.first ? counts.first : 1),
		            phase);
	}

	std::sort(intra_frame_gaps.begin(), intra_frame_gaps.end());
	std::sort(frame_spans.begin(), frame_spans.end());
	std::sort(healthy_frame_spans.begin(), healthy_frame_spans.end());
	auto percentile = [](const std::vector<int64_t> & v, double p) -> int64_t {
		if (v.empty())
			return 0;
		return v[std::min(v.size() - 1, size_t(p * double(v.size())))];
	};

	std::printf("\n  %zu data shards, %zu parity shards, %zu eye-frames, %zu stream descriptions\n",
	            data_shards,
	            parity_shards,
	            before.frames + bad.frames + after.frames,
	            descriptions);
	std::printf("  intra-frame shard gaps: p50 %lld us, p90 %lld us, max %lld us over %zu gaps\n",
	            static_cast<long long>(percentile(intra_frame_gaps, 0.5) / 1000),
	            static_cast<long long>(percentile(intra_frame_gaps, 0.9) / 1000),
	            static_cast<long long>(intra_frame_gaps.empty() ? 0 : intra_frame_gaps.back() / 1000),
	            intra_frame_gaps.size());
	std::printf("  frame spans: p50 %lld us, p90 %lld us over the run; p50 %lld us, p90 %lld us "
	            "while the frames were full size (a frame period is 11111 us)\n",
	            static_cast<long long>(percentile(frame_spans, 0.5) / 1000),
	            static_cast<long long>(percentile(frame_spans, 0.9) / 1000),
	            static_cast<long long>(percentile(healthy_frame_spans, 0.5) / 1000),
	            static_cast<long long>(percentile(healthy_frame_spans, 0.9) / 1000));
	std::printf("  mean frame size: %.1f kB healthy, %.1f kB while losing, %.1f kB after\n\n",
	            before.mean_frame_bytes() / 1024,
	            bad.mean_frame_bytes() / 1024,
	            after.mean_frame_bytes() / 1024);

	// --- assertions --------------------------------------------------------

	check(descriptions >= 1, "the server described its video stream");
	check(before.frames > 100, "frames arrived: " + std::to_string(before.frames) + " in the first phase");

	// --- what the headset's own reassembler made of it ----------------------
	//
	// This is the half that the Pico failure showed up in and this harness did
	// not: shards can arrive, be counted and carry a timing_info while the
	// shipping client still reassembles nothing at all.
	if (stream)
	{
		size_t decoded = 0, no_shard = 0, incomplete = 0, no_view = 0;
		size_t inserted = 0, duplicate = 0, rebuilt = 0;
		for (const StrictStream & s: strict)
		{
			decoded += s.frames_decoded;
			no_shard += s.frames_no_shard;
			incomplete += s.frames_incomplete;
			no_view += s.frames_no_view_info;
			inserted += s.shards_inserted;
			duplicate += s.shards_duplicate;
			rebuilt += s.shards_reconstructed;
		}

		std::printf("  headset reassembly: %zu frames decoded, %zu with no shard at all, "
		            "%zu incomplete, %zu without view_info\n",
		            decoded,
		            no_shard,
		            incomplete,
		            no_view);
		std::printf("  shards: %zu inserted, %zu duplicates, %zu rebuilt from parity, "
		            "%zu for a stream with no decoder\n",
		            inserted,
		            duplicate,
		            rebuilt,
		            shards_unknown_stream);
		std::printf("  stream socket: %zu datagrams, %llu invalid, largest %zu bytes, "
		            "%zu over the %zu byte MTU-safe payload\n",
		            stream_datagrams,
		            static_cast<unsigned long long>(stream.dropped_datagrams()),
		            max_datagram,
		            oversize_datagrams,
		            mtu_safe_payload);

		// Every datagram the helper puts on the stream socket has to decode.
		// The shipping client counts these and prints "Dropped N invalid
		// datagram(s) on the stream socket" (client/wivrn_client.h:388-389);
		// nothing here used to look at the counter at all.
		check(stream.dropped_datagrams() == 0,
		      "every datagram on the stream socket deserialized");

		// The exact line the Pico prints for almost every frame. The window
		// starts at index 0 and restart()s onto whatever the first frame index
		// really is (frame_window.h:110-119), retiring at most `depth` empty
		// slots per stream on the way; anything past that is the failure.
		const size_t startup_allowance = StrictStream::window_t::depth * strict.size();
		check(no_shard <= startup_allowance,
		      "no frame index was opened and retired without a single shard "
		      "(\"no shard was received\"): " +
		              std::to_string(no_shard) + ", allowance " + std::to_string(startup_allowance));

		check(no_view == 0, "every complete frame carried view_info on its first shard");
		check(shards_unknown_stream == 0, "every shard named a stream the headset has a decoder for");

		// The headset shows a picture for a frame it decoded, not for one it
		// merely received datagrams of.
		check(decoded > 100,
		      "the headset's own reassembler decoded " + std::to_string(decoded) + " frames");
		check(incomplete <= decoded / 20,
		      "at most 5% of frames were retired incomplete (" + std::to_string(incomplete) +
		              " of " + std::to_string(incomplete + decoded) + ")");

		// Loopback has a 64 kB MTU, so a datagram too big for a real 1500 byte
		// link is invisible here unless it is asserted on.
		check(oversize_datagrams == 0,
		      "no datagram was too large to cross a 1500 byte MTU link unfragmented");

		// The parity shards actually repairing something. Nothing is ever lost
		// on loopback, so without injected loss the helper's parity is emitted,
		// counted and thrown away without fec::reconstruct ever running - the
		// one thing a real Wi-Fi link does with it every second.
		if (drop_permille > 0)
		{
			std::printf("  injected loss: %zu of %zu data shards dropped (%.1f permille), "
			            "%zu rebuilt from parity\n",
			            shards_dropped,
			            data_shards,
			            data_shards ? 1000.0 * double(shards_dropped) / double(data_shards) : 0.0,
			            rebuilt);

			check(shards_dropped > 0, "the harness really dropped datagrams");
			// A group is 8 data shards and one parity repairs a single erasure,
			// so at a couple of per mille almost every hole is a lone one.
			check(rebuilt > shards_dropped / 2,
			      "parity rebuilt most of what the link ate (" + std::to_string(rebuilt) +
			              " of " + std::to_string(shards_dropped) + ")");
			check(decoded > 100,
			      "and the headset still decoded " + std::to_string(decoded) + " frames through the loss");
		}
	}

	// (a) the bitrate reacted. The synthetic encoder's frames are sized from the
	// controller's bitrate, so a smaller frame is a lower bitrate, full stop.
	// Compared second by second rather than phase by phase: a phase mean smears
	// the decay across the phase, and what matters is that the bitrate went down
	// and then came back up.
	double smallest = 1e18;
	double last_second = 0;
	for (const auto & [second, counts]: per_second)
	{
		if (counts.first == 0)
			continue;
		const double mean = double(counts.second) / double(counts.first);
		// Ignore the first second: the stream is still starting up.
		if (second > 0)
			smallest = std::min(smallest, mean);
		last_second = mean;
	}

	check(smallest < 0.6 * before.mean_frame_bytes(),
	      "the bitrate came down while frames were being lost");
	check(last_second > 1.2 * smallest,
	      "and climbed back once the link read healthy again");

	// (b) pacing. A frame that left in one blast arrives in one blast: every shard
	// within a few tens of microseconds of the last. Paced, the shards of a frame
	// are spread over a good fraction of a millisecond at least.
	check(not intra_frame_gaps.empty(), "frames arrived in more than one shard");
	check(percentile(healthy_frame_spans, 0.5) > 500'000,
	      "a full size frame takes more than half a millisecond to arrive (paced, not blasted)");
	check(percentile(healthy_frame_spans, 0.95) < 11'111'111,
	      "and still less than a frame period");

	// (c) parity, only where something can be lost.
	if (expect_parity)
		check(parity_shards > 0, "parity shards arrived on the UDP session");
	else
		check(parity_shards == 0, "no parity shards on a TCP-only session");

	// (d) the IDR floor. Every key frame goes out on the control socket, so on a
	// UDP session counting the frames that arrived there counts them exactly. Two
	// eye-frames per key frame, and the floor is one key frame per 500 ms, so a
	// stretch that loses every single frame may produce four eye-frames a second
	// and no more — against the 180 a second an undamped tracker asks for.
	if (stream)
	{
		const double losing_seconds = bad_to - bad_from;
		const double rate = control_frames_while_losing / losing_seconds;
		std::printf("  key frames while losing every frame: %zu eye-frames over %.0f s (%.1f/s)\n",
		            control_frames_while_losing,
		            losing_seconds,
		            rate);
		check(rate <= 5.0, "the IDR floor held the key frame rate down");
		check(control_frames_while_losing > 0, "and key frames were still produced");
	}

	std::printf("\n%s\n", failures == 0 ? "all checks passed" : "FAILURES");
	return failures == 0 ? 0 : 1;
}
