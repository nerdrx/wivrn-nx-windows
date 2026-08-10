// The shard packetizer, checked against the real wire types and against the
// shipping client's own reassembler.
//
// This is the one part of the video path that can be proven without an RX 580,
// and it is also the part where a mistake is invisible: a stream sharded a byte
// differently still looks like a stream, it just never completes on the headset.
// So it is checked five ways.
//
//   Part A: structure. shard_idx runs from zero with no gaps, view_info is on
//           the first shard and only there, timing_info is on the last and only
//           there, and the payloads concatenate back to the input byte for byte.
//   Part B: boundaries against upstream. tests/fec_test.cpp in the Linux tree
//           carries WiVRn's own transcription of video_encoder::SendData's
//           slicing loop (make_frame(), tests/fec_test.cpp:101-146). The same
//           loop with FEC turned off is used here as the reference, and every
//           shard boundary has to match it exactly. Diffing against upstream's
//           own copy rather than against one written here is the whole point.
//   Part C: serialization. Every shard goes through the real
//           serialization_packet/deserialization_packet round trip, and no shard
//           may serialize to more than the max_payload_size the wire budget is
//           expressed in.
//   Part D: reassembly by the client. The deserialized shards are fed into
//           wivrn::shard_set, which is the headset's own code
//           (client/decoder/shard_set.h), in order and shuffled. It has to say
//           complete() and hand back the original bitstream.
//   Part E: the two geometry helpers next to the packetizer - the head-times-eye
//           pose composition and the neutral foveation map, which the headset
//           asserts on.
//
// Native Linux build; see run_tests.sh next to this file.

#include "../wivrn/video_out.h"

#include "decoder/shard_set.h"
#include "wivrn_serialization.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <span>
#include <vector>

using namespace wivrn;
using wivrnnx::helper::compose_pose;
using wivrnnx::helper::identity_foveation;
using wivrnnx::helper::VideoPacketizer;

namespace
{

int checks = 0;
int failures = 0;

#define CHECK(...)                                                                            \
	do                                                                                    \
	{                                                                                     \
		++checks;                                                                     \
		if (not(__VA_ARGS__))                                                         \
		{                                                                             \
			++failures;                                                           \
			std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__);   \
		}                                                                             \
	} while (0)

using data_shard = to_headset::video_stream_data_shard;
using view_info_t = data_shard::view_info_t;
using timing_info_t = data_shard::timing_info_t;

// --------------------------------------------------------------------------
// A synthetic annex-B HEVC elementary stream.
//
// Not a decodable one - nothing here parses it - but the right shape: 4-byte
// start codes, a two-byte HEVC NAL header with a plausible unit type, and
// payload bytes that are deterministic per frame so a reassembly error is a
// content mismatch and not just a length mismatch. An IDR access unit carries
// VPS/SPS/PPS in front of it, which is what AMF emits when INSERT_HEADER is set
// on the surface.
// --------------------------------------------------------------------------

enum HevcNalType : uint8_t
{
	trail_r = 1,
	idr_w_radl = 19,
	vps = 32,
	sps = 33,
	pps = 34,
};

void append_nal(std::vector<uint8_t> & out, HevcNalType type, size_t payload_bytes, uint32_t seed)
{
	out.insert(out.end(), {0x00, 0x00, 0x00, 0x01});
	// forbidden_zero_bit(1) nal_unit_type(6) nuh_layer_id(6) nuh_temporal_id_plus1(3)
	out.push_back(static_cast<uint8_t>(type << 1));
	out.push_back(0x01);

	uint32_t x = seed * 2654435761u + 1u;
	for (size_t i = 0; i < payload_bytes; ++i)
	{
		x = x * 1664525u + 1013904223u;
		uint8_t b = static_cast<uint8_t>(x >> 24);
		// Keep the emulation-prevention pattern out of the synthetic payload so
		// the byte stream stays a legal annex-B one.
		if (b == 0x00)
			b = 0x11;
		out.push_back(b);
	}
}

std::vector<uint8_t> make_bitstream(size_t target_bytes, bool idr, uint32_t seed)
{
	std::vector<uint8_t> out;
	out.reserve(target_bytes + 64);

	if (idr)
	{
		append_nal(out, vps, 20, seed);
		append_nal(out, sps, 40, seed + 1);
		append_nal(out, pps, 8, seed + 2);
	}

	const size_t header = out.size();
	const size_t slice = target_bytes > header + 6 ? target_bytes - header - 6 : 1;
	append_nal(out, idr ? idr_w_radl : trail_r, slice, seed + 3);
	return out;
}

view_info_t make_view_info()
{
	view_info_t vi{};
	vi.display_time = 1'234'567'890;
	vi.pose = {XrPosef{{0, 0, 0, 1}, {0.0315f, 0, 0}}, XrPosef{{0, 0, 0, 1}, {-0.0315f, 0, 0}}};
	vi.fov = {XrFovf{-0.9f, 0.9f, 0.9f, -0.9f}, XrFovf{-0.9f, 0.9f, 0.9f, -0.9f}};
	vi.foveation = {identity_foveation(1600, 1760), identity_foveation(1600, 1760)};
	vi.alpha = false;
	vi.quad = {};
	return vi;
}

timing_info_t make_timing_info()
{
	return timing_info_t{1000, 2000, 3000, 4000};
}

VideoPacketizer::Frame make_frame_desc(bool idr, bool has_stream_socket)
{
	VideoPacketizer::Frame f{};
	f.stream_index = 1;
	f.frame_index = 987654;
	f.idr = idr;
	f.has_stream_socket = has_stream_socket;
	f.view_info = make_view_info();
	f.timing_info = make_timing_info();
	return f;
}

// One shard, with its payload copied so it outlives the send() call exactly the
// way a serialized datagram would.
struct CapturedShard
{
	data_shard shard;
	std::vector<uint8_t> payload;
	bool control = false;
};

std::vector<CapturedShard> run_packetizer(const VideoPacketizer::Frame & frame, std::vector<uint8_t> & data)
{
	std::vector<CapturedShard> out;
	VideoPacketizer::send(frame, std::span<uint8_t>(data), [&](data_shard && shard, bool control) {
		CapturedShard captured;
		captured.payload.assign(shard.payload.begin(), shard.payload.end());
		captured.control = control;
		captured.shard = std::move(shard);
		out.push_back(std::move(captured));
		// Re-point at our own copy, since the caller's span dies with the call.
		out.back().shard.payload = out.back().payload;
	});
	return out;
}

// --------------------------------------------------------------------------
// Part B's reference: WiVRn's own transcription of SendData's slicing loop,
// tests/fec_test.cpp:130-146 in the Linux tree, with fec turned off and the
// bookkeeping that belongs to the parity builder removed. Nothing else about it
// is changed - that is the point.
// --------------------------------------------------------------------------
std::vector<size_t> reference_boundaries(size_t bytes, const view_info_t & view_info, bool fec_enabled)
{
	std::vector<size_t> sizes;

	std::optional<view_info_t> vi = view_info;
	size_t offset = 0;
	while (offset < bytes)
	{
		const size_t budget = fec::shard_payload_budget(fec_enabled) - serialized_size(vi);
		const size_t next = std::min(bytes, offset + budget);
		sizes.push_back(next - offset);
		vi.reset();
		offset = next;
	}
	return sizes;
}

template <typename T>
T round_trip(const T & value)
{
	// The pattern from tests/fec_test.cpp:62-76: serialize into the real packet
	// writer, flatten its scatter list, then read it back through the real
	// reader. Anything the wire format cannot express shows up here.
	serialization_packet packet;
	packet.serialize(value);

	std::vector<uint8_t> flat;
	for (const auto & span: static_cast<std::vector<std::span<uint8_t>> &>(packet))
		flat.insert(flat.end(), span.begin(), span.end());

	auto memory = std::shared_ptr<uint8_t[]>(new uint8_t[flat.size() + 1]);
	std::memcpy(memory.get(), flat.data(), flat.size());

	deserialization_packet in{memory, std::span<uint8_t>(memory.get(), flat.size())};
	return in.deserialize<T>();
}

size_t serialized_bytes(const data_shard & shard)
{
	serialization_packet packet;
	packet.serialize(shard);
	size_t total = 0;
	for (const auto & span: static_cast<std::vector<std::span<uint8_t>> &>(packet))
		total += span.size();
	return total;
}

// --------------------------------------------------------------------------

void part_a_structure()
{
	std::printf("Part A: shard structure\n");

	for (size_t target: {size_t(1), size_t(64), size_t(1399), size_t(1400), size_t(1401),
	                     size_t(2800), size_t(5000), size_t(300'000)})
	{
		std::vector<uint8_t> data = make_bitstream(target, false, uint32_t(target));
		auto frame = make_frame_desc(false, true);
		auto shards = run_packetizer(frame, data);

		CHECK(not shards.empty());
		if (shards.empty())
			continue;

		std::vector<uint8_t> rebuilt;
		for (size_t i = 0; i < shards.size(); ++i)
		{
			const data_shard & s = shards[i].shard;
			CHECK(s.shard_idx == i);
			CHECK(s.stream_item_idx == frame.stream_index);
			CHECK(s.frame_idx == frame.frame_index);
			CHECK(s.view_info.has_value() == (i == 0));
			CHECK(s.timing_info.has_value() == (i + 1 == shards.size()));
			CHECK(shards[i].control == false);
			rebuilt.insert(rebuilt.end(), shards[i].payload.begin(), shards[i].payload.end());
		}

		CHECK(rebuilt.size() == data.size());
		CHECK(std::memcmp(rebuilt.data(), data.data(), data.size()) == 0);
	}

	// An empty frame produces no shards at all: video_encoder.cpp:103 skips a
	// frame with no bytes, and a shard set with no last shard can never complete.
	{
		std::vector<uint8_t> empty;
		auto frame = make_frame_desc(false, true);
		auto shards = run_packetizer(frame, empty);
		CHECK(shards.empty());
	}

	// An IDR rides the control socket whole: one shard, both the view info and
	// the timings on it, and no fragmentation because TCP does its own.
	{
		std::vector<uint8_t> data = make_bitstream(200'000, true, 7);
		auto frame = make_frame_desc(true, true);
		auto shards = run_packetizer(frame, data);
		CHECK(shards.size() == 1);
		if (shards.size() == 1)
		{
			CHECK(shards[0].control == true);
			CHECK(shards[0].shard.view_info.has_value());
			CHECK(shards[0].shard.timing_info.has_value());
			CHECK(shards[0].payload.size() == data.size());
		}
	}

	// A TCP-only session (no UDP stream socket) fragments nothing either.
	{
		std::vector<uint8_t> data = make_bitstream(50'000, false, 9);
		auto frame = make_frame_desc(false, false);
		auto shards = run_packetizer(frame, data);
		CHECK(shards.size() == 1);
	}
}

void part_b_boundaries()
{
	std::printf("Part B: boundaries against upstream's own transcription of SendData\n");

	for (size_t target: {size_t(1), size_t(1399), size_t(1400), size_t(1401), size_t(4096),
	                     size_t(65'537), size_t(300'000)})
	{
		std::vector<uint8_t> data = make_bitstream(target, false, uint32_t(target + 1));
		auto frame = make_frame_desc(false, true);
		auto shards = run_packetizer(frame, data);

		const std::vector<size_t> expected = reference_boundaries(data.size(), frame.view_info, false);
		CHECK(shards.size() == expected.size());
		if (shards.size() != expected.size())
			continue;

		for (size_t i = 0; i < expected.size(); ++i)
			CHECK(shards[i].payload.size() == expected[i]);
	}

	// And the budget itself is the FEC one with parity off, which is
	// data_shard::max_payload_size exactly. If FEC is ever switched on, this is
	// the line that has to change with it.
	{
		auto frame = make_frame_desc(false, true);
		CHECK(VideoPacketizer::payload_budget(frame) == data_shard::max_payload_size);
		CHECK(VideoPacketizer::payload_budget(frame) == fec::shard_payload_budget(false));

		// The first shard is short by whatever view_info costs. Every other one
		// is short by exactly one byte, and that is not a rounding error: the
		// budget is charged serialized_size() of the *optional*, and an empty
		// optional still serializes its presence flag. Upstream does the same
		// thing (video_encoder.cpp:571 subtracts serialized_size(shard.view_info)
		// unconditionally), so a shard after the first is 1399 bytes, not 1400.
		// Worth pinning: an "obvious simplification" here would silently change
		// every shard boundary in the stream.
		std::vector<uint8_t> data = make_bitstream(10'000, false, 3);
		auto shards = run_packetizer(frame, data);
		CHECK(shards.size() > 2);
		CHECK(shards[0].payload.size() ==
		      data_shard::max_payload_size - serialized_size(shards[0].shard.view_info));
		CHECK(serialized_size(shards[1].shard.view_info) == 1);
		CHECK(shards[1].payload.size() == data_shard::max_payload_size - 1);
	}
}

void part_c_serialization()
{
	std::printf("Part C: serialization round trip\n");

	std::vector<uint8_t> data = make_bitstream(20'000, false, 11);
	auto frame = make_frame_desc(false, true);
	auto shards = run_packetizer(frame, data);
	CHECK(shards.size() > 1);

	for (const CapturedShard & captured: shards)
	{
		const to_headset::packets packet = captured.shard;
		const to_headset::packets back = round_trip(packet);
		const auto * decoded = std::get_if<data_shard>(&back);
		CHECK(decoded != nullptr);
		if (decoded == nullptr)
			continue;

		CHECK(decoded->stream_item_idx == captured.shard.stream_item_idx);
		CHECK(decoded->frame_idx == captured.shard.frame_idx);
		CHECK(decoded->shard_idx == captured.shard.shard_idx);
		CHECK(decoded->view_info.has_value() == captured.shard.view_info.has_value());
		CHECK(decoded->timing_info.has_value() == captured.shard.timing_info.has_value());
		CHECK(decoded->payload.size() == captured.payload.size());
		CHECK(std::memcmp(decoded->payload.data(), captured.payload.data(), captured.payload.size()) == 0);

		if (decoded->view_info)
		{
			CHECK(decoded->view_info->display_time == captured.shard.view_info->display_time);
			CHECK(decoded->view_info->foveation[0].x == captured.shard.view_info->foveation[0].x);
			CHECK(decoded->view_info->foveation[0].y == captured.shard.view_info->foveation[0].y);
			CHECK(decoded->view_info->alpha == false);
			CHECK(not decoded->view_info->quad.has_value());
		}
		if (decoded->timing_info)
			CHECK(decoded->timing_info->send_end == captured.shard.timing_info->send_end);

		// The budget is a payload budget, and view_info is charged against it,
		// so the whole serialized shard has to stay inside one datagram's worth.
		// The header (stream/frame/shard index, the two optional flags, the
		// payload length prefix) is what the slack above max_payload_size is.
		CHECK(serialized_bytes(captured.shard) <= data_shard::max_payload_size + 64);
	}
}

void part_d_client_reassembly()
{
	std::printf("Part D: reassembly by the client's own shard_set\n");

	auto reassemble = [](std::vector<CapturedShard> & shards, bool shuffle) {
		// The headset receives datagrams, not our structs: everything goes
		// through the wire format first, exactly as it would off a socket.
		std::vector<to_headset::packets> wire;
		for (const CapturedShard & c: shards)
			wire.push_back(round_trip(to_headset::packets{c.shard}));

		if (shuffle)
		{
			std::mt19937 rng(12345);
			std::shuffle(wire.begin(), wire.end(), rng);
		}

		shard_set set(1);
		set.reset(987654);
		for (auto & p: wire)
			set.insert(std::move(std::get<data_shard>(p)), 42);

		std::vector<uint8_t> rebuilt;
		if (set.complete())
		{
			for (const auto & s: set.data)
				rebuilt.insert(rebuilt.end(), s->payload.begin(), s->payload.end());
		}
		return std::pair{set.complete(), rebuilt};
	};

	for (size_t target: {size_t(1), size_t(1400), size_t(9000), size_t(300'000)})
	{
		std::vector<uint8_t> data = make_bitstream(target, false, uint32_t(target + 5));
		auto frame = make_frame_desc(false, true);
		auto shards = run_packetizer(frame, data);

		for (bool shuffle: {false, true})
		{
			auto [complete, rebuilt] = reassemble(shards, shuffle);
			CHECK(complete);
			CHECK(rebuilt.size() == data.size());
			if (rebuilt.size() == data.size())
				CHECK(std::memcmp(rebuilt.data(), data.data(), data.size()) == 0);
		}

		// One datagram lost and the frame must *not* complete - which is what
		// makes an IDR request the only way out, and why the IDR tracker exists.
		if (shards.size() > 2)
		{
			auto missing = shards;
			missing.erase(missing.begin() + 1);
			auto [complete, rebuilt] = reassemble(missing, false);
			CHECK(not complete);
			CHECK(rebuilt.empty());
		}
	}
}

void part_e_geometry()
{
	std::printf("Part E: pose composition and the neutral foveation map\n");

	// count_pixels() from client/scenes/stream_defoveator.cpp:653-664, which is
	// what sizes the headset's reprojection swapchain.
	auto count_pixels = [](const std::vector<uint16_t> & param) {
		int res = 0;
		const int n_ratio = (int(param.size()) - 1) / 2;
		for (size_t i = 0; i < param.size(); ++i)
			res += (std::abs(n_ratio - int(i)) + 1) * param[i];
		return res;
	};

	const auto fov = identity_foveation(1600, 1760);
	// The defoveator asserts the run count is odd (stream_defoveator.cpp:512).
	CHECK(fov.x.size() % 2 == 1);
	CHECK(fov.y.size() % 2 == 1);
	CHECK(count_pixels(fov.x) == 1600);
	CHECK(count_pixels(fov.y) == 1760);

	auto close = [](float a, float b) { return std::fabs(a - b) < 1e-5f; };

	// Identity head: the eye pose is the eye-to-head transform unchanged.
	{
		XrPosef head{{0, 0, 0, 1}, {0, 0, 0}};
		XrPosef eye{{0, 0, 0, 1}, {-0.0315f, 0, 0}};
		XrPosef out = compose_pose(head, eye);
		CHECK(close(out.position.x, -0.0315f));
		CHECK(close(out.position.y, 0));
		CHECK(close(out.position.z, 0));
		CHECK(close(out.orientation.w, 1));
	}

	// Head yawed 90 degrees about +y and moved: +x in head space becomes -z in
	// world space, and the translation adds on top.
	{
		const float s = std::sin(float(M_PI) / 4.f);
		XrPosef head{{0, s, 0, s}, {1, 2, 3}};
		XrPosef eye{{0, 0, 0, 1}, {0.0315f, 0, 0}};
		XrPosef out = compose_pose(head, eye);
		CHECK(close(out.position.x, 1.f));
		CHECK(close(out.position.y, 2.f));
		CHECK(close(out.position.z, 3.f - 0.0315f));
		CHECK(close(out.orientation.y, s));
		CHECK(close(out.orientation.w, s));
	}

	// A canted eye: the two rotations compose, they do not overwrite.
	{
		const float s = std::sin(float(M_PI) / 8.f);
		const float c = std::cos(float(M_PI) / 8.f);
		XrPosef head{{0, s, 0, c}, {0, 0, 0}};
		XrPosef eye{{0, s, 0, c}, {0, 0, 0}};
		XrPosef out = compose_pose(head, eye);
		const float half = std::sin(float(M_PI) / 4.f);
		CHECK(close(out.orientation.y, half));
		CHECK(close(out.orientation.w, half));
	}
}

} // namespace

int main()
{
	std::printf("shard_test: the video shard packetizer\n\n");

	part_a_structure();
	part_b_boundaries();
	part_c_serialization();
	part_d_client_reassembly();
	part_e_geometry();

	std::printf("\n%d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
