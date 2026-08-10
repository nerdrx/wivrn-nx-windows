/*
 * WiVRn NX - Windows port
 *
 * Self test for wivrn-common-net.
 *
 * A successful compile only proves the shadow headers parse. This proves the
 * three things that actually matter for Phase 1:
 *
 *  1. the serialization templates instantiate for the real protocol variants,
 *     including the constexpr protocol hash over every packet type,
 *  2. the ported UDP/TCP classes produce sockets that talk to each other over
 *     loopback through Winsock - connect, accept, writev, sendmmsg, recvmmsg
 *     and the recvfrom(MSG_PEEK | MSG_TRUNC) size probe,
 *  3. the AES-CTR framing survives a round trip, with the key material derived
 *     the way the real handshake derives it (crypto.cpp + secrets.cpp).
 */

#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "crypto.h"
#include "protocol_version.h"
#include "secrets.h"
#include "win_net.h"
#include "wivrn_packets.h"
#include "wivrn_sockets.h"

namespace
{

int failures = 0;
int checks = 0;

void check(bool ok, const char * what)
{
	++checks;
	if (not ok)
		++failures;
	std::printf("  [%s] %s\n", ok ? "ok  " : "FAIL", what);
}

using packets = wivrn::from_headset::packets;

template <typename Socket>
using typed = wivrn::typed_socket<Socket, packets, packets>;

int port_of(int fd)
{
	sockaddr_in6 addr{};
	socklen_t len = sizeof(addr);
	if (getsockname(fd, (sockaddr *)&addr, &len) < 0)
		throw std::system_error{errno, std::generic_category(), "getsockname"};
	return ntohs(addr.sin6_port);
}

// common/ receives with MSG_DONTWAIT and reports "nothing there yet" by
// throwing system_error(EAGAIN), so a test that wants one packet has to wait
// for readability itself. That is exactly what the real server's poll loop
// does, and it is the code path the shim's select() emulation sits on.
template <typename Socket>
packets receive_within(typed<Socket> & socket, int timeout_ms, const char * what)
{
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

	while (std::chrono::steady_clock::now() < deadline)
	{
		if (wivrn::win::wait_readable(socket.get_fd(), 20) != 1)
			continue;

		try
		{
			if (auto packet = socket.receive())
				return *packet;
		}
		catch (const std::system_error & e)
		{
			if (e.code().value() != EAGAIN)
				throw;
		}
	}

	throw std::runtime_error(std::string("timed out waiting for ") + what);
}

bool same(const wivrn::from_headset::battery & a, const wivrn::from_headset::battery & b)
{
	return a.charge == b.charge and a.present == b.present and a.charging == b.charging;
}

// ---------------------------------------------------------------------------

void test_protocol_hash()
{
	std::printf("protocol hash\n");

	// constinit, so this is the whole serialization type graph walked at compile
	// time. If boost-pfr or magic_enum did not work here it would not link.
	std::printf("  protocol_version = %016llx\n",
	            (unsigned long long)wivrn::protocol_version);
	check(wivrn::protocol_version != 0, "constexpr protocol hash computed");

	wivrn::serialization_packet p;
	wivrn::from_headset::battery sent{.charge = 0.5f, .present = true, .charging = false};
	typed<wivrn::TCP>::serialize(p, sent);
	std::vector<std::span<uint8_t>> & spans = p;
	size_t total = 0;
	for (const auto & s: spans)
		total += s.size();
	check(total > 0, "packet serializes to a non-empty buffer");
}

// Both ends of the real handshake derive the session keys from an X25519
// exchange plus the pairing PIN. Doing it here exercises crypto.cpp's
// diffie_hellman and pbkdf2 and secrets.cpp's layout assumptions, and gives the
// socket tests below real key material.
std::pair<secrets, secrets> test_key_agreement()
{
	std::printf("key agreement (crypto.cpp + secrets.cpp)\n");

	crypto::key alice = crypto::key::generate_x25519_keypair();
	crypto::key bob = crypto::key::generate_x25519_keypair();
	check(bool(alice) and bool(bob), "x25519 keypairs generated");

	crypto::key alice_pub = crypto::key::from_public_key(alice.public_key());
	crypto::key bob_pub = crypto::key::from_public_key(bob.public_key());
	check(bool(alice_pub) and bool(bob_pub), "public keys round-trip through PEM");

	secrets from_alice{alice, bob_pub, "123456"};
	secrets from_bob{bob, alice_pub, "123456"};

	check(from_alice.control_key == from_bob.control_key, "both sides derive the same control key");
	check(from_alice.stream_key == from_bob.stream_key, "both sides derive the same stream key");
	check(from_alice.control_iv_to_headset == from_bob.control_iv_to_headset, "control IVs agree");

	secrets other{alice, bob_pub, "654321"};
	check(other.control_key != from_alice.control_key, "a different PIN gives a different key");

	return {from_alice, from_bob};
}

// TCP: TCPListener/accept, then the length-prefixed framing over sendmsg and
// recv(MSG_DONTWAIT).
void test_tcp(secrets * keys)
{
	std::printf("TCP over loopback%s\n", keys ? " (encrypted)" : "");

	wivrn::TCPListener listener{0};
	int port = port_of(listener.get_fd());

	typed<wivrn::TCP> client{in6addr_loopback, port};
	auto [server, peer] = listener.accept<typed<wivrn::TCP>>();
	(void)peer;
	check(bool(client) and bool(server), "connected and accepted");

	if (keys)
	{
		// Directions are crossed on purpose: what one end sends the other
		// receives, and a mismatch here is exactly what a wrong keystream looks
		// like on the wire.
		client.set_aes_key_and_ivs(keys->control_key,
		                           keys->control_iv_to_headset,
		                           keys->control_iv_from_headset);
		server.set_aes_key_and_ivs(keys->control_key,
		                           keys->control_iv_from_headset,
		                           keys->control_iv_to_headset);
	}

	wivrn::from_headset::battery sent{.charge = 0.42f, .present = true, .charging = true};
	client.send(wivrn::from_headset::battery{sent});

	packets got = receive_within(server, 2000, "the battery packet");
	check(std::holds_alternative<wivrn::from_headset::battery>(got), "TCP: right variant alternative");
	if (auto * b = std::get_if<wivrn::from_headset::battery>(&got))
		check(same(*b, sent), "TCP: payload survived the round trip");

	// The other direction, and a packet with a string in it, so the
	// variable-length paths of the serializer run too.
	wivrn::from_headset::wifi_state wifi{
	        .valid = true,
	        .rssi_dbm = -55,
	        .link_speed_mbps = 866,
	        .timestamp = 1234567890,
	};
	server.send(wivrn::from_headset::wifi_state{wifi});

	packets back = receive_within(client, 2000, "the wifi_state packet");
	check(std::holds_alternative<wivrn::from_headset::wifi_state>(back), "TCP: reverse direction");
	if (auto * w = std::get_if<wivrn::from_headset::wifi_state>(&back))
		check(w->rssi_dbm == wifi.rssi_dbm and w->link_speed_mbps == wifi.link_speed_mbps and w->timestamp == wifi.timestamp,
		      "TCP: reverse payload intact");

	// Several packets before a single read: this is the reassembly buffer plus
	// receive_pending, and on the send side it is sendmmsg's TCP twin.
	for (int i = 0; i < 4; ++i)
		client.send(wivrn::from_headset::battery{.charge = 0.1f * i, .present = true, .charging = false});

	// The shape the real receive loop has: drain whatever the last read already
	// buffered with receive_pending(), and only go back to the socket - which
	// is a non-blocking recv and throws EAGAIN when it is dry - once readable.
	int received = 0;
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
	while (received < 4 and std::chrono::steady_clock::now() < deadline)
	{
		while (received < 4 and server.receive_pending())
			++received;

		if (received == 4)
			break;

		if (wivrn::win::wait_readable(server.get_fd(), 20) != 1)
			continue;

		try
		{
			if (server.receive())
				++received;
		}
		catch (const std::system_error & e)
		{
			if (e.code().value() != EAGAIN)
				throw;
		}
	}
	check(received == 4, "TCP: four back-to-back packets reassembled");
}

// UDP: socket()/bind()/connect(), writev for one datagram, sendmmsg for a
// batch, recvmmsg for the receive side.
void test_udp(secrets * keys)
{
	std::printf("UDP over loopback%s\n", keys ? " (encrypted)" : "");

	typed<wivrn::UDP> receiver;
	sockaddr_in6 bind_addr{};
	bind_addr.sin6_family = AF_INET6;
	bind_addr.sin6_addr = in6addr_loopback;
	bind_addr.sin6_port = 0;
	receiver.bind(bind_addr);
	int port = port_of(receiver.get_fd());

	typed<wivrn::UDP> sender;
	sender.connect(in6addr_loopback, port);
	check(bool(sender) and bool(receiver), "UDP sockets bound and connected");

	if (keys)
	{
		sender.set_aes_key_and_ivs(keys->stream_key,
		                           keys->stream_iv_header_to_headset,
		                           keys->stream_iv_header_from_headset);
		receiver.set_aes_key_and_ivs(keys->stream_key,
		                             keys->stream_iv_header_from_headset,
		                             keys->stream_iv_header_to_headset);
	}

	// One datagram, one writev.
	wivrn::from_headset::battery sent{.charge = 0.77f, .present = true, .charging = false};
	sender.send(wivrn::from_headset::battery{sent});

	packets got = receive_within(receiver, 2000, "the UDP battery packet");
	check(std::holds_alternative<wivrn::from_headset::battery>(got), "UDP: right variant alternative");
	if (auto * b = std::get_if<wivrn::from_headset::battery>(&got))
		check(same(*b, sent), "UDP: payload survived the round trip");

	// A batch, which is sendmmsg on Linux and a WSASend loop here. The
	// serialization packets keep spans into their sources, so those have to
	// outlive the send.
	std::vector<wivrn::serialization_packet> batch(3);
	std::vector<wivrn::from_headset::battery> sources{
	        {.charge = 0.1f, .present = true, .charging = false},
	        {.charge = 0.2f, .present = true, .charging = true},
	        {.charge = 0.3f, .present = false, .charging = false},
	};
	for (size_t i = 0; i < batch.size(); ++i)
		typed<wivrn::UDP>::serialize(batch[i], sources[i]);

	sender.send(std::span<wivrn::serialization_packet>{batch});

	int received = 0;
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
	while (received < 3 and std::chrono::steady_clock::now() < deadline)
	{
		if (wivrn::win::wait_readable(receiver.get_fd(), 20) != 1)
			continue;
		try
		{
			// receive() drains the whole recvmmsg batch into the socket's
			// pending list, receive_pending() takes the rest of it.
			if (receiver.receive())
			{
				++received;
				while (received < 3 and receiver.receive_pending())
					++received;
			}
		}
		catch (const std::system_error & e)
		{
			if (e.code().value() != EAGAIN)
				throw;
		}
	}
	check(received == 3, "UDP: batched send delivered all three datagrams");
	check(receiver.dropped_datagrams() == 0, "UDP: nothing dropped");
}

// receive_from_raw() asks for the size of the next datagram with
// recvfrom(nullptr, 0, MSG_PEEK | MSG_TRUNC), which Winsock cannot answer
// directly. This is the test for that emulation.
void test_udp_receive_from()
{
	std::printf("UDP receive_from_raw (MSG_PEEK | MSG_TRUNC size probe)\n");

	wivrn::UDP receiver;
	sockaddr_in6 bind_addr{};
	bind_addr.sin6_family = AF_INET6;
	bind_addr.sin6_addr = in6addr_loopback;
	bind_addr.sin6_port = 0;
	receiver.bind(bind_addr);
	int port = port_of(receiver.get_fd());

	wivrn::UDP sender;
	sender.connect(in6addr_loopback, port);

	wivrn::serialization_packet packet;
	std::vector<uint8_t> payload(700);
	for (size_t i = 0; i < payload.size(); ++i)
		payload[i] = (uint8_t)i;
	packet.write(payload.data(), payload.size());

	size_t sent_bytes = sender.send_raw(std::move(packet));
	check(sent_bytes == payload.size(), "send_raw reported the payload size");

	check(wivrn::win::wait_readable(receiver.get_fd(), 2000) == 1, "datagram arrived");

	auto [received, from] = receiver.receive_from_raw();
	check(received.wire_size() == payload.size(), "peeked size matched the datagram");
	check(from.sin6_family == AF_INET6, "source address filled in");

	auto data = received.read_span(payload.size());
	check(data.size() == payload.size() and std::memcmp(data.data(), payload.data(), payload.size()) == 0,
	      "receive_from_raw payload intact");
}

// UDP::receive_raw reads into 2 kB slots and asks for MSG_TRUNC so it can tell
// a datagram that did not fit and drop it. Linux reports the true length there;
// Windows only reports WSAEMSGSIZE, and having already discarded the tail. This
// is the test that the substitute still makes the caller drop the datagram
// instead of handing half of one to the deserializer.
void test_udp_oversize()
{
	std::printf("UDP oversized datagram (WSAEMSGSIZE for MSG_TRUNC)\n");

	wivrn::UDP receiver;
	sockaddr_in6 bind_addr{};
	bind_addr.sin6_family = AF_INET6;
	bind_addr.sin6_addr = in6addr_loopback;
	bind_addr.sin6_port = 0;
	receiver.bind(bind_addr);
	int port = port_of(receiver.get_fd());

	wivrn::UDP sender;
	sender.connect(in6addr_loopback, port);
	sender.set_send_buffer_size(1 << 16);

	// Over the 2048-byte receive slot.
	std::vector<uint8_t> payload(4000, 0xab);
	wivrn::serialization_packet packet;
	packet.write(payload.data(), payload.size());
	sender.send_raw(std::move(packet));

	check(wivrn::win::wait_readable(receiver.get_fd(), 2000) == 1, "oversized datagram arrived");

	bool empty = true;
	try
	{
		empty = receiver.receive_raw().empty();
	}
	catch (const std::system_error & e)
	{
		if (e.code().value() != EAGAIN)
			throw;
	}

	check(empty, "oversized datagram was not handed to the deserializer");
	check(receiver.dropped_datagrams() == 1, "oversized datagram counted as dropped");
}

// Not a pass/fail: the DSCP marks are best effort by design (common/ only logs
// a refusal), and Windows ignores IP_TOS unless a registry key is set. Reported
// so the Phase 2 work knows what it is getting.
void report_tos()
{
	std::printf("type of service (informational)\n");

	wivrn::UDP udp;
	wivrn::TCPListener tcp{0};

	std::printf("  UDP  set_tos(AF41) -> %s\n", udp.set_tos(wivrn::tos::dscp_af41) ? "accepted" : "refused");
	std::printf("  TCP  set_tos(EF)   -> %s\n", tcp.set_tos(wivrn::tos::dscp_ef) ? "accepted" : "refused");
}

void test_poll()
{
	std::printf("poll/WSAPoll\n");

	wivrn::UDP receiver;
	sockaddr_in6 bind_addr{};
	bind_addr.sin6_family = AF_INET6;
	bind_addr.sin6_addr = in6addr_loopback;
	bind_addr.sin6_port = 0;
	receiver.bind(bind_addr);

	pollfd fds{};
	fds.fd = (SOCKET)receiver.get_fd();
	fds.events = POLLIN;
	int n = poll(&fds, 1, 0);
	check(n == 0, "poll on an idle socket reports nothing ready");
}

} // namespace

int main()
{
	// Not strictly needed - every socket-creating wrapper starts Winsock on
	// demand - but it makes the teardown deterministic and is what the helper
	// should do in main().
	wivrn::win::winsock_scope winsock;

	std::printf("wivrnnx-net-selftest: common/ network core on Windows\n\n");

	try
	{
		test_protocol_hash();
		std::printf("\n");

		auto [alice_secrets, bob_secrets] = test_key_agreement();
		std::printf("\n");

		test_tcp(nullptr);
		std::printf("\n");

		test_tcp(&alice_secrets);
		std::printf("\n");

		test_udp(nullptr);
		std::printf("\n");

		test_udp(&alice_secrets);
		std::printf("\n");

		test_udp_receive_from();
		std::printf("\n");

		test_udp_oversize();
		std::printf("\n");

		test_poll();
		std::printf("\n");

		report_tos();
		std::printf("\n");
	}
	catch (const std::exception & e)
	{
		std::printf("\nEXCEPTION: %s\n", e.what());
		++failures;
	}

	std::printf("%d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
