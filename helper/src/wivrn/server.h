// The listen/accept half of the WiVRn server.
//
// Copy-adapted from wivrn::accept_connection (server/accept_connection.cpp:34-99):
// the retry-until-the-port-is-free loop and the poll-with-a-stop-token shape are
// the same. What is gone is the second file descriptor it polls — the Unix
// socket to the main-loop process — because the helper is one process, and with
// it the to_monado::stop handling that rode on it.
#pragma once

#include <atomic>
#include <optional>

#include "wivrn_sockets.h"

#include "session.h"

namespace wivrnnx::helper
{

class Bridge;
class VideoBridge;

// A dual-stack listening socket.
//
// Not wivrn::TCPListener, and this is the one place the port genuinely cannot
// reuse common/. TCPListener opens an AF_INET6 socket and binds it to
// in6addr_any (common/wivrn_sockets.cpp:213-238) without touching IPV6_V6ONLY,
// which is right on Linux — net.ipv6.bindv6only defaults to 0, so one socket
// serves both families. Winsock defaults that option the other way, so the same
// code on Windows would accept IPv6 headsets and quietly refuse every IPv4 one.
// The option has to be cleared before bind(), so it cannot be fixed after the
// fact on a TCPListener; the socket is built here instead and the accepted
// descriptor handed to wivrn::TCP, which is the part that matters.
class Listener
{
public:
	~Listener();

	Listener() = default;
	Listener(const Listener &) = delete;
	Listener & operator=(const Listener &) = delete;

	// Throws on failure, like TCPListener does.
	void open(int port);

	int get_fd() const
	{
		return fd_;
	}

	explicit operator bool() const
	{
		return fd_ >= 0;
	}

private:
	int fd_ = -1;
};

// Waits for a headset to connect. Returns an empty optional when `stop` is set
// or the listening socket failed.
std::optional<wivrn::TCP> accept_connection(Listener & listener, std::atomic<bool> & stop);

// Listen, accept, run one session, repeat, until `stop`. This is the whole
// WiVRn thread; it never returns before then.
void run_wivrn_server(const ServerOptions & options,
                      Bridge & bridge,
                      VideoBridge & video,
                      std::atomic<bool> & stop);

} // namespace wivrnnx::helper
