#include "server.h"

#include "win_net.h" // winsock2 first

#include <poll.h>

#include <chrono>
#include <thread>

#include "../bridge.h"
#include "../log.h"
#include "../video_bridge.h"

using namespace std::chrono_literals;

namespace wivrnnx::helper
{

Listener::~Listener()
{
	if (fd_ >= 0)
		wivrn::win::sys_close(fd_);
}

void Listener::open(int port)
{
	if (fd_ >= 0)
	{
		wivrn::win::sys_close(fd_);
		fd_ = -1;
	}

	const int fd = socket(AF_INET6, SOCK_STREAM, 0);
	if (fd < 0)
		throw std::system_error{errno, std::generic_category(), "socket"};

	try
	{
		int reuse_addr = 1;
		if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)) < 0)
			throw std::system_error{errno, std::generic_category(), "SO_REUSEADDR"};

		// The whole reason this function exists. Must precede bind().
		int v6only = 0;
		if (setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only)) < 0)
			throw std::system_error{errno, std::generic_category(), "IPV6_V6ONLY"};

		sockaddr_in6 addr{};
		addr.sin6_family = AF_INET6;
		addr.sin6_port = htons(static_cast<uint16_t>(port));
		addr.sin6_addr = in6addr_any;

		if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
			throw std::system_error{errno, std::generic_category(), "bind"};

		if (listen(fd, 1) < 0)
			throw std::system_error{errno, std::generic_category(), "listen"};
	}
	catch (...)
	{
		// close() is not spelled that way here on purpose: the shadow that
		// redirects it to Winsock only exists inside <unistd.h>, which this
		// file does not include, and mingw's CRT close() would shut an
		// unrelated file descriptor.
		wivrn::win::sys_close(fd);
		throw;
	}

	fd_ = fd;
}

std::optional<wivrn::TCP> accept_connection(Listener & listener, std::atomic<bool> & stop)
{
	pollfd fds{};
	fds.fd = listener.get_fd();
	fds.events = POLLIN;

	while (not stop.load())
	{
		const int r = ::poll(&fds, 1, 100);
		if (r < 0)
		{
			log_line("accept: poll failed (errno %d)", errno);
			return {};
		}

		if (fds.revents & POLLIN)
		{
			sockaddr_in6 peer{};
			socklen_t len = sizeof(peer);
			const int fd = accept(listener.get_fd(), reinterpret_cast<sockaddr *>(&peer), &len);
			if (fd < 0)
			{
				log_line("accept failed (errno %d)", errno);
				return {};
			}
			return wivrn::TCP(fd);
		}
	}

	return {};
}

void run_wivrn_server(const ServerOptions & options,
                      Bridge & bridge,
                      VideoBridge & video,
                      std::atomic<bool> & stop)
{
	wivrn::win::winsock_scope winsock;

	Listener listener;

	// The port may still be held for a moment by a previous run; the Linux
	// server has the same retry loop for the same reason.
	while (not stop.load())
	{
		try
		{
			listener.open(options.port);
			break;
		}
		catch (const std::exception & e)
		{
			log_line("waiting for TCP port %d: %s", options.port, e.what());
			std::this_thread::sleep_for(500ms);
		}
	}

	if (not listener)
		return;

	log_line("listening for headsets on TCP port %d", options.port);

	while (not stop.load())
	{
		auto tcp = accept_connection(listener, stop);
		if (not tcp)
			break;

		try
		{
			Session session(std::move(*tcp), options, bridge, video, stop);
			session.run();
		}
		catch (const incorrect_pin &)
		{
			log_line("pairing failed: the headset entered the wrong PIN");
		}
		catch (const std::system_error & e)
		{
			log_line("session failed: %s (%s error %d)",
			         e.what(),
			         e.code().category().name(),
			         e.code().value());
		}
		catch (const std::exception & e)
		{
			log_line("session failed: %s", e.what());
		}

		// ~Session does this too, but a constructor that threw (a wrong PIN, a
		// protocol mismatch) never gets a destructor, and by then the headset
		// info may already have marked the video path active. An encoder left
		// running for a client that never finished connecting would encode into
		// a queue nothing drains.
		video.clear_client();

		// Every device is marked gone by ~Session; say so once here too so the
		// log reads in order.
		log_line("headset disconnected, waiting for the next one");
	}
}

} // namespace wivrnnx::helper
