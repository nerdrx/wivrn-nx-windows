/*
 * WiVRn NX - Windows port
 *
 * Winsock2 implementations behind the POSIX shadow headers. See win_net.h for
 * how the two are wired together.
 *
 * WIVRNNX_NET_IMPL suppresses the global overloads in win_net.h, so everything
 * below calls the real Winsock entry points with no chance of recursing.
 */

#define WIVRNNX_NET_IMPL
#include "win_net.h"

#include <mswsock.h>

#include <algorithm>
#include <vector>

namespace
{

// Winsock keeps its errors out of errno entirely. common/ throws
// std::system_error{errno, std::generic_category()} straight after a failed
// call, so every wrapper translates before returning.
//
// Only the codes a socket path can actually produce are listed; anything else
// becomes EIO, which at least prints as a real error rather than as whatever
// errno happened to hold.
int translate(int wsa)
{
	switch (wsa)
	{
		case 0:
			return 0;
		case WSAEINTR:
			return EINTR;
		case WSAEBADF:
			return EBADF;
		case WSAEACCES:
			return EACCES;
		case WSAEFAULT:
			return EFAULT;
		case WSAEINVAL:
			return EINVAL;
		case WSAEMFILE:
			return EMFILE;
		// Linux makes EAGAIN and EWOULDBLOCK the same value; mingw does not
		// (EAGAIN 11, EWOULDBLOCK 140). EAGAIN is the one common/ and the
		// server compare against, so that is what a would-block maps to.
		case WSAEWOULDBLOCK:
			return EAGAIN;
		case WSAEINPROGRESS:
		case WSAEALREADY:
			return EINPROGRESS;
		case WSAENOTSOCK:
			return ENOTSOCK;
		case WSAEDESTADDRREQ:
			return EDESTADDRREQ;
		case WSAEMSGSIZE:
			return EMSGSIZE;
		case WSAEPROTOTYPE:
			return EPROTOTYPE;
		case WSAENOPROTOOPT:
			return ENOPROTOOPT;
		case WSAEPROTONOSUPPORT:
			return EPROTONOSUPPORT;
		case WSAEOPNOTSUPP:
		case WSAESOCKTNOSUPPORT:
			return EOPNOTSUPP;
		case WSAEAFNOSUPPORT:
			return EAFNOSUPPORT;
		case WSAEADDRINUSE:
			return EADDRINUSE;
		case WSAEADDRNOTAVAIL:
			return EADDRNOTAVAIL;
		case WSAENETDOWN:
			return ENETDOWN;
		case WSAENETUNREACH:
			return ENETUNREACH;
		case WSAENETRESET:
			return ENETRESET;
		case WSAECONNABORTED:
			return ECONNABORTED;
		case WSAECONNRESET:
			return ECONNRESET;
		case WSAENOBUFS:
			return ENOBUFS;
		case WSAEISCONN:
			return EISCONN;
		case WSAENOTCONN:
			return ENOTCONN;
		case WSAESHUTDOWN:
			return EPIPE;
		case WSAETIMEDOUT:
			return ETIMEDOUT;
		case WSAECONNREFUSED:
			return ECONNREFUSED;
		case WSAEHOSTDOWN:
		case WSAEHOSTUNREACH:
			return EHOSTUNREACH;
		case WSAELOOP:
			return ELOOP;
		case WSAENAMETOOLONG:
			return ENAMETOOLONG;
		case WSAENOTEMPTY:
			return ENOTEMPTY;
		case WSANOTINITIALISED:
			return ENOTSOCK;
		default:
			return EIO;
	}
}

inline SOCKET sock(int fd)
{
	return static_cast<SOCKET>(static_cast<intptr_t>(fd));
}

// MSG_DONTWAIT and MSG_NOSIGNAL are this shim's own, and mingw's MSG_TRUNC /
// MSG_CTRUNC are WSARecvMsg *output* flags that Winsock rejects as input.
inline int winsock_flags(int flags)
{
	return flags & ~(MSG_DONTWAIT | MSG_NOSIGNAL | MSG_TRUNC | MSG_CTRUNC);
}

// iovec is deliberately not layout-compatible with WSABUF (opposite member
// order), so every scatter/gather call converts. Small counts, which is all
// common/ produces for the control socket, stay on the stack.
class wsabuf_array
{
	static constexpr int inline_capacity = 32;
	WSABUF inline_bufs[inline_capacity];
	std::vector<WSABUF> heap_bufs;
	WSABUF * bufs = nullptr;
	DWORD count = 0;

public:
	wsabuf_array(const struct iovec * iov, int iovcnt)
	{
		if (iovcnt < 0)
			iovcnt = 0;
		if (iovcnt > inline_capacity)
		{
			heap_bufs.resize(iovcnt);
			bufs = heap_bufs.data();
		}
		else
		{
			bufs = inline_bufs;
		}

		count = static_cast<DWORD>(iovcnt);
		for (int i = 0; i < iovcnt; ++i)
		{
			bufs[i].len = static_cast<ULONG>(iov[i].iov_len);
			bufs[i].buf = static_cast<CHAR *>(iov[i].iov_base);
		}
	}

	WSABUF * data()
	{
		return bufs;
	}
	DWORD size() const
	{
		return count;
	}
	size_t total() const
	{
		size_t n = 0;
		for (DWORD i = 0; i < count; ++i)
			n += bufs[i].len;
		return n;
	}
};

} // namespace

namespace wivrn::win
{

int errno_from_wsa(int wsa_error)
{
	return translate(wsa_error);
}

int fail()
{
	errno = translate(WSAGetLastError());
	return -1;
}

void ensure_winsock()
{
	// Magic static: one relaxed guard load per call after the first, and the
	// destructor runs WSACleanup at exit.
	static const struct starter
	{
		bool ok = false;
		starter()
		{
			WSADATA data{};
			ok = WSAStartup(MAKEWORD(2, 2), &data) == 0;
		}
		~starter()
		{
			if (ok)
				WSACleanup();
		}
	} started;
	(void)started;
}

winsock_scope::winsock_scope()
{
	ensure_winsock();
}

winsock_scope::~winsock_scope() = default;

int wait_readable(int fd, int timeout_ms)
{
	fd_set readable;
	FD_ZERO(&readable);
	FD_SET(sock(fd), &readable);

	timeval tv{};
	timeval * ptv = nullptr;
	if (timeout_ms >= 0)
	{
		tv.tv_sec = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;
		ptv = &tv;
	}

	// nfds is ignored by Winsock.
	int n = ::select(0, &readable, nullptr, nullptr, ptv);
	if (n == SOCKET_ERROR)
		return fail();
	return n > 0 ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

int sys_socket(int domain, int type, int protocol)
{
	ensure_winsock();

	SOCKET s = ::socket(domain, type, protocol);
	if (s == INVALID_SOCKET)
		return fail();

	// common/ stores descriptors in an int and uses -1 as the empty sentinel.
	// Windows socket handles are documented to fit in the low 32 bits, but a
	// value that would alias the sentinel or truncate must not be handed out.
	intptr_t raw = static_cast<intptr_t>(s);
	if (raw != static_cast<intptr_t>(static_cast<int>(raw)) or static_cast<int>(raw) < 0)
	{
		::closesocket(s);
		errno = EMFILE;
		return -1;
	}

	if (domain == AF_INET6)
	{
		// Linux ships net.ipv6.bindv6only=0, so an AF_INET6 socket also
		// accepts v4-mapped peers, which is what WiVRn relies on for its
		// single listening socket. Windows defaults IPV6_V6ONLY the other way;
		// match Linux explicitly rather than silently losing IPv4 clients.
		DWORD v6only = 0;
		::setsockopt(s, IPPROTO_IPV6, IPV6_V6ONLY, reinterpret_cast<const char *>(&v6only), sizeof(v6only));
	}

	if (type == SOCK_DGRAM)
	{
		// Windows reports an ICMP port-unreachable from a previous sendto by
		// failing the *next* recvfrom with WSAECONNRESET. common/ turns any
		// receive error into an exception, so one unreachable peer would tear
		// the session down. Linux does not do this on unconnected UDP sockets;
		// SIO_UDP_CONNRESET off restores that behaviour.
		BOOL behaviour = FALSE;
		DWORD returned = 0;
		::WSAIoctl(s, SIO_UDP_CONNRESET, &behaviour, sizeof(behaviour), nullptr, 0, &returned, nullptr, nullptr);
	}

	return static_cast<int>(raw);
}

int sys_close(int fd)
{
	if (fd < 0)
		return 0;
	if (::closesocket(sock(fd)) == SOCKET_ERROR)
		return fail();
	return 0;
}

int sys_fcntl(int fd, int cmd, int arg)
{
	switch (cmd)
	{
		case F_SETFD:
			// FD_CLOEXEC. The Windows equivalent is the handle-inherit flag,
			// and Winsock sockets are only inherited by a child that is created
			// with bInheritHandles and an explicit handle list, so the default
			// already matches what common/ is asking for. Reported as success
			// because a failure here would abort socket construction.
			return 0;

		case F_GETFD:
			return FD_CLOEXEC;

		case F_SETFL:
		{
			u_long nonblocking = (arg & O_NONBLOCK) ? 1 : 0;
			if (::ioctlsocket(sock(fd), FIONBIO, &nonblocking) == SOCKET_ERROR)
				return fail();
			return 0;
		}

		case F_GETFL:
			// Winsock cannot report the blocking mode back; nothing in common/
			// reads it, so answer "blocking" rather than guessing.
			return 0;

		default:
			errno = EINVAL;
			return -1;
	}
}

// ---------------------------------------------------------------------------
// Plain pass-throughs, with errno translation
// ---------------------------------------------------------------------------

int sys_connect(int fd, const sockaddr * addr, socklen_t len)
{
	ensure_winsock();
	if (::connect(sock(fd), addr, len) == SOCKET_ERROR)
		return fail();
	return 0;
}

int sys_bind(int fd, const sockaddr * addr, socklen_t len)
{
	ensure_winsock();
	if (::bind(sock(fd), addr, len) == SOCKET_ERROR)
		return fail();
	return 0;
}

int sys_listen(int fd, int backlog)
{
	if (::listen(sock(fd), backlog) == SOCKET_ERROR)
		return fail();
	return 0;
}

int sys_accept(int fd, sockaddr * addr, socklen_t * len)
{
	SOCKET s = ::accept(sock(fd), addr, len);
	if (s == INVALID_SOCKET)
		return fail();

	intptr_t raw = static_cast<intptr_t>(s);
	if (raw != static_cast<intptr_t>(static_cast<int>(raw)) or static_cast<int>(raw) < 0)
	{
		::closesocket(s);
		errno = EMFILE;
		return -1;
	}
	return static_cast<int>(raw);
}

int sys_shutdown(int fd, int how)
{
	if (::shutdown(sock(fd), how) == SOCKET_ERROR)
		return fail();
	return 0;
}

int sys_getsockname(int fd, sockaddr * addr, socklen_t * len)
{
	if (::getsockname(sock(fd), addr, len) == SOCKET_ERROR)
		return fail();
	return 0;
}

int sys_setsockopt(int fd, int level, int optname, const void * val, socklen_t len)
{
	if (::setsockopt(sock(fd), level, optname, static_cast<const char *>(val), len) == SOCKET_ERROR)
		return fail();
	return 0;
}

int sys_getsockopt(int fd, int level, int optname, void * val, socklen_t * len)
{
	// SO_DOMAIN is a Linux extension with no Winsock option behind it.
	// common/socket_tos.h uses it only to decide whether IPV6_TCLASS applies,
	// and the bound address answers that just as well.
	if (level == SOL_SOCKET and optname == SO_DOMAIN)
	{
		if (not val or not len or *len < static_cast<socklen_t>(sizeof(int)))
		{
			errno = EINVAL;
			return -1;
		}

		sockaddr_storage ss{};
		socklen_t sslen = sizeof(ss);
		if (::getsockname(sock(fd), reinterpret_cast<sockaddr *>(&ss), &sslen) == SOCKET_ERROR)
			return fail();

		int domain = ss.ss_family;
		std::memcpy(val, &domain, sizeof(domain));
		*len = sizeof(domain);
		return 0;
	}

	if (::getsockopt(sock(fd), level, optname, static_cast<char *>(val), len) == SOCKET_ERROR)
		return fail();
	return 0;
}

// ---------------------------------------------------------------------------
// Transfers
// ---------------------------------------------------------------------------

int sys_recv(int fd, void * buf, int len, int flags)
{
	// Winsock has no per-call non-blocking receive. Poll for readability with a
	// zero timeout and only then read; the socket stays in blocking mode, so
	// nothing else on it changes behaviour. Safe because common/ documents the
	// receive side of both socket classes as single-threaded, and a readable
	// socket only becomes unreadable again if someone else consumes the data.
	if (flags & MSG_DONTWAIT)
	{
		int ready = wait_readable(fd, 0);
		if (ready < 0)
			return -1;
		if (ready == 0)
		{
			errno = EAGAIN;
			return -1;
		}
	}

	int n = ::recv(sock(fd), static_cast<char *>(buf), len, winsock_flags(flags));
	if (n == SOCKET_ERROR)
		return fail();
	return n;
}

int sys_send(int fd, const void * buf, int len, int flags)
{
	int n = ::send(sock(fd), static_cast<const char *>(buf), len, winsock_flags(flags));
	if (n == SOCKET_ERROR)
		return fail();
	return n;
}

int sys_recvfrom(int fd, void * buf, int len, int flags, sockaddr * from, socklen_t * fromlen)
{
	if (flags & MSG_DONTWAIT)
	{
		int ready = wait_readable(fd, 0);
		if (ready < 0)
			return -1;
		if (ready == 0)
		{
			errno = EAGAIN;
			return -1;
		}
	}

	// Linux idiom for "how big is the next datagram": recvfrom(nullptr, 0,
	// MSG_PEEK | MSG_TRUNC) returns the full length without consuming it.
	// Winsock returns WSAEMSGSIZE for a short peek buffer and never reports the
	// real length, and FIONREAD counts the whole queue rather than the head
	// datagram, so peek into a full-size scratch buffer instead and report what
	// came back. One extra copy of at most 64 kB, on a path that is only used
	// by the discovery/pairing socket.
	if (buf == nullptr and (flags & MSG_TRUNC) and (flags & MSG_PEEK))
	{
		static constexpr int max_datagram = 65536;
		static thread_local std::vector<char> scratch;
		if (scratch.size() < max_datagram)
			scratch.resize(max_datagram);

		int n = ::recvfrom(sock(fd), scratch.data(), max_datagram, MSG_PEEK, from, fromlen);
		if (n == SOCKET_ERROR)
		{
			if (WSAGetLastError() == WSAEMSGSIZE)
				return max_datagram;
			return fail();
		}
		return n;
	}

	int n = ::recvfrom(sock(fd), static_cast<char *>(buf), len, winsock_flags(flags), from, fromlen);
	if (n == SOCKET_ERROR)
		return fail();
	return n;
}

int sys_sendto(int fd, const void * buf, int len, int flags, const sockaddr * to, socklen_t tolen)
{
	int n = ::sendto(sock(fd), static_cast<const char *>(buf), len, winsock_flags(flags), to, tolen);
	if (n == SOCKET_ERROR)
		return fail();
	return n;
}

ssize_t sys_readv(int fd, const struct iovec * iov, int iovcnt)
{
	wsabuf_array bufs(iov, iovcnt);
	DWORD received = 0;
	DWORD flags = 0;

	if (::WSARecv(sock(fd), bufs.data(), bufs.size(), &received, &flags, nullptr, nullptr) == SOCKET_ERROR)
		return fail();
	return static_cast<ssize_t>(received);
}

ssize_t sys_writev(int fd, const struct iovec * iov, int iovcnt)
{
	wsabuf_array bufs(iov, iovcnt);
	DWORD sent = 0;

	// On a connected datagram socket this is one datagram assembled from every
	// buffer, same as writev on Linux.
	if (::WSASend(sock(fd), bufs.data(), bufs.size(), &sent, 0, nullptr, nullptr) == SOCKET_ERROR)
		return fail();
	return static_cast<ssize_t>(sent);
}

ssize_t sys_sendmsg(int fd, const struct msghdr * msg, int flags)
{
	if (not msg)
	{
		errno = EINVAL;
		return -1;
	}

	wsabuf_array bufs(msg->msg_iov, static_cast<int>(msg->msg_iovlen));
	DWORD sent = 0;

	int rc;
	if (msg->msg_name)
	{
		// WSASendTo rather than WSASendMsg: the latter needs a runtime-looked-up
		// extension pointer and only buys ancillary data, which common/ never
		// sends (msg_control is always null).
		rc = ::WSASendTo(sock(fd),
		                 bufs.data(),
		                 bufs.size(),
		                 &sent,
		                 winsock_flags(flags),
		                 static_cast<const sockaddr *>(msg->msg_name),
		                 msg->msg_namelen,
		                 nullptr,
		                 nullptr);
	}
	else
	{
		rc = ::WSASend(sock(fd), bufs.data(), bufs.size(), &sent, winsock_flags(flags), nullptr, nullptr);
	}

	if (rc == SOCKET_ERROR)
		return fail();
	return static_cast<ssize_t>(sent);
}

ssize_t sys_recvmsg(int fd, struct msghdr * msg, int flags)
{
	if (not msg)
	{
		errno = EINVAL;
		return -1;
	}

	if (flags & MSG_DONTWAIT)
	{
		int ready = wait_readable(fd, 0);
		if (ready < 0)
			return -1;
		if (ready == 0)
		{
			errno = EAGAIN;
			return -1;
		}
	}

	wsabuf_array bufs(msg->msg_iov, static_cast<int>(msg->msg_iovlen));
	DWORD received = 0;
	DWORD wflags = static_cast<DWORD>(winsock_flags(flags));

	int rc;
	if (msg->msg_name)
	{
		socklen_t namelen = msg->msg_namelen;
		rc = ::WSARecvFrom(sock(fd),
		                   bufs.data(),
		                   bufs.size(),
		                   &received,
		                   &wflags,
		                   static_cast<sockaddr *>(msg->msg_name),
		                   &namelen,
		                   nullptr,
		                   nullptr);
		msg->msg_namelen = namelen;
	}
	else
	{
		rc = ::WSARecv(sock(fd), bufs.data(), bufs.size(), &received, &wflags, nullptr, nullptr);
	}

	msg->msg_controllen = 0;
	msg->msg_flags = 0;

	if (rc == SOCKET_ERROR)
	{
		// The datagram did not fit. Windows has already consumed and discarded
		// the tail, which is what MSG_TRUNC asks for; flag it and report the
		// bytes that did land.
		if (WSAGetLastError() == WSAEMSGSIZE)
		{
			msg->msg_flags = MSG_TRUNC;
			return static_cast<ssize_t>(bufs.total());
		}
		return fail();
	}
	return static_cast<ssize_t>(received);
}

int sys_sendmmsg(int fd, struct mmsghdr * msgvec, unsigned int vlen, int flags)
{
	if (vlen == 0)
		return 0;

	// No batched send syscall on Windows: one WSASend per message. The
	// transition count goes from 1 to vlen, which is the main throughput cost
	// of this port on the video path.
	for (unsigned int i = 0; i < vlen; ++i)
	{
		ssize_t sent = sys_sendmsg(fd, &msgvec[i].msg_hdr, flags);
		if (sent < 0)
		{
			// Same contract as Linux: report the messages that did go out, and
			// only fail outright if the very first one did.
			if (i == 0)
				return -1;
			return static_cast<int>(i);
		}
		msgvec[i].msg_len = static_cast<unsigned int>(sent);
	}
	return static_cast<int>(vlen);
}

int sys_recvmmsg(int fd, struct mmsghdr * msgvec, unsigned int vlen, int flags, const struct timespec * timeout)
{
	(void)timeout; // common/ always passes nullptr

	if (vlen == 0)
		return 0;

	unsigned int received = 0;

	for (unsigned int i = 0; i < vlen; ++i)
	{
		// After the first datagram, never block: Linux would keep filling the
		// batch until vlen or an error, which on a live socket means waiting
		// for datagrams that have not been sent yet. Stopping at the first
		// empty read is MSG_WAITFORONE behaviour and is what the caller
		// (UDP::receive_raw, always MSG_DONTWAIT) wants anyway.
		if (i > 0 or (flags & MSG_DONTWAIT))
		{
			int ready = wait_readable(fd, 0);
			if (ready < 0)
				return received > 0 ? static_cast<int>(received) : -1;
			if (ready == 0)
			{
				if (received == 0)
				{
					errno = EAGAIN;
					return -1;
				}
				break;
			}
		}

		msghdr & hdr = msgvec[i].msg_hdr;
		wsabuf_array bufs(hdr.msg_iov, static_cast<int>(hdr.msg_iovlen));
		DWORD got = 0;
		DWORD wflags = static_cast<DWORD>(winsock_flags(flags));

		int rc;
		if (hdr.msg_name)
		{
			socklen_t namelen = hdr.msg_namelen;
			rc = ::WSARecvFrom(sock(fd),
			                   bufs.data(),
			                   bufs.size(),
			                   &got,
			                   &wflags,
			                   static_cast<sockaddr *>(hdr.msg_name),
			                   &namelen,
			                   nullptr,
			                   nullptr);
			hdr.msg_namelen = namelen;
		}
		else
		{
			rc = ::WSARecv(sock(fd), bufs.data(), bufs.size(), &got, &wflags, nullptr, nullptr);
		}

		if (rc == SOCKET_ERROR)
		{
			int err = WSAGetLastError();

			if (err == WSAEMSGSIZE)
			{
				// Truncated: the tail is gone either way. Linux with MSG_TRUNC
				// would report the true datagram length here, which Windows
				// never tells us; reporting capacity+1 keeps the caller's
				// "msg_len > buffer size means drop it" test correct, which is
				// the only thing it does with the value.
				hdr.msg_flags = MSG_TRUNC;
				msgvec[i].msg_len = static_cast<unsigned int>(bufs.total() + 1);
				++received;
				continue;
			}

			if (err == WSAEWOULDBLOCK)
			{
				if (received == 0)
				{
					errno = EAGAIN;
					return -1;
				}
				break;
			}

			if (received == 0)
				return fail();
			break;
		}

		hdr.msg_flags = 0;
		msgvec[i].msg_len = got;
		++received;
	}

	return static_cast<int>(received);
}

int sys_poll(struct pollfd * fds, nfds_t nfds, int timeout)
{
	ensure_winsock();
	int n = ::WSAPoll(fds, static_cast<ULONG>(nfds), timeout);
	if (n == SOCKET_ERROR)
		return fail();
	return n;
}

} // namespace wivrn::win
