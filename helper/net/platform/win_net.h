/*
 * WiVRn NX - Windows port
 *
 * Winsock2 backing for the POSIX socket API that wivrn-nx/common/ is written
 * against.
 *
 * The project rule is that common/ stays genuinely shared and #ifdef-free, so
 * nothing here is ever included by name from common/. Instead the sibling
 * shadow headers (sys/socket.h, sys/uio.h, netinet/in.h, ...) sit first on the
 * include path and pull this file in, so common/'s own
 *
 *     #include <sys/socket.h>
 *
 * lands here on Windows and on the real header everywhere else.
 *
 * Two mechanisms map the calls:
 *
 *  - Overloads. Winsock declares every socket call taking SOCKET (an unsigned
 *    64-bit handle) as the first parameter; common/ passes an int fd. Declaring
 *    a global overload whose first parameter is int makes it a strictly better
 *    match for every call site in common/ (exact match on argument 1, no worse
 *    on the rest), so the call binds here without any macro trickery. That also
 *    lets the buffer parameters be void* instead of Winsock's char*, which is
 *    what makes recv(fd, uint8_t*, ...) compile at all.
 *
 *  - Macros, for the three names where an overload is impossible because the
 *    signature is identical to an existing declaration (socket, close) or the
 *    function does not exist on Windows in a compatible shape (fcntl). They are
 *    all function-like with a fixed arity, so a bare `socket` or `close` token
 *    that is not a call is left alone. `close` in particular is defined only in
 *    the shadow <unistd.h>, which common/ includes last, to keep it away from
 *    the standard library headers.
 *
 * Everything that needs a real function body lives in win_net.cpp, under
 * wivrn::win::sys_*. The global entry points below are thin forwarders, which
 * is also what keeps the implementation from recursing into itself: win_net.cpp
 * defines WIVRNNX_NET_IMPL so it never sees the global overloads.
 */

#pragma once

#ifndef _WIN32
#error "helper/net/platform is the Windows backing for common/; do not use it elsewhere"
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef _WIN32_WINNT
// Windows 10: WSAPoll, IPV6_TCLASS, inet_pton.
#define _WIN32_WINNT 0x0A00
#endif

// winsock2.h must come before any windows.h, which it includes itself.
#include <winsock2.h>
#include <ws2tcpip.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <ctime>

// ---------------------------------------------------------------------------
// Constants POSIX has and Winsock does not
// ---------------------------------------------------------------------------

// Per-call non-blocking receive. Winsock has no such flag: the emulation polls
// the socket for readability with a zero timeout first and reports EAGAIN when
// it is not ready. Picked out of the way of every Winsock MSG_* value, and
// stripped before the flags reach Winsock.
#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT 0x40000000
#endif

// Windows never raises SIGPIPE, so this one is simply dropped.
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0x20000000
#endif

// mingw-w64 already defines MSG_TRUNC (0x0100) as a WSARecvMsg *output* flag.
// Passing it into recv/recvfrom is rejected by Winsock, so it is stripped as
// well; the two call sites that use it are emulated explicitly.
#ifndef MSG_TRUNC
#define MSG_TRUNC 0x0100
#endif

// fcntl commands. Only the two combinations common/ uses are honoured.
#ifndef F_GETFD
#define F_GETFD 1
#endif
#ifndef F_SETFD
#define F_SETFD 2
#endif
#ifndef F_GETFL
#define F_GETFL 3
#endif
#ifndef F_SETFL
#define F_SETFL 4
#endif
#ifndef FD_CLOEXEC
#define FD_CLOEXEC 1
#endif
#ifndef O_NONBLOCK
#define O_NONBLOCK 0x4000
#endif

// sendmsg chunks its iovec array at this. Windows has no published ceiling on
// the WSABUF count; 1024 matches Linux and keeps the stack scratch buffer small.
#ifndef IOV_MAX
#define IOV_MAX 1024
#endif

// getsockopt(SOL_SOCKET, SO_DOMAIN) is a Linux extension, used by
// common/socket_tos.h to decide which traffic-class option applies. The value
// is private to this shim and answered from getsockname().
#ifndef SO_DOMAIN
#define SO_DOMAIN 0x7F01
#endif

#ifndef SHUT_RD
#define SHUT_RD SD_RECEIVE
#endif
#ifndef SHUT_WR
#define SHUT_WR SD_SEND
#endif
#ifndef SHUT_RDWR
#define SHUT_RDWR SD_BOTH
#endif

// poll.h
#ifndef POLLIN
#define POLLIN POLLRDNORM
#endif
#ifndef POLLOUT
#define POLLOUT POLLWRNORM
#endif
typedef unsigned long nfds_t;

// ---------------------------------------------------------------------------
// Types POSIX has and Winsock spells differently
// ---------------------------------------------------------------------------

// Scatter/gather buffer. Deliberately not a WSABUF alias: WSABUF is
// {ULONG len; CHAR * buf}, the opposite member order, and common/ builds these
// with designated initializers, which require declaration order. The conversion
// to WSABUF happens inside the sys_* implementations.
struct iovec
{
	void * iov_base;
	size_t iov_len;
};

struct msghdr
{
	void * msg_name;
	socklen_t msg_namelen;
	struct iovec * msg_iov;
	size_t msg_iovlen;
	void * msg_control;
	size_t msg_controllen;
	int msg_flags;
};

// Linux's batched send/receive. Windows has no equivalent syscall; the
// emulation loops, which costs one transition per datagram instead of one per
// batch. See win_net.cpp.
struct mmsghdr
{
	struct msghdr msg_hdr;
	unsigned int msg_len;
};

namespace wivrn::win
{

// WSAStartup/WSACleanup. Idempotent and thread safe; every socket-creating
// entry point below calls it, so a caller that only uses common/ never has to.
// Hold one of these in main() if you want the teardown to be deterministic.
void ensure_winsock();

struct winsock_scope
{
	winsock_scope();
	~winsock_scope();
	winsock_scope(const winsock_scope &) = delete;
	winsock_scope & operator=(const winsock_scope &) = delete;
};

// Translate a WSAGetLastError() code to the closest POSIX errno. common/ throws
// std::system_error{errno, std::generic_category()} straight after a failed
// call, so every wrapper here sets errno before returning -1 and the thrown
// error carries a meaningful code and message.
int errno_from_wsa(int wsa_error);

// errno = errno_from_wsa(WSAGetLastError()), then return -1.
int fail();

// 1 = readable (or at EOF), 0 = not ready, -1 = error with errno set.
int wait_readable(int fd, int timeout_ms);

int sys_socket(int domain, int type, int protocol);
int sys_close(int fd);
int sys_fcntl(int fd, int cmd, int arg);

int sys_connect(int fd, const sockaddr * addr, socklen_t len);
int sys_bind(int fd, const sockaddr * addr, socklen_t len);
int sys_listen(int fd, int backlog);
int sys_accept(int fd, sockaddr * addr, socklen_t * len);
int sys_shutdown(int fd, int how);
int sys_getsockname(int fd, sockaddr * addr, socklen_t * len);

int sys_setsockopt(int fd, int level, int optname, const void * val, socklen_t len);
int sys_getsockopt(int fd, int level, int optname, void * val, socklen_t * len);

int sys_recv(int fd, void * buf, int len, int flags);
int sys_send(int fd, const void * buf, int len, int flags);
int sys_recvfrom(int fd, void * buf, int len, int flags, sockaddr * from, socklen_t * fromlen);
int sys_sendto(int fd, const void * buf, int len, int flags, const sockaddr * to, socklen_t tolen);

ssize_t sys_readv(int fd, const struct iovec * iov, int iovcnt);
ssize_t sys_writev(int fd, const struct iovec * iov, int iovcnt);
ssize_t sys_sendmsg(int fd, const struct msghdr * msg, int flags);
ssize_t sys_recvmsg(int fd, struct msghdr * msg, int flags);
int sys_sendmmsg(int fd, struct mmsghdr * msgvec, unsigned int vlen, int flags);
int sys_recvmmsg(int fd, struct mmsghdr * msgvec, unsigned int vlen, int flags, const struct timespec * timeout);

int sys_poll(struct pollfd * fds, nfds_t nfds, int timeout);

} // namespace wivrn::win

#ifndef WIVRNNX_NET_IMPL

// ---------------------------------------------------------------------------
// Global entry points
//
// Overloads first: each takes int where Winsock takes SOCKET, which makes it
// the unambiguously better match for every call in common/.
// ---------------------------------------------------------------------------

inline int connect(int fd, const sockaddr * addr, socklen_t len)
{
	return wivrn::win::sys_connect(fd, addr, len);
}

inline int bind(int fd, const sockaddr * addr, socklen_t len)
{
	return wivrn::win::sys_bind(fd, addr, len);
}

inline int listen(int fd, int backlog)
{
	return wivrn::win::sys_listen(fd, backlog);
}

inline int accept(int fd, sockaddr * addr, socklen_t * len)
{
	return wivrn::win::sys_accept(fd, addr, len);
}

inline int shutdown(int fd, int how)
{
	return wivrn::win::sys_shutdown(fd, how);
}

inline int getsockname(int fd, sockaddr * addr, socklen_t * len)
{
	return wivrn::win::sys_getsockname(fd, addr, len);
}

inline int setsockopt(int fd, int level, int optname, const void * val, socklen_t len)
{
	return wivrn::win::sys_setsockopt(fd, level, optname, val, len);
}

inline int getsockopt(int fd, int level, int optname, void * val, socklen_t * len)
{
	return wivrn::win::sys_getsockopt(fd, level, optname, val, len);
}

inline int recv(int fd, void * buf, int len, int flags)
{
	return wivrn::win::sys_recv(fd, buf, len, flags);
}

inline int send(int fd, const void * buf, int len, int flags)
{
	return wivrn::win::sys_send(fd, buf, len, flags);
}

inline int recvfrom(int fd, void * buf, int len, int flags, sockaddr * from, socklen_t * fromlen)
{
	return wivrn::win::sys_recvfrom(fd, buf, len, flags, from, fromlen);
}

inline int sendto(int fd, const void * buf, int len, int flags, const sockaddr * to, socklen_t tolen)
{
	return wivrn::win::sys_sendto(fd, buf, len, flags, to, tolen);
}

// No Winsock counterpart, so a plain declaration is enough.

inline ssize_t readv(int fd, const struct iovec * iov, int iovcnt)
{
	return wivrn::win::sys_readv(fd, iov, iovcnt);
}

inline ssize_t writev(int fd, const struct iovec * iov, int iovcnt)
{
	return wivrn::win::sys_writev(fd, iov, iovcnt);
}

inline ssize_t sendmsg(int fd, const struct msghdr * msg, int flags)
{
	return wivrn::win::sys_sendmsg(fd, msg, flags);
}

inline ssize_t recvmsg(int fd, struct msghdr * msg, int flags)
{
	return wivrn::win::sys_recvmsg(fd, msg, flags);
}

inline int sendmmsg(int fd, struct mmsghdr * msgvec, unsigned int vlen, int flags)
{
	return wivrn::win::sys_sendmmsg(fd, msgvec, vlen, flags);
}

inline int recvmmsg(int fd, struct mmsghdr * msgvec, unsigned int vlen, int flags, const struct timespec * timeout)
{
	return wivrn::win::sys_recvmmsg(fd, msgvec, vlen, flags, timeout);
}

inline int poll(struct pollfd * fds, nfds_t nfds, int timeout)
{
	return wivrn::win::sys_poll(fds, nfds, timeout);
}

// ---------------------------------------------------------------------------
// The three that need macros.
//
// Function-like with a fixed arity, so only an actual call is rewritten and a
// `::` in front survives (the expansion is an unqualified global name).
// ---------------------------------------------------------------------------

inline int wivrnnx_socket(int domain, int type, int protocol)
{
	return wivrn::win::sys_socket(domain, type, protocol);
}

inline int wivrnnx_fcntl(int fd, int cmd, int arg)
{
	return wivrn::win::sys_fcntl(fd, cmd, arg);
}

inline int wivrnnx_close(int fd)
{
	return wivrn::win::sys_close(fd);
}

// socket(): Winsock's returns SOCKET, so an int-returning overload would be a
// conflicting redeclaration rather than an overload.
#define socket(domain, type, protocol) wivrnnx_socket((domain), (type), (protocol))

// fcntl(): does not exist on Windows, and mingw's <fcntl.h> is a real header
// this shadow chains to, so the name is introduced here rather than declared.
#define fcntl(fd, cmd, arg) wivrnnx_fcntl((fd), (cmd), (arg))

// close() is *not* defined here on purpose - see the shadow <unistd.h>.

#endif // WIVRNNX_NET_IMPL
