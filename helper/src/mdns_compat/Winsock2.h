/*
 * WiVRn NX - Windows port: case shim for mdns.h.
 *
 * ${WIVRNNX_LINUX_REPO}/external/mdns.h spells its Windows includes
 * <Winsock2.h> and <Ws2tcpip.h>, with capitals. That is fine on Windows, where
 * the filesystem does not care, and fatal when cross compiling from Linux,
 * where mingw-w64 ships them entirely in lower case. mdns.h is upstream code in
 * a read-only checkout, so the fix goes here instead.
 *
 * This directory is only on the include path for the non-MSVC build, and only
 * for the helper. It has to behave correctly under both spellings, because on a
 * case-insensitive filesystem win_net.h's own `#include <winsock2.h>` also
 * lands here:
 *
 *  - case-insensitive host: this file is what <winsock2.h> resolved to, so the
 *    real header is further down the search path and #include_next reaches it.
 *  - case-sensitive host: nothing later on the path is called "Winsock2.h", so
 *    __has_include_next is false and the lower case spelling is asked for
 *    instead - which on such a host can never resolve back to this file.
 */
#pragma once

#if defined(__has_include_next) && __has_include_next(<Winsock2.h>)
#include_next <Winsock2.h>
#else
#include <winsock2.h>
#endif
