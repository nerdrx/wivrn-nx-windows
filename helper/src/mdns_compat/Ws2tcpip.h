/*
 * WiVRn NX - Windows port: case shim for mdns.h. See Winsock2.h beside it.
 */
#pragma once

#if defined(__has_include_next) && __has_include_next(<Ws2tcpip.h>)
#include_next <Ws2tcpip.h>
#else
#include <ws2tcpip.h>
#endif
