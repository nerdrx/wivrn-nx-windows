/*
 * WiVRn NX - Windows port: shadow for <unistd.h>.
 *
 * mingw-w64 has a real <unistd.h> (it pulls in io.h/process.h/getopt.h), so
 * this chains to it and then adds close().
 *
 * close() is the one name that has to be a macro *and* cannot live in
 * win_net.h: mingw's <io.h> declares `int close(int)` for CRT file descriptors
 * with exactly the signature we would want to overload, and calling that on a
 * Winsock handle would close an unrelated CRT descriptor. Defining it here
 * rather than in win_net.h keeps the macro out of every translation unit that
 * only wants sockets, and in common/wivrn_sockets.cpp <unistd.h> is the last
 * include, so no standard library header is ever parsed with it in effect.
 *
 * It is function-like with one parameter, so a bare `close` token is left
 * alone; a member call spelled `x.close()` would be rewritten, which is why
 * this header must stay the last one included.
 */
#pragma once

#include_next <unistd.h>

#include "win_net.h"

#define close(fd) wivrnnx_close(fd)
