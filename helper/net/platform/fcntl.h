/*
 * WiVRn NX - Windows port: shadow for <fcntl.h>.
 *
 * mingw-w64 has a real <fcntl.h> (open/O_* for CRT file descriptors) that the
 * standard library relies on, so this chains to it with #include_next and only
 * adds the parts POSIX has and Windows does not: the fcntl() commands, and
 * fcntl() itself as a macro onto the Winsock equivalents.
 */
#pragma once

#include_next <fcntl.h>

#include "win_net.h"
