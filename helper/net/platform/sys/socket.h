/*
 * WiVRn NX - Windows port: shadow for <sys/socket.h>.
 *
 * First on the include path ahead of common/, so common/'s POSIX includes land
 * on Winsock2. See ../win_net.h.
 */
#pragma once
#include "../win_net.h"
