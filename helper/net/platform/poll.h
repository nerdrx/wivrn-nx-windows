/*
 * WiVRn NX - Windows port: shadow for <poll.h>.
 *
 * struct pollfd and the POLL* bits are Winsock's (WSAPOLLFD); poll() forwards
 * to WSAPoll. WSAPoll only accepts sockets, never pipes or files, and before
 * Windows 10 2004 it failed to report a refused connect - both irrelevant to
 * common/, which only ever polls its own sockets.
 */
#pragma once
#include "win_net.h"
