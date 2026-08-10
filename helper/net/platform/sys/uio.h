/*
 * WiVRn NX - Windows port: shadow for <sys/uio.h>.
 *
 * struct iovec, readv/writev and IOV_MAX, mapped onto WSABUF + WSASend/WSARecv.
 */
#pragma once
#include "../win_net.h"
