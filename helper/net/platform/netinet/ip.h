/*
 * WiVRn NX - Windows port: shadow for <netinet/ip.h>.
 *
 * common/ only wants the IP-level socket options (IP_TOS) and the address
 * types out of this one.
 */
#pragma once
#include "../win_net.h"
