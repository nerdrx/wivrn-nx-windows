/*
 * WiVRn NX - Windows port: shadow for <sys/types.h>.
 *
 * Nothing to add - mingw-w64 has a real one, and this exists only so that a
 * shadowed include path cannot accidentally hide it. ssize_t comes from
 * corecrt.h underneath.
 */
#pragma once

#include_next <sys/types.h>
