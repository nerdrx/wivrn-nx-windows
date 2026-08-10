// Include openvr_driver.h through this header. The vendored Valve header has a
// pile of unused-parameter warnings in its inline default implementations, and
// they would otherwise drown out our own diagnostics under -Wextra / /W4.
// (Same trick as ALVR's alvr_server/openvr_driver_wrap.h.)
#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4100) // unreferenced formal parameter
#endif

#include <openvr_driver.h>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif
