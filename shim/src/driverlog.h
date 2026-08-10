// Thin wrapper over vr::IVRDriverLog, in the spirit of ALVR's driverlog.
#pragma once

namespace vr
{
class IVRDriverLog;
}

namespace wivrnnx
{

// Both are safe to call before init / after cleanup (they become no-ops).
bool init_driver_log(vr::IVRDriverLog * log) noexcept;
void cleanup_driver_log() noexcept;

#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
void
driver_log(const char * format, ...) noexcept;

} // namespace wivrnnx

// All shim logging goes through this so the "[wivrnnx]" tag stays consistent.
#define WNX_LOG(...) ::wivrnnx::driver_log(__VA_ARGS__)
