#include "driverlog.h"

#include "openvr_driver_wrap.h"

#include <atomic>
#include <cstdarg>
#include <cstdio>

namespace wivrnnx
{
namespace
{
std::atomic<vr::IVRDriverLog *> g_log{nullptr};
} // namespace

bool init_driver_log(vr::IVRDriverLog * log) noexcept
{
	g_log.store(log, std::memory_order_release);
	return log != nullptr;
}

void cleanup_driver_log() noexcept
{
	g_log.store(nullptr, std::memory_order_release);
}

void driver_log(const char * format, ...) noexcept
{
	vr::IVRDriverLog * const log = g_log.load(std::memory_order_acquire);
	if (log == nullptr)
		return;

	char message[1024];
	int prefix = std::snprintf(message, sizeof(message), "[wivrnnx] ");
	if (prefix < 0)
		return;

	va_list args;
	va_start(args, format);
	std::vsnprintf(message + prefix, sizeof(message) - static_cast<size_t>(prefix), format, args);
	va_end(args);

	log->Log(message);
}

} // namespace wivrnnx
