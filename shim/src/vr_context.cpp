#include "vr_context.h"

#include <atomic>
#include <cstdio>

namespace wivrnnx::vrctx
{
namespace
{

std::atomic<bool> g_live{false};
char g_missing[128] = "not queried";

// True only if it is safe to run one of openvr_driver.h's lazily-caching
// accessors: they all dereference VRDriverContext() unconditionally.
bool context_usable() noexcept
{
	return g_live.load(std::memory_order_acquire) && vr::VRDriverContext() != nullptr;
}

} // namespace

void set_live(bool live) noexcept
{
	g_live.store(live, std::memory_order_release);
}

bool live() noexcept
{
	return context_usable();
}

vr::IVRServerDriverHost * host() noexcept
{
	if (!context_usable())
		return nullptr;
	return vr::VRServerDriverHost();
}

vr::IVRProperties * properties_raw() noexcept
{
	if (!context_usable())
		return nullptr;
	return vr::VRPropertiesRaw();
}

vr::CVRPropertyHelpers * properties() noexcept
{
	// The helper is a value inside the module context and is never null itself;
	// it is only usable once the raw interface behind it exists.
	if (properties_raw() == nullptr)
		return nullptr;
	return vr::VRProperties();
}

vr::IVRDriverInput * input() noexcept
{
	if (!context_usable())
		return nullptr;
	return vr::VRDriverInput();
}

vr::IVRDriverLog * driver_log() noexcept
{
	if (!context_usable())
		return nullptr;
	return vr::VRDriverLog();
}

vr::IVRSettings * settings() noexcept
{
	if (!context_usable())
		return nullptr;
	return vr::VRSettings();
}

const char * missing_interfaces() noexcept
{
	if (!context_usable())
	{
		std::snprintf(g_missing, sizeof(g_missing), "context is not live");
		return g_missing;
	}

	char list[128] = {};
	size_t used = 0;
	const auto append = [&](const char * name) {
		const int written = std::snprintf(list + used, sizeof(list) - used, used == 0 ? "%s" : ",%s", name);
		if (written > 0)
			used += static_cast<size_t>(written);
	};

	if (vr::VRServerDriverHost() == nullptr)
		append("IVRServerDriverHost");
	if (vr::VRPropertiesRaw() == nullptr)
		append("IVRProperties");
	if (vr::VRDriverInput() == nullptr)
		append("IVRDriverInput");
	if (vr::VRDriverLog() == nullptr)
		append("IVRDriverLog");
	if (vr::VRSettings() == nullptr)
		append("IVRSettings");

	std::snprintf(g_missing, sizeof(g_missing), "%s", used == 0 ? "none" : list);
	return g_missing;
}

} // namespace wivrnnx::vrctx
