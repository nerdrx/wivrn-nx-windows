// Null-safe access to the OpenVR driver-context interfaces.
//
// Why this exists rather than calling vr::VRServerDriverHost() directly:
// openvr_driver.h's accessors are lazily-caching inline functions that do
//
//     if ( m_pVRServerDriverHost == nullptr )
//         m_pVRServerDriverHost = VRDriverContext()->GetGenericInterface(...)
//
// with no check on VRDriverContext() itself. CleanupDriverContext() sets that
// pointer back to nullptr and clears every cached interface, so a single call
// to vr::VRServerDriverHost() after Cleanup() -- from a lingering driver thread,
// or from a RunFrame() that raced the teardown -- dereferences a null pointer
// and takes the whole of vrserver down with it. Reproduced under Wine.
//
// vr::VRProperties() has a second trap: it never returns nullptr. It hands back
// the address of a CVRPropertyHelpers value that wraps a possibly-null
// IVRProperties, so `if (vr::VRProperties() == nullptr)` is always false and the
// null shows up later, inside the helper. Only VRPropertiesRaw() tells the
// truth.
//
// Everything here returns nullptr instead of crashing. Callers degrade.
#pragma once

#include "openvr_driver_wrap.h"

namespace wivrnnx::vrctx
{

// Marks the context usable / unusable. set_live(true) belongs immediately after
// a successful VR_INIT_SERVER_DRIVER_CONTEXT, set_live(false) immediately
// before VR_CLEANUP_SERVER_DRIVER_CONTEXT.
void set_live(bool live) noexcept;
bool live() noexcept;

// All of these return nullptr when the context is down or the interface was not
// offered by this SteamVR build.
vr::IVRServerDriverHost * host() noexcept;
vr::IVRProperties * properties_raw() noexcept;
vr::CVRPropertyHelpers * properties() noexcept;
vr::IVRDriverInput * input() noexcept;
vr::IVRDriverLog * driver_log() noexcept;
vr::IVRSettings * settings() noexcept;

// Which of the interfaces the driver actually uses resolved to null, as a
// human-readable list ("host,input" / "none"). Used by the breadcrumb log at
// Init time: an interface-version mismatch shows up here first.
const char * missing_interfaces() noexcept;

} // namespace wivrnnx::vrctx
