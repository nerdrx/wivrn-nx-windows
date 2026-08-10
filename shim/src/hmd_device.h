// The headset the shim presents to SteamVR: a fictional HMD whose geometry
// comes from HmdConfig and whose poses come from PoseUpdate.
#pragma once

#include "direct_mode.h"
#include "display_component.h"
#include "openvr_driver_wrap.h"
#include <wivrnnx_ipc.h>

#include <atomic>
#include <chrono>
#include <mutex>

namespace wivrnnx
{

// Single inheritance on purpose: GetPose returns a struct in memory and has to
// be emitted with the MSVC this/sret register order (msvc_abi.h), which only
// works when no this-adjusting thunk is in the way. The display component is a
// separate object for the same reason.
class HmdDevice final : public vr::ITrackedDeviceServerDriver
{
public:
	static constexpr const char * kSerialNumber = "WiVRnNX-HMD";
	static constexpr const char * kModelNumber = "WiVRn NX";

	HmdDevice() noexcept;
	// The OpenVR interfaces have no virtual destructor, so neither can this.
	~HmdDevice() = default;

	ipc::HmdConfig config_copy() const noexcept;

	// The video path lives in the direct mode component but is driven by the
	// IPC client, which the provider owns; this is how the two are introduced.
	DirectModeComponent & direct_mode() noexcept
	{
		return direct_mode_;
	}

	// --- driven by the IPC thread ---------------------------------------

	// Stores the config and, on the first call, asks for registration. The
	// registration itself is performed by the provider from RunFrame, i.e. on
	// vrserver's own thread.
	void set_config(const ipc::HmdConfig & config) noexcept;
	void on_pose_update(const ipc::PoseUpdate & pose) noexcept;
	void on_helper_disconnected() noexcept;

	// --- driven by the provider on vrserver's thread ---------------------

	// True exactly once, on the first RunFrame after a config arrived: the
	// caller must respond by calling TrackedDeviceAdded.
	bool consume_registration_request() noexcept;
	// Called when TrackedDeviceAdded failed, so the next frame tries again.
	void registration_failed() noexcept;
	// Pushes whatever set_config left pending: the eye-to-head transforms and
	// raw projection, and the runtime property batch. No-op when nothing is
	// pending or the device is not activated yet. This is the only place the
	// config reaches vrserver after activation -- IVRServerDriverHost and
	// CVRPropertyHelpers are not documented thread-safe, so both writes must
	// happen here, on vrserver's RunFrame thread, never on the IPC thread.
	void push_pending_geometry() noexcept;

	// --- driven by the idle-pose thread ---------------------------------

	// Submits the fallback pose if no helper pose arrived recently. No-op
	// until the device has been activated.
	void tick_idle() noexcept;

	bool activated() const noexcept
	{
		return activated_.load(std::memory_order_acquire);
	}

	// --- vr::ITrackedDeviceServerDriver ----------------------------------

	vr::EVRInitError Activate(vr::TrackedDeviceIndex_t object_id) override;
	void Deactivate() override;
	void EnterStandby() override;
	void * GetComponent(const char * component_name_and_version) override;
	void DebugRequest(const char * request, char * response_buffer, uint32_t response_buffer_size) override;
	vr::DriverPose_t GetPose() override;

	// Real body behind GetPose; see msvc_abi.h for why it is split out.
	void copy_pose_into(vr::DriverPose_t * out) noexcept;

private:
	vr::EVRInitError activate_impl(vr::TrackedDeviceIndex_t object_id);
	void apply_runtime_properties(const ipc::HmdConfig & config) noexcept;
	void apply_display_geometry(const ipc::HmdConfig & config) noexcept;
	void submit_pose(const vr::DriverPose_t & pose) noexcept;

	DisplayComponent display_{this};
	// Separate object for the same reason as display_ (msvc_abi.h rule 1), and
	// the thing that stops vrcompositor looking for a desktop output.
	DirectModeComponent direct_mode_{this};

	mutable std::mutex config_mutex_;
	ipc::HmdConfig config_{};

	mutable std::mutex pose_mutex_;
	vr::DriverPose_t last_pose_{};

	std::atomic<vr::TrackedDeviceIndex_t> object_id_{vr::k_unTrackedDeviceIndexInvalid};
	std::atomic<uint64_t> prop_container_{vr::k_ulInvalidPropertyContainer};
	// Set when a config arrives, cleared by the provider once it has actually
	// registered the device with vrserver.
	std::atomic<bool> registration_wanted_{false};
	std::atomic<bool> registered_{false};
	std::atomic<bool> activated_{false};
	std::atomic<bool> first_pose_logged_{false};
	// Set by Activate() so the provider re-pushes the eye geometry: some
	// SteamVR builds activate asynchronously, in which case set_config() ran
	// before we had an object id to send it with.
	std::atomic<bool> geometry_dirty_{false};
	// Set by set_config() on a config that arrives after activation (helper
	// reconnect, runtime mode change) so the scalar properties are rewritten
	// from RunFrame instead of the IPC thread.
	std::atomic<bool> properties_dirty_{false};

	// steady_clock ticks of the last pose received from the helper; used by
	// tick_idle() to decide when to fall back to the static pose.
	std::atomic<int64_t> last_helper_pose_ticks_{0};
};

} // namespace wivrnnx
