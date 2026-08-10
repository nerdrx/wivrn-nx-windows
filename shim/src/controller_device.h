// One tracked controller. Mirrors ALVR's alvr_server/Controller.{h,cpp}:
// components are created in Activate() via vr::VRDriverInput(), inputs are
// pushed with UpdateBooleanComponent/UpdateScalarComponent, and haptics come
// back as a VREvent_Input_HapticVibration matched against our component handle
// (see DriverProvider::RunFrame, mirroring alvr_server.cpp:127-143).
//
// Unlike the HMD there is no fallback pose: a controller the helper has not
// told us about stays disconnected, because a phantom controller stuck at the
// origin is far worse than no controller at all.
#pragma once

#include "openvr_driver_wrap.h"
#include <wivrnnx_ipc.h>

#include <atomic>
#include <mutex>

namespace wivrnnx
{

class ControllerDevice final : public vr::ITrackedDeviceServerDriver
{
public:
	// Boolean components, in creation order. `primary`/`secondary` are the
	// face buttons: X/Y on the left hand, A/B on the right (matching the
	// ipc::Button comments).
	enum class BoolInput : int
	{
		system = 0,
		menu,
		primary,
		secondary,
		trigger_click,
		trigger_touch,
		grip_click,
		thumbstick_click,
		thumbstick_touch,
		count,
	};

	enum class ScalarInput : int
	{
		trigger_value = 0,
		grip_value,
		thumbstick_x,
		thumbstick_y,
		count,
	};

	explicit ControllerDevice(ipc::DeviceId device) noexcept;
	// The OpenVR interfaces have no virtual destructor, so neither can this.
	~ControllerDevice() = default;

	ControllerDevice(const ControllerDevice &) = delete;
	ControllerDevice & operator=(const ControllerDevice &) = delete;

	ipc::DeviceId device() const noexcept
	{
		return device_;
	}

	bool is_left() const noexcept
	{
		return device_ == ipc::DeviceId::left_controller;
	}

	const char * serial() const noexcept;
	const char * model() const noexcept;

	// --- driven by the IPC thread ---------------------------------------

	// Marks the controller present and, on first sight, asks for registration.
	// The TrackedDeviceAdded call itself is made by the provider from RunFrame,
	// on vrserver's own thread.
	void on_device_add() noexcept;
	// Marks the controller disconnected and invalidates its pose.
	void on_device_remove() noexcept;
	void on_pose_update(const ipc::PoseUpdate & update) noexcept;
	void on_input_update(const ipc::InputUpdate & update) noexcept;
	// The helper went away: same observable state as DeviceRemove.
	void on_helper_disconnected() noexcept;

	// --- driven by vrserver's RunFrame thread ----------------------------

	// True exactly once, on the first RunFrame after the helper announced this
	// controller: the caller must respond by calling TrackedDeviceAdded.
	bool consume_registration_request() noexcept;
	// Called when TrackedDeviceAdded failed, so the next frame tries again.
	void registration_failed() noexcept;

	// True if a VREvent_Input_HapticVibration belongs to this controller.
	bool owns_haptic(uint64_t container_handle, uint64_t component_handle) const noexcept;

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
	void create_components(vr::PropertyContainerHandle_t container) noexcept;
	void submit_pose(const vr::DriverPose_t & pose) noexcept;
	void submit_disconnected_pose() noexcept;
	void update_bool(BoolInput input, bool value, double time_offset) noexcept;
	void update_scalar(ScalarInput input, float value, double time_offset) noexcept;

	const ipc::DeviceId device_;

	mutable std::mutex pose_mutex_;
	vr::DriverPose_t last_pose_{};

	std::atomic<vr::TrackedDeviceIndex_t> object_id_{vr::k_unTrackedDeviceIndexInvalid};
	std::atomic<uint64_t> prop_container_{vr::k_ulInvalidPropertyContainer};
	std::atomic<bool> registration_wanted_{false};
	std::atomic<bool> registered_{false};
	std::atomic<bool> first_pose_logged_{false};
	std::atomic<bool> activated_{false};
	std::atomic<bool> connected_{false};

	std::atomic<uint64_t> bool_handles_[static_cast<int>(BoolInput::count)]{};
	std::atomic<uint64_t> scalar_handles_[static_cast<int>(ScalarInput::count)]{};
	std::atomic<uint64_t> haptic_handle_{vr::k_ulInvalidInputComponentHandle};
};

} // namespace wivrnnx
