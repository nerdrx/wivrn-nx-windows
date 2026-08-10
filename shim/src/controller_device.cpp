#include "controller_device.h"

#include "breadcrumb.h"
#include "driverlog.h"
#include "msvc_abi.h"
#include "vr_context.h"
#include "vr_util.h"

#include <cstring>

namespace wivrnnx
{
namespace
{

// The input profile ships inside our own driver tree; "{wivrnnx}" is the
// driver-name substitution SteamVR performs on resource paths, exactly like the
// "{htc}/input/vive_tracker_profile.json" ALVR points its fake trackers at
// (alvr_server/FakeViveTracker.cpp:136-140).
constexpr const char * kInputProfilePath = "{wivrnnx}/input/wivrnnx_controller_profile.json";
constexpr const char * kControllerType = "wivrnnx_controller";

struct BoolComponent
{
	ControllerDevice::BoolInput input;
	const char * path_left;
	const char * path_right;
};

// One creation table for both hands. The face buttons are the only asymmetric
// pair: the client reports X/Y on the left and A/B on the right (see the
// ipc::Button comments), so each hand only gets the two components it can
// actually drive, the way real Touch-style drivers do.
constexpr BoolComponent kBoolComponents[] = {
        {ControllerDevice::BoolInput::system, "/input/system/click", "/input/system/click"},
        {ControllerDevice::BoolInput::menu, "/input/application_menu/click", "/input/application_menu/click"},
        {ControllerDevice::BoolInput::primary, "/input/x/click", "/input/a/click"},
        {ControllerDevice::BoolInput::secondary, "/input/y/click", "/input/b/click"},
        {ControllerDevice::BoolInput::trigger_click, "/input/trigger/click", "/input/trigger/click"},
        {ControllerDevice::BoolInput::trigger_touch, "/input/trigger/touch", "/input/trigger/touch"},
        {ControllerDevice::BoolInput::grip_click, "/input/grip/click", "/input/grip/click"},
        {ControllerDevice::BoolInput::thumbstick_click, "/input/joystick/click", "/input/joystick/click"},
        {ControllerDevice::BoolInput::thumbstick_touch, "/input/joystick/touch", "/input/joystick/touch"},
};

struct ScalarComponent
{
	ControllerDevice::ScalarInput input;
	const char * path;
	vr::EVRScalarUnits units;
};

constexpr ScalarComponent kScalarComponents[] = {
        {ControllerDevice::ScalarInput::trigger_value, "/input/trigger/value", vr::VRScalarUnits_NormalizedOneSided},
        {ControllerDevice::ScalarInput::grip_value, "/input/grip/value", vr::VRScalarUnits_NormalizedOneSided},
        {ControllerDevice::ScalarInput::thumbstick_x, "/input/joystick/x", vr::VRScalarUnits_NormalizedTwoSided},
        {ControllerDevice::ScalarInput::thumbstick_y, "/input/joystick/y", vr::VRScalarUnits_NormalizedTwoSided},
};

constexpr const char * kHapticPath = "/output/haptic";

bool has_bit(uint32_t mask, ipc::Button button) noexcept
{
	return (mask & static_cast<uint32_t>(button)) != 0;
}

// NaN maps to 0 (neutral for both the one-sided and the two-sided ranges), not
// to the low bound: a NaN from a wedged client must not read as a hard-left
// stick deflection.
float clamp_unit(float v, float low, float high) noexcept
{
	if (v != v)
		return 0.0f;
	if (v < low)
		return low;
	if (v > high)
		return high;
	return v;
}

} // namespace

ControllerDevice::ControllerDevice(ipc::DeviceId device) noexcept :
        device_(device)
{
	last_pose_ = util::make_base_pose();
	last_pose_.poseIsValid = false;
	last_pose_.deviceIsConnected = false;
	last_pose_.result = vr::TrackingResult_Uninitialized;

	for (auto & handle: bool_handles_)
		handle.store(vr::k_ulInvalidInputComponentHandle, std::memory_order_relaxed);
	for (auto & handle: scalar_handles_)
		handle.store(vr::k_ulInvalidInputComponentHandle, std::memory_order_relaxed);
}

const char * ControllerDevice::serial() const noexcept
{
	return is_left() ? "WiVRnNX-L" : "WiVRnNX-R";
}

const char * ControllerDevice::model() const noexcept
{
	return is_left() ? "WiVRn NX Left" : "WiVRn NX Right";
}

void ControllerDevice::on_device_add() noexcept
{
	connected_.store(true, std::memory_order_release);

	if (registered_.load(std::memory_order_acquire))
	{
		// Repeat DeviceAdd, e.g. after an IPC reconnect. vrserver cannot
		// re-register a device, so all we do is un-hide it again; the next
		// PoseUpdate flips the pose back to valid.
		WNX_LOG("controller %s: re-announced by the helper", serial());
		return;
	}

	// TrackedDeviceAdded re-enters the driver (Activate, GetComponent,
	// GetPose) and must not run on the IPC thread; the provider picks this up
	// in RunFrame.
	registration_wanted_.store(true, std::memory_order_release);
	breadcrumb("controller %s: registration requested by the helper", serial());
}

bool ControllerDevice::consume_registration_request() noexcept
{
	if (registered_.load(std::memory_order_acquire))
		return false;
	if (!registration_wanted_.exchange(false, std::memory_order_acq_rel))
		return false;
	registered_.store(true, std::memory_order_release);
	return true;
}

void ControllerDevice::registration_failed() noexcept
{
	registered_.store(false, std::memory_order_release);
	registration_wanted_.store(true, std::memory_order_release);
}

void ControllerDevice::on_device_remove() noexcept
{
	if (!connected_.exchange(false, std::memory_order_acq_rel))
		return;

	WNX_LOG("controller %s: removed by the helper", serial());
	submit_disconnected_pose();
}

void ControllerDevice::on_helper_disconnected() noexcept
{
	// No fallback pose for controllers: without the helper we have nothing
	// truthful to say about where the user's hands are.
	if (!connected_.exchange(false, std::memory_order_acq_rel))
		return;

	WNX_LOG("controller %s: helper gone, reporting disconnected", serial());
	submit_disconnected_pose();
}

void ControllerDevice::submit_disconnected_pose() noexcept
{
	vr::DriverPose_t pose = util::make_base_pose();
	pose.poseIsValid = false;
	pose.deviceIsConnected = false;
	pose.result = vr::TrackingResult_Uninitialized;
	submit_pose(pose);
}

void ControllerDevice::submit_pose(const vr::DriverPose_t & pose) noexcept
{
	{
		std::lock_guard<std::mutex> lock(pose_mutex_);
		last_pose_ = pose;
	}

	// Never hand vrserver an index it did not give us, and never report before
	// Activate has run: vrserver indexes its own device array with this.
	if (!activated_.load(std::memory_order_acquire))
		return;

	const vr::TrackedDeviceIndex_t object_id = object_id_.load(std::memory_order_acquire);
	if (object_id == vr::k_unTrackedDeviceIndexInvalid)
		return;

	vr::IVRServerDriverHost * host = vrctx::host();
	if (host == nullptr)
		return;

	host->TrackedDevicePoseUpdated(object_id, pose, sizeof(vr::DriverPose_t));

	if (!first_pose_logged_.exchange(true, std::memory_order_acq_rel))
		breadcrumb("controller %s: first pose submitted for device index %u", serial(), object_id);
}

void ControllerDevice::on_pose_update(const ipc::PoseUpdate & update) noexcept
{
	if (static_cast<ipc::DeviceId>(update.device) != device_)
		return;

	// A connected pose for a controller we currently believe is gone counts as
	// an implicit DeviceAdd: the helper is the authority on what exists, and a
	// dropped DeviceAdd must not cost the user a controller for the session.
	if (update.connected != 0 && !connected_.load(std::memory_order_acquire))
		on_device_add();

	if (update.connected == 0)
	{
		submit_disconnected_pose();
		return;
	}

	// Same field-for-field copy as ALVR's Controller::OnPoseUpdate
	// (alvr_server/Controller.cpp:202-230): the helper already speaks the
	// OpenVR tracking-universe convention, so both offsets stay identity.
	vr::DriverPose_t pose = util::make_base_pose();
	pose.qRotation = util::make_quaternion(update.qw, update.qx, update.qy, update.qz);

	pose.vecPosition[0] = update.px;
	pose.vecPosition[1] = update.py;
	pose.vecPosition[2] = update.pz;

	pose.vecVelocity[0] = update.vx;
	pose.vecVelocity[1] = update.vy;
	pose.vecVelocity[2] = update.vz;

	pose.vecAngularVelocity[0] = update.wx;
	pose.vecAngularVelocity[1] = update.wy;
	pose.vecAngularVelocity[2] = update.wz;

	pose.poseTimeOffset = util::qpc_offset_seconds(update.time_qpc);

	submit_pose(pose);
}

void ControllerDevice::update_bool(BoolInput input, bool value, double time_offset) noexcept
{
	const uint64_t handle = bool_handles_[static_cast<int>(input)].load(std::memory_order_acquire);
	if (handle == vr::k_ulInvalidInputComponentHandle)
		return;

	vr::IVRDriverInput * driver_input = vrctx::input();
	if (driver_input != nullptr)
		driver_input->UpdateBooleanComponent(handle, value, time_offset);
}

void ControllerDevice::update_scalar(ScalarInput input, float value, double time_offset) noexcept
{
	const uint64_t handle = scalar_handles_[static_cast<int>(input)].load(std::memory_order_acquire);
	if (handle == vr::k_ulInvalidInputComponentHandle)
		return;

	vr::IVRDriverInput * driver_input = vrctx::input();
	if (driver_input != nullptr)
		driver_input->UpdateScalarComponent(handle, value, time_offset);
}

void ControllerDevice::on_input_update(const ipc::InputUpdate & update) noexcept
{
	if (static_cast<ipc::DeviceId>(update.device) != device_)
		return;
	if (!activated_.load(std::memory_order_acquire) || !connected_.load(std::memory_order_acquire))
		return;

	// ALVR passes a flat 0.0 for every component update
	// (alvr_server/Controller.cpp:150-156). We can do slightly better because
	// the helper timestamps each InputUpdate: reuse the pose convention
	// ("seconds from now", negative = already happened) and clamp it to the
	// past, since an input event can never be in the future. A helper that
	// leaves time_qpc at 0 degrades exactly to ALVR's behaviour.
	double time_offset = util::qpc_offset_seconds(update.time_qpc);
	if (time_offset > 0.0)
		time_offset = 0.0;

	const uint32_t pressed = update.pressed;
	const uint32_t touched = update.touched;

	update_bool(BoolInput::system, has_bit(pressed, ipc::Button::system), time_offset);
	update_bool(BoolInput::menu, has_bit(pressed, ipc::Button::menu), time_offset);

	if (is_left())
	{
		update_bool(BoolInput::primary, has_bit(pressed, ipc::Button::x), time_offset);
		update_bool(BoolInput::secondary, has_bit(pressed, ipc::Button::y), time_offset);
	}
	else
	{
		update_bool(BoolInput::primary, has_bit(pressed, ipc::Button::a), time_offset);
		update_bool(BoolInput::secondary, has_bit(pressed, ipc::Button::b), time_offset);
	}

	update_bool(BoolInput::trigger_click, has_bit(pressed, ipc::Button::trigger_click), time_offset);
	update_bool(BoolInput::grip_click, has_bit(pressed, ipc::Button::grip_click), time_offset);
	update_bool(BoolInput::thumbstick_click, has_bit(pressed, ipc::Button::thumbstick_click), time_offset);

	// The touch bits live in their own mask; accept them from `pressed` too so
	// a helper that only fills one mask still lights the touch components up.
	update_bool(BoolInput::trigger_touch,
	            has_bit(touched, ipc::Button::trigger_touch) || has_bit(pressed, ipc::Button::trigger_touch),
	            time_offset);
	update_bool(BoolInput::thumbstick_touch,
	            has_bit(touched, ipc::Button::thumbstick_touch) || has_bit(pressed, ipc::Button::thumbstick_touch),
	            time_offset);

	update_scalar(ScalarInput::trigger_value, clamp_unit(update.trigger, 0.0f, 1.0f), time_offset);
	update_scalar(ScalarInput::grip_value, clamp_unit(update.grip, 0.0f, 1.0f), time_offset);
	update_scalar(ScalarInput::thumbstick_x, clamp_unit(update.thumbstick_x, -1.0f, 1.0f), time_offset);
	update_scalar(ScalarInput::thumbstick_y, clamp_unit(update.thumbstick_y, -1.0f, 1.0f), time_offset);
}

bool ControllerDevice::owns_haptic(uint64_t container_handle, uint64_t component_handle) const noexcept
{
	const uint64_t haptic = haptic_handle_.load(std::memory_order_acquire);
	if (haptic != vr::k_ulInvalidInputComponentHandle && component_handle == haptic)
		return true;

	// ALVR matches on the property container instead (alvr_server.cpp:135-141);
	// keep that as the fallback for SteamVR builds that leave componentHandle
	// unset.
	const uint64_t container = prop_container_.load(std::memory_order_acquire);
	return container != vr::k_ulInvalidPropertyContainer && container_handle == container;
}

vr::EVRInitError ControllerDevice::Activate(vr::TrackedDeviceIndex_t object_id)
{
	// OpenVR ABI boundary: nothing may propagate out of here.
	breadcrumb("controller %s: Activate(object_id=%u) entered", serial(), object_id);
	try
	{
		const vr::EVRInitError result = activate_impl(object_id);
		breadcrumb("controller %s: Activate returned %d", serial(), static_cast<int>(result));
		return result;
	}
	catch (const std::exception & e)
	{
		WNX_LOG("controller %s: Activate threw (%s), failing activation", serial(), e.what());
		breadcrumb("controller %s: Activate threw std::exception (%s)", serial(), e.what());
		return vr::VRInitError_Driver_Failed;
	}
	catch (...)
	{
		WNX_LOG("controller %s: Activate threw, failing activation", serial());
		breadcrumb("controller %s: Activate threw a non-standard exception", serial());
		return vr::VRInitError_Driver_Failed;
	}
}

vr::EVRInitError ControllerDevice::activate_impl(vr::TrackedDeviceIndex_t object_id)
{
	vr::CVRPropertyHelpers * properties = vrctx::properties();
	if (properties == nullptr)
	{
		WNX_LOG("controller %s: Activate without a property helper", serial());
		breadcrumb("controller %s: Activate failed, IVRProperties is not available", serial());
		return vr::VRInitError_Driver_Failed;
	}

	object_id_.store(object_id, std::memory_order_release);

	const vr::PropertyContainerHandle_t container = properties->TrackedDeviceToPropertyContainer(object_id);
	prop_container_.store(container, std::memory_order_release);

	const bool left = is_left();

	properties->SetStringProperty(container, vr::Prop_TrackingSystemName_String, "wivrnnx");
	properties->SetStringProperty(container, vr::Prop_ModelNumber_String, model());
	properties->SetStringProperty(container, vr::Prop_SerialNumber_String, serial());
	properties->SetStringProperty(container, vr::Prop_ManufacturerName_String, "WiVRn");
	properties->SetStringProperty(container, vr::Prop_DriverVersion_String, "0.1.0");
	properties->SetStringProperty(container, vr::Prop_RenderModelName_String, "generic_controller");
	properties->SetStringProperty(container, vr::Prop_ControllerType_String, kControllerType);
	properties->SetStringProperty(container, vr::Prop_InputProfilePath_String, kInputProfilePath);
	properties->SetStringProperty(container,
	                              vr::Prop_RegisteredDeviceType_String,
	                              left ? "wivrnnx/WiVRnNX-L" : "wivrnnx/WiVRnNX-R");

	properties->SetInt32Property(container, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
	properties->SetInt32Property(container,
	                             vr::Prop_ControllerRoleHint_Int32,
	                             left ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand);

	// Legacy (pre-input-system) axis description; harmless for modern apps but
	// it keeps the old IVRSystem controller API from reporting a blank device.
	properties->SetInt32Property(container, vr::Prop_Axis0Type_Int32, vr::k_eControllerAxis_Joystick);
	properties->SetInt32Property(container, vr::Prop_Axis1Type_Int32, vr::k_eControllerAxis_Trigger);
	properties->SetUint64Property(container,
	                              vr::Prop_SupportedButtons_Uint64,
	                              vr::ButtonMaskFromId(vr::k_EButton_System) |
	                                      vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu) |
	                                      vr::ButtonMaskFromId(vr::k_EButton_Grip) |
	                                      vr::ButtonMaskFromId(vr::k_EButton_A) |
	                                      vr::ButtonMaskFromId(vr::k_EButton_Axis0) |
	                                      vr::ButtonMaskFromId(vr::k_EButton_Axis1));

	properties->SetBoolProperty(container, vr::Prop_DeviceIsWireless_Bool, true);
	properties->SetBoolProperty(container, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
	properties->SetBoolProperty(container, vr::Prop_Identifiable_Bool, false);
	properties->SetBoolProperty(container, vr::Prop_DeviceCanPowerOff_Bool, false);
	properties->SetBoolProperty(container, vr::Prop_Firmware_RemindUpdate_Bool, false);

	properties->SetUint64Property(container, vr::Prop_CurrentUniverseId_Uint64, 2);

	create_components(container);

	activated_.store(true, std::memory_order_release);
	WNX_LOG("controller %s: activated as device index %u (%s hand)", serial(), object_id, left ? "left" : "right");

	// Publish the current (still disconnected until the first pose) state so
	// vrserver has something for this index straight away.
	{
		vr::DriverPose_t pose;
		{
			std::lock_guard<std::mutex> lock(pose_mutex_);
			pose = last_pose_;
		}
		if (vr::IVRServerDriverHost * host = vrctx::host(); host != nullptr)
			host->TrackedDevicePoseUpdated(object_id, pose, sizeof(vr::DriverPose_t));
	}

	return vr::VRInitError_None;
}

void ControllerDevice::create_components(vr::PropertyContainerHandle_t container) noexcept
{
	vr::IVRDriverInput * driver_input = vrctx::input();
	if (driver_input == nullptr)
	{
		WNX_LOG("controller %s: no IVRDriverInput, running without input", serial());
		breadcrumb("controller %s: no IVRDriverInput, running without input components", serial());
		return;
	}

	const bool left = is_left();

	for (const BoolComponent & component: kBoolComponents)
	{
		vr::VRInputComponentHandle_t handle = vr::k_ulInvalidInputComponentHandle;
		const char * path = left ? component.path_left : component.path_right;
		const vr::EVRInputError error = driver_input->CreateBooleanComponent(container, path, &handle);
		if (error != vr::VRInputError_None)
		{
			WNX_LOG("controller %s: CreateBooleanComponent(%s) failed (%d)", serial(), path, static_cast<int>(error));
			continue;
		}
		bool_handles_[static_cast<int>(component.input)].store(handle, std::memory_order_release);
	}

	for (const ScalarComponent & component: kScalarComponents)
	{
		vr::VRInputComponentHandle_t handle = vr::k_ulInvalidInputComponentHandle;
		const vr::EVRInputError error = driver_input->CreateScalarComponent(
		        container, component.path, &handle, vr::VRScalarType_Absolute, component.units);
		if (error != vr::VRInputError_None)
		{
			WNX_LOG("controller %s: CreateScalarComponent(%s) failed (%d)",
			        serial(),
			        component.path,
			        static_cast<int>(error));
			continue;
		}
		scalar_handles_[static_cast<int>(component.input)].store(handle, std::memory_order_release);
	}

	// Mirrors Controller::activate (alvr_server/Controller.cpp:30).
	vr::VRInputComponentHandle_t haptic = vr::k_ulInvalidInputComponentHandle;
	const vr::EVRInputError error = driver_input->CreateHapticComponent(container, kHapticPath, &haptic);
	if (error != vr::VRInputError_None)
		WNX_LOG("controller %s: CreateHapticComponent failed (%d)", serial(), static_cast<int>(error));
	else
		haptic_handle_.store(haptic, std::memory_order_release);
}

void ControllerDevice::Deactivate()
{
	WNX_LOG("controller %s: deactivated", serial());
	breadcrumb("controller %s: Deactivate", serial());
	activated_.store(false, std::memory_order_release);
	object_id_.store(vr::k_unTrackedDeviceIndexInvalid, std::memory_order_release);
	prop_container_.store(vr::k_ulInvalidPropertyContainer, std::memory_order_release);
	haptic_handle_.store(vr::k_ulInvalidInputComponentHandle, std::memory_order_release);
	for (auto & handle: bool_handles_)
		handle.store(vr::k_ulInvalidInputComponentHandle, std::memory_order_release);
	for (auto & handle: scalar_handles_)
		handle.store(vr::k_ulInvalidInputComponentHandle, std::memory_order_release);
}

void ControllerDevice::EnterStandby()
{
	WNX_LOG("controller %s: entering standby", serial());
}

void ControllerDevice::copy_pose_into(vr::DriverPose_t * out) noexcept
{
	if (out == nullptr)
		return;
	try
	{
		std::lock_guard<std::mutex> lock(pose_mutex_);
		*out = last_pose_;
	}
	catch (...)
	{
		*out = util::make_base_pose();
		out->poseIsValid = false;
		out->deviceIsConnected = false;
		out->result = vr::TrackingResult_Uninitialized;
	}
}

void * ControllerDevice::GetComponent(const char * component_name_and_version)
{
	// No display, camera or direct-mode components on a controller, but log
	// what SteamVR asked for: it tells us what this build expects.
	breadcrumb("controller %s: GetComponent(\"%s\") -> null",
	           serial(),
	           component_name_and_version != nullptr ? component_name_and_version : "(null)");
	return nullptr;
}

void ControllerDevice::DebugRequest(const char *, char * response_buffer, uint32_t response_buffer_size)
{
	if (response_buffer != nullptr && response_buffer_size >= 1)
		response_buffer[0] = '\0';
}

extern "C" vr::DriverPose_t * wnx_controller_get_pose(ControllerDevice * self, vr::DriverPose_t * out) noexcept
{
	if (out == nullptr)
		return out;
	if (self == nullptr)
	{
		*out = util::make_base_pose();
		out->poseIsValid = false;
		out->deviceIsConnected = false;
		out->result = vr::TrackingResult_Uninitialized;
		return out;
	}
	self->copy_pose_into(out);
	return out;
}

#if WNX_MSVC_SRET_FIXUP
WNX_SRET_THUNK(vr::DriverPose_t, ControllerDevice::GetPose(), wnx_controller_get_pose)
#else
vr::DriverPose_t ControllerDevice::GetPose()
{
	vr::DriverPose_t pose{};
	wnx_controller_get_pose(this, &pose);
	return pose;
}
#endif

} // namespace wivrnnx
