#include "hmd_device.h"

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

// Fallback pose used whenever the helper is not feeding us: standing height,
// facing forward. SteamVR complains loudly about an HMD that never reports.
constexpr double kIdleHeightMeters = 1.6;

// A helper pose older than this means we are effectively untracked.
constexpr auto kHelperPoseTimeout = std::chrono::milliseconds(100);

ipc::HmdConfig default_config() noexcept
{
	ipc::HmdConfig config{};
	config.eye_width = 1920;
	config.eye_height = 1920;
	config.refresh_hz = 90.0f;
	config.ipd_m = 0.063f;
	for (int eye = 0; eye < 2; ++eye)
	{
		config.proj_left[eye] = -1.0f;
		config.proj_right[eye] = 1.0f;
		config.proj_top[eye] = -1.0f;
		config.proj_bottom[eye] = 1.0f;

		// v2: a plain non-canted headset — identity rotation, eyes at
		// (∓ipd/2, 0, 0). Overwritten by the first real HmdConfig.
		config.eye_to_head_q[eye][0] = 1.0f;
		config.eye_to_head_q[eye][1] = 0.0f;
		config.eye_to_head_q[eye][2] = 0.0f;
		config.eye_to_head_q[eye][3] = 0.0f;
		config.eye_to_head_p[eye][0] = (eye == 0 ? -0.5f : 0.5f) * config.ipd_m;
		config.eye_to_head_p[eye][1] = 0.0f;
		config.eye_to_head_p[eye][2] = 0.0f;
	}
	return config;
}

using util::make_base_pose;
using util::make_quaternion;
using util::steady_ticks;

} // namespace

HmdDevice::HmdDevice() noexcept
{
	config_ = default_config();
	last_pose_ = make_base_pose();
	last_pose_.vecPosition[1] = kIdleHeightMeters;
}

ipc::HmdConfig HmdDevice::config_copy() const noexcept
{
	std::lock_guard<std::mutex> lock(config_mutex_);
	return config_;
}

void HmdDevice::set_config(const ipc::HmdConfig & config) noexcept
{
	{
		std::lock_guard<std::mutex> lock(config_mutex_);
		config_ = config;
	}

	WNX_LOG("HMD config: %ux%u per eye, %.2f Hz, ipd %.4f m",
	        config.eye_width,
	        config.eye_height,
	        static_cast<double>(config.refresh_hz),
	        static_cast<double>(config.ipd_m));
	breadcrumb("HMD: config %ux%u per eye, %.2f Hz, ipd %.4f m",
	           config.eye_width,
	           config.eye_height,
	           static_cast<double>(config.refresh_hz),
	           static_cast<double>(config.ipd_m));

	if (registered_.load(std::memory_order_acquire))
	{
		// Already live. The render-target size cannot be renegotiated yet, but
		// the eye geometry and the cheap scalar properties can be refreshed.
		// Neither is written from here: this runs on the IPC thread, and the
		// host interfaces are only touched from RunFrame (push_pending_geometry).
		properties_dirty_.store(true, std::memory_order_release);
		geometry_dirty_.store(true, std::memory_order_release);
		return;
	}

	// TrackedDeviceAdded is not called from here. It happens in RunFrame, on
	// vrserver's own thread: registering a device re-enters the driver
	// (Activate, GetComponent, GetPose) and doing that from the IPC thread
	// means vrserver runs its device bring-up concurrently with whatever its
	// main thread is doing.
	registration_wanted_.store(true, std::memory_order_release);
}

bool HmdDevice::consume_registration_request() noexcept
{
	if (registered_.load(std::memory_order_acquire))
		return false;
	if (!registration_wanted_.exchange(false, std::memory_order_acq_rel))
		return false;
	registered_.store(true, std::memory_order_release);
	return true;
}

void HmdDevice::registration_failed() noexcept
{
	registered_.store(false, std::memory_order_release);
	registration_wanted_.store(true, std::memory_order_release);
}

void HmdDevice::push_pending_geometry() noexcept
{
	if (!activated_.load(std::memory_order_acquire))
		return;

	const bool geometry = geometry_dirty_.exchange(false, std::memory_order_acq_rel);
	const bool properties = properties_dirty_.exchange(false, std::memory_order_acq_rel);
	if (!geometry && !properties)
		return;

	const ipc::HmdConfig config = config_copy();
	if (properties)
		apply_runtime_properties(config);
	if (geometry)
		apply_display_geometry(config);
}

void HmdDevice::apply_display_geometry(const ipc::HmdConfig & config) noexcept
{
	const vr::TrackedDeviceIndex_t object_id = object_id_.load(std::memory_order_acquire);
	if (object_id == vr::k_unTrackedDeviceIndexInvalid)
		return;

	vr::IVRServerDriverHost * host = vrctx::host();
	if (host == nullptr)
	{
		breadcrumb("HMD: cannot push display geometry, no IVRServerDriverHost");
		return;
	}

	// ALVR does exactly this in Hmd::SetViewParams (alvr_server/HMD.cpp:277-295):
	// the eye-to-head transforms and the raw projection go to
	// IVRServerDriverHost, *not* through IVRDisplayComponent (which only gets
	// asked for GetProjectionRaw), and a LensDistortionChanged vendor event
	// nudges the compositor into picking the new geometry up.
	const vr::HmdMatrix34_t left_eye = util::make_matrix34(config.eye_to_head_q[0], config.eye_to_head_p[0]);
	const vr::HmdMatrix34_t right_eye = util::make_matrix34(config.eye_to_head_q[1], config.eye_to_head_p[1]);
	host->SetDisplayEyeToHead(object_id, left_eye, right_eye);

	vr::HmdRect2_t left_proj{};
	left_proj.vTopLeft.v[0] = config.proj_left[0];
	left_proj.vTopLeft.v[1] = config.proj_top[0];
	left_proj.vBottomRight.v[0] = config.proj_right[0];
	left_proj.vBottomRight.v[1] = config.proj_bottom[0];

	vr::HmdRect2_t right_proj{};
	right_proj.vTopLeft.v[0] = config.proj_left[1];
	right_proj.vTopLeft.v[1] = config.proj_top[1];
	right_proj.vBottomRight.v[0] = config.proj_right[1];
	right_proj.vBottomRight.v[1] = config.proj_bottom[1];

	host->SetDisplayProjectionRaw(object_id, left_proj, right_proj);
	host->VendorSpecificEvent(object_id, vr::VREvent_LensDistortionChanged, {}, 0);

	WNX_LOG("HMD: eye-to-head left (%.4f %.4f %.4f) right (%.4f %.4f %.4f)",
	        static_cast<double>(config.eye_to_head_p[0][0]),
	        static_cast<double>(config.eye_to_head_p[0][1]),
	        static_cast<double>(config.eye_to_head_p[0][2]),
	        static_cast<double>(config.eye_to_head_p[1][0]),
	        static_cast<double>(config.eye_to_head_p[1][1]),
	        static_cast<double>(config.eye_to_head_p[1][2]));
	breadcrumb("HMD: pushed display geometry for device index %u", object_id);
}

void HmdDevice::apply_runtime_properties(const ipc::HmdConfig & config) noexcept
{
	const auto container = static_cast<vr::PropertyContainerHandle_t>(prop_container_.load(std::memory_order_acquire));
	if (container == vr::k_ulInvalidPropertyContainer)
		return;

	vr::CVRPropertyHelpers * properties = vrctx::properties();
	if (properties == nullptr)
		return;

	properties->SetFloatProperty(container, vr::Prop_DisplayFrequency_Float, config.refresh_hz);
	properties->SetFloatProperty(container, vr::Prop_UserIpdMeters_Float, config.ipd_m);
	properties->SetFloatProperty(container,
	                             vr::Prop_SecondsFromVsyncToPhotons_Float,
	                             config.refresh_hz > 0.0f ? 1.0f / config.refresh_hz : 0.011f);
	breadcrumb("HMD: refreshed runtime properties (%.2f Hz, ipd %.4f m)",
	           static_cast<double>(config.refresh_hz),
	           static_cast<double>(config.ipd_m));
}

vr::EVRInitError HmdDevice::Activate(vr::TrackedDeviceIndex_t object_id)
{
	// Activate is an OpenVR ABI boundary: nothing may propagate out of it.
	breadcrumb("HMD: Activate(object_id=%u) entered", object_id);
	try
	{
		const vr::EVRInitError result = activate_impl(object_id);
		breadcrumb("HMD: Activate returned %d", static_cast<int>(result));
		return result;
	}
	catch (const std::exception & e)
	{
		WNX_LOG("HMD: Activate threw (%s), failing activation", e.what());
		breadcrumb("HMD: Activate threw std::exception (%s)", e.what());
		return vr::VRInitError_Driver_Failed;
	}
	catch (...)
	{
		WNX_LOG("HMD: Activate threw, failing activation");
		breadcrumb("HMD: Activate threw a non-standard exception");
		return vr::VRInitError_Driver_Failed;
	}
}

vr::EVRInitError HmdDevice::activate_impl(vr::TrackedDeviceIndex_t object_id)
{
	vr::CVRPropertyHelpers * properties = vrctx::properties();
	if (properties == nullptr)
	{
		WNX_LOG("HMD: Activate without a property helper");
		breadcrumb("HMD: Activate failed, IVRProperties is not available");
		return vr::VRInitError_Driver_Failed;
	}

	object_id_.store(object_id, std::memory_order_release);

	const vr::PropertyContainerHandle_t container = properties->TrackedDeviceToPropertyContainer(object_id);
	prop_container_.store(container, std::memory_order_release);

	const ipc::HmdConfig config = config_copy();

	// Every string below is a literal in the DLL's .rdata: vrserver may keep
	// the pointer, so nothing here may be a temporary.
	properties->SetStringProperty(container, vr::Prop_ModelNumber_String, kModelNumber);
	properties->SetStringProperty(container, vr::Prop_SerialNumber_String, kSerialNumber);
	properties->SetStringProperty(container, vr::Prop_ManufacturerName_String, "WiVRn");
	properties->SetStringProperty(container, vr::Prop_TrackingSystemName_String, "wivrnnx");
	properties->SetStringProperty(container, vr::Prop_DriverVersion_String, "0.1.0");
	properties->SetStringProperty(container, vr::Prop_RenderModelName_String, "generic_hmd");
	properties->SetStringProperty(container, vr::Prop_ControllerType_String, "wivrnnx_hmd");

	properties->SetFloatProperty(container, vr::Prop_DisplayFrequency_Float, config.refresh_hz);
	properties->SetFloatProperty(container, vr::Prop_UserIpdMeters_Float, config.ipd_m);
	properties->SetFloatProperty(container, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.0f);
	properties->SetFloatProperty(container,
	                             vr::Prop_SecondsFromVsyncToPhotons_Float,
	                             config.refresh_hz > 0.0f ? 1.0f / config.refresh_hz : 0.011f);

	// Universe 2 is what every streaming driver uses so the standing-play space
	// lines up with whatever the room setup produced.
	properties->SetUint64Property(container, vr::Prop_CurrentUniverseId_Uint64, 2);

	properties->SetBoolProperty(container, vr::Prop_IsOnDesktop_Bool, false);
	properties->SetBoolProperty(container, vr::Prop_DisplayDebugMode_Bool, false);
	properties->SetBoolProperty(container, vr::Prop_DeviceIsWireless_Bool, true);
	properties->SetBoolProperty(container, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
	properties->SetBoolProperty(container, vr::Prop_ContainsProximitySensor_Bool, false);
	properties->SetBoolProperty(container, vr::Prop_DriverIsDrawingControllers_Bool, false);
	// Direct mode. GetComponent answering IVRDriverDirectModeComponent_009 is
	// what actually switches the compositor over; this property is the
	// advertisement that goes with it, and until Phase 2 it was explicitly
	// false, which was worse than not setting it at all.
	//
	// ALVR is the reference for the whole set and it is a short one: it never
	// sets this property true on the HMD (it leaves it at its default and lets
	// GetComponent speak for itself), and only ever sets it *false*, on the
	// devices that must not be mistaken for a display -- FakeViveTracker.cpp:197
	// and ViveTrackerProxy.cpp:172. Setting it true here is the honest value
	// now that the component exists.
	properties->SetBoolProperty(container, vr::Prop_HasDriverDirectModeComponent_Bool, true);
	// Deliberately NOT set, both mirroring ALVR:
	//   Prop_DriverDirectModeSendsVsyncEvents_Bool -- ALVR sends VsyncEvent from
	//     its own loop (alvr_server.cpp:362) without declaring it, so vrserver
	//     keeps its own estimator as a backstop. Declaring it would make the
	//     compositor depend entirely on our thread, and a stub is the wrong
	//     place to remove a safety net.
	//   Prop_Hmd_SupportsAppThrottling_Bool -- enables the throttling/prediction
	//     UI and makes PostPresent's Throttling_t meaningful; nothing here acts
	//     on it yet.

	properties->SetInt32Property(container, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_HMD);
	properties->SetInt32Property(container, vr::Prop_ExpectedTrackingReferenceCount_Int32, 0);

	activated_.store(true, std::memory_order_release);
	geometry_dirty_.store(true, std::memory_order_release);
	WNX_LOG("HMD: activated as device index %u", object_id);

	// Publish the fallback pose immediately so SteamVR never sees an HMD that
	// has not reported at all.
	{
		std::lock_guard<std::mutex> lock(pose_mutex_);
		last_pose_ = make_base_pose();
		last_pose_.vecPosition[1] = kIdleHeightMeters;
		if (vr::IVRServerDriverHost * host = vrctx::host(); host != nullptr)
			host->TrackedDevicePoseUpdated(object_id, last_pose_, sizeof(vr::DriverPose_t));
	}

	return vr::VRInitError_None;
}

void HmdDevice::Deactivate()
{
	try
	{
		WNX_LOG("HMD: deactivated");
		breadcrumb("HMD: Deactivate");
		activated_.store(false, std::memory_order_release);
		object_id_.store(vr::k_unTrackedDeviceIndexInvalid, std::memory_order_release);
		prop_container_.store(vr::k_ulInvalidPropertyContainer, std::memory_order_release);
	}
	catch (...)
	{
	}
}

void HmdDevice::EnterStandby()
{
	try
	{
		WNX_LOG("HMD: entering standby");
		breadcrumb("HMD: EnterStandby");
	}
	catch (...)
	{
	}
}

void * HmdDevice::GetComponent(const char * component_name_and_version)
{
	try
	{
		// The requested name is the single most useful thing the breadcrumb log
		// can capture: it says exactly which components this SteamVR build
		// expects a headset driver to provide.
		breadcrumb("HMD: GetComponent(\"%s\")",
		           component_name_and_version != nullptr ? component_name_and_version : "(null)");

		if (component_name_and_version == nullptr)
			return nullptr;

		if (std::strcmp(component_name_and_version, vr::IVRDisplayComponent_Version) == 0)
			return static_cast<vr::IVRDisplayComponent *>(&display_);

		// vrserver asks for this by exact name ("IVRDriverDirectModeComponent_009"
		// on SteamVR 2.16.7, breadcrumb-confirmed). Answering it is what keeps
		// vrcompositor from treating the headset as an extended desktop display
		// and dying on "Invalid adapter output specified! (0) 887A0022".
		if (std::strcmp(component_name_and_version, vr::IVRDriverDirectModeComponent_Version) == 0)
		{
			breadcrumb("HMD: handing out the direct mode component (%s)",
			           vr::IVRDriverDirectModeComponent_Version);
			return static_cast<vr::IVRDriverDirectModeComponent *>(&direct_mode_);
		}
	}
	catch (...)
	{
	}
	return nullptr;
}

void HmdDevice::DebugRequest(const char *, char * response_buffer, uint32_t response_buffer_size)
{
	if (response_buffer != nullptr && response_buffer_size >= 1)
		response_buffer[0] = '\0';
}

void HmdDevice::copy_pose_into(vr::DriverPose_t * out) noexcept
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
		out->result = vr::TrackingResult_Uninitialized;
	}
}

extern "C" vr::DriverPose_t * wnx_hmd_get_pose(HmdDevice * self, vr::DriverPose_t * out) noexcept
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
WNX_SRET_THUNK(vr::DriverPose_t, HmdDevice::GetPose(), wnx_hmd_get_pose)
#else
vr::DriverPose_t HmdDevice::GetPose()
{
	vr::DriverPose_t pose{};
	wnx_hmd_get_pose(this, &pose);
	return pose;
}
#endif

void HmdDevice::submit_pose(const vr::DriverPose_t & pose) noexcept
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
		breadcrumb("HMD: first pose submitted for device index %u", object_id);
}

void HmdDevice::on_pose_update(const ipc::PoseUpdate & update) noexcept
{
	if (static_cast<ipc::DeviceId>(update.device) != ipc::DeviceId::hmd)
		return;

	last_helper_pose_ticks_.store(steady_ticks(), std::memory_order_release);

	vr::DriverPose_t pose = make_base_pose();

	if (update.connected == 0)
	{
		pose.result = vr::TrackingResult_Running_OutOfRange;
		pose.poseIsValid = false;
		pose.deviceIsConnected = false;
		submit_pose(pose);
		return;
	}

	// The helper already speaks the OpenVR tracking-universe convention, so the
	// world/head offsets stay identity and the fields copy across verbatim.
	pose.qRotation = make_quaternion(update.qw, update.qx, update.qy, update.qz);

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

void HmdDevice::on_helper_disconnected() noexcept
{
	// Nothing to tear down: tick_idle() takes over as soon as the last helper
	// pose goes stale.
	last_helper_pose_ticks_.store(0, std::memory_order_release);
}

void HmdDevice::tick_idle() noexcept
{
	if (!activated_.load(std::memory_order_acquire))
		return;

	const int64_t last = last_helper_pose_ticks_.load(std::memory_order_acquire);
	if (last != 0)
	{
		const auto age = std::chrono::steady_clock::duration(steady_ticks() - last);
		if (age < kHelperPoseTimeout)
			return;
	}

	vr::DriverPose_t pose = make_base_pose();
	pose.vecPosition[1] = kIdleHeightMeters;
	submit_pose(pose);
}

} // namespace wivrnnx
