// IServerTrackedDeviceProvider implementation and the driver DLL entry point.
//
// Nothing here may throw across the OpenVR ABI boundary, so every override
// either is noexcept-by-construction or wraps its body in a catch-all, and
// every step leaves a breadcrumb: if vrserver dies the file is the only
// evidence that survives.

#include "breadcrumb.h"
#include "controller_device.h"
#include "driverlog.h"
#include "hmd_device.h"
#include "ipc_client.h"
#include "vr_context.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "openvr_driver_wrap.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

namespace wivrnnx
{
namespace
{

// Cadence of the fallback pose thread. Only actually submits a pose when the
// helper has gone quiet, see HmdDevice::tick_idle().
constexpr auto kIdlePosePeriod = std::chrono::milliseconds(20); // 50 Hz
// How long Cleanup waits for the fallback thread before saying so in the log.
constexpr auto kJoinWarnAfter = std::chrono::milliseconds(500);

class DriverProvider final : public vr::IServerTrackedDeviceProvider
{
public:
	vr::EVRInitError Init(vr::IVRDriverContext * context) override
	{
		breadcrumb("provider: Init entered (context %p)", static_cast<void *>(context));
		try
		{
			const vr::EVRInitError result = init_impl(context);
			breadcrumb("provider: Init returning %d", static_cast<int>(result));
			return result;
		}
		catch (const std::exception & e)
		{
			breadcrumb("provider: Init threw std::exception (%s)", e.what());
			return vr::VRInitError_Driver_Failed;
		}
		catch (...)
		{
			breadcrumb("provider: Init threw a non-standard exception");
			return vr::VRInitError_Driver_Failed;
		}
	}

	void Cleanup() override
	{
		breadcrumb("provider: Cleanup entered");
		try
		{
			cleanup_impl();
		}
		catch (const std::exception & e)
		{
			breadcrumb("provider: Cleanup threw std::exception (%s)", e.what());
		}
		catch (...)
		{
			breadcrumb("provider: Cleanup threw a non-standard exception");
		}
		breadcrumb("provider: Cleanup done");
	}

	const char * const * GetInterfaceVersions() override
	{
		// The header's table, null-terminated, exactly like ALVR's
		// DriverProvider::GetInterfaceVersions. vrserver walks it until the
		// null, so it must never be a local or a partially-filled array.
		return vr::k_InterfaceVersions;
	}

	// Called on vrserver's main thread, which makes it the only safe place to
	// register devices: TrackedDeviceAdded synchronously re-enters the driver
	// (Activate, GetComponent, GetPose) and doing that from the IPC thread runs
	// vrserver's device bring-up concurrently with its own frame loop.
	void RunFrame() override
	{
		try
		{
			run_frame_impl();
		}
		catch (...)
		{
			// A throwing RunFrame would unwind straight into vrserver.
		}
	}

	bool ShouldBlockStandbyMode() override
	{
		return false;
	}

	void EnterStandby() override
	{
		WNX_LOG("provider: EnterStandby");
		breadcrumb("provider: EnterStandby");
	}

	void LeaveStandby() override
	{
		WNX_LOG("provider: LeaveStandby");
		breadcrumb("provider: LeaveStandby");
	}

private:
	vr::EVRInitError init_impl(vr::IVRDriverContext * context)
	{
		if (context == nullptr)
		{
			breadcrumb("provider: Init got a null IVRDriverContext, refusing to load");
			return vr::VRInitError_Init_InvalidInterface;
		}

		// Deliberately not VR_INIT_SERVER_DRIVER_CONTEXT: that macro returns
		// immediately if *any* of the six interfaces InitServer() checks is
		// missing, including IVRDriverManager and IVRResources, which this
		// driver never touches. On a SteamVR whose interface versions have
		// moved on that would turn a cosmetic mismatch into "driver refuses to
		// load", with nothing in any log to say which interface was to blame.
		const vr::EVRInitError context_error = vr::InitServerDriverContext(context);
		vrctx::set_live(true);

		// The single most diagnostic line in the file: exactly which of the
		// interfaces resolved to null on this SteamVR build.
		breadcrumb("provider: driver context init returned %d, interfaces resolving to null: %s",
		           static_cast<int>(context_error),
		           vrctx::missing_interfaces());

		if (context_error != vr::VRInitError_None)
		{
			// Only the interfaces the driver actually uses are load-bearing.
			if (vrctx::host() == nullptr || vrctx::properties_raw() == nullptr)
			{
				breadcrumb("provider: IVRServerDriverHost and/or IVRProperties are missing, "
				           "this SteamVR cannot drive this build - refusing to load");
				vrctx::set_live(false);
				vr::CleanupDriverContext();
				return context_error;
			}
			breadcrumb("provider: continuing anyway, the missing interfaces are ones this "
			           "driver does not use");
		}

		init_driver_log(vrctx::driver_log());

		WNX_LOG("provider: Init (SteamVR %u.%u.%u headers)",
		        vr::k_nSteamVRVersionMajor,
		        vr::k_nSteamVRVersionMinor,
		        vr::k_nSteamVRVersionBuild);
		breadcrumb("provider: context is live, built against SteamVR %u.%u.%u headers",
		           vr::k_nSteamVRVersionMajor,
		           vr::k_nSteamVRVersionMinor,
		           vr::k_nSteamVRVersionBuild);

		if (vrctx::host() == nullptr)
		{
			// Nothing we can usefully do without a host, but staying loaded and
			// inert is far better than taking vrserver with us.
			breadcrumb("provider: no IVRServerDriverHost, loading inert (no devices, no threads)");
			WNX_LOG("provider: no IVRServerDriverHost, driver will stay inert");
			return vr::VRInitError_None;
		}

		try
		{
			hmd_ = std::make_unique<HmdDevice>();
			// Both controllers exist from the start but stay unregistered
			// until the helper announces them with a DeviceAdd.
			left_ = std::make_unique<ControllerDevice>(ipc::DeviceId::left_controller);
			right_ = std::make_unique<ControllerDevice>(ipc::DeviceId::right_controller);
		}
		catch (...)
		{
			WNX_LOG("provider: failed to allocate the tracked devices");
			breadcrumb("provider: failed to allocate the tracked devices");
			return vr::VRInitError_Driver_Failed;
		}

		// Introduce the video path to the pipe before the client thread that
		// drives it exists. Ordering matters both ways: cleanup_impl stops the
		// IPC client before releasing hmd_, so the pointer cannot outlive the
		// object it points at either.
		hmd_->direct_mode().attach_ipc(&ipc_);

		IpcCallbacks callbacks;
		callbacks.on_config = [this](const ipc::HmdConfig & config) {
			if (hmd_)
				hmd_->set_config(config);
		};
		callbacks.on_pose = [this](const ipc::PoseUpdate & pose) {
			switch (static_cast<ipc::DeviceId>(pose.device))
			{
				case ipc::DeviceId::hmd:
					if (hmd_)
						hmd_->on_pose_update(pose);
					break;
				default:
					if (ControllerDevice * controller = controller_for(pose.device); controller != nullptr)
						controller->on_pose_update(pose);
					break;
			}
		};
		callbacks.on_device_add = [this](const ipc::DeviceAdd & add) {
			if (ControllerDevice * controller = controller_for(add.device); controller != nullptr)
				controller->on_device_add();
			else
				WNX_LOG("provider: DeviceAdd for unsupported device id %u", add.device);
		};
		callbacks.on_device_remove = [this](const ipc::DeviceRemove & remove) {
			if (ControllerDevice * controller = controller_for(remove.device); controller != nullptr)
				controller->on_device_remove();
		};
		callbacks.on_input = [this](const ipc::InputUpdate & input) {
			if (ControllerDevice * controller = controller_for(input.device); controller != nullptr)
				controller->on_input_update(input);
		};
		callbacks.on_frame_done = [this](const ipc::FrameDone & done) {
			if (hmd_)
				hmd_->direct_mode().on_frame_done(done);
		};
		callbacks.on_session_ready = [this]() {
			if (hmd_)
				hmd_->direct_mode().on_ipc_connected();
		};
		callbacks.on_disconnect = [this]() {
			if (hmd_)
			{
				hmd_->direct_mode().on_ipc_disconnected();
				hmd_->on_helper_disconnected();
			}
			if (left_)
				left_->on_helper_disconnected();
			if (right_)
				right_->on_helper_disconnected();
		};

		if (!ipc_.start(std::move(callbacks)))
		{
			WNX_LOG("provider: IPC client failed to start");
			breadcrumb("provider: IPC client failed to start, loading inert");
			return vr::VRInitError_None; // inert, not fatal
		}
		breadcrumb("provider: IPC client thread started");

		idle_running_.store(true, std::memory_order_release);
		idle_exited_.store(false, std::memory_order_release);
		try
		{
			idle_thread_ = std::thread([this] { idle_pose_loop(); });
		}
		catch (...)
		{
			idle_running_.store(false, std::memory_order_release);
			idle_exited_.store(true, std::memory_order_release);
			WNX_LOG("provider: failed to start the fallback pose thread");
			breadcrumb("provider: failed to start the fallback pose thread, poses will only "
			           "flow while the helper is connected");
		}

		WNX_LOG("provider: Init done, waiting for the helper's HmdConfig");
		breadcrumb("provider: Init done, waiting for the helper's HmdConfig");
		return vr::VRInitError_None;
	}

	void cleanup_impl()
	{
		// Stop everything that can call back into vrserver *before* the driver
		// context is torn down, then mark the context dead so any thread that
		// somehow outlives this finds a null host instead of a dangling one.
		idle_running_.store(false, std::memory_order_release);
		if (idle_thread_.joinable())
		{
			const auto deadline = std::chrono::steady_clock::now() + kJoinWarnAfter;
			while (!idle_exited_.load(std::memory_order_acquire) &&
			       std::chrono::steady_clock::now() < deadline)
				std::this_thread::sleep_for(std::chrono::milliseconds(2));

			if (!idle_exited_.load(std::memory_order_acquire))
				breadcrumb("provider: fallback pose thread has not exited after %lld ms, joining anyway",
				           static_cast<long long>(kJoinWarnAfter.count()));

			try
			{
				idle_thread_.join();
			}
			catch (...)
			{
				breadcrumb("provider: joining the fallback pose thread threw");
			}
		}
		breadcrumb("provider: fallback pose thread joined");

		ipc_.stop();
		breadcrumb("provider: IPC client stopped");

		left_.reset();
		right_.reset();
		hmd_.reset();

		cleanup_driver_log();
		vrctx::set_live(false);
		VR_CLEANUP_SERVER_DRIVER_CONTEXT();
	}

	void run_frame_impl()
	{
		// vrserver's frame loop can outlive Cleanup by a frame; every openvr
		// accessor dereferences the (now null) driver context, so bail early.
		if (!vrctx::live())
			return;

		vr::IVRServerDriverHost * host = vrctx::host();
		if (host == nullptr)
			return;

		register_pending_devices(host);

		if (hmd_)
			hmd_->push_pending_geometry();

		// Mirrors ALVR's DriverProvider::RunFrame (alvr_server.cpp:127-143):
		// drain the event queue and turn haptic events into something the
		// client can buzz with.
		vr::VREvent_t event{};
		while (host->PollNextEvent(&event, sizeof(event)))
		{
			if (event.eventType != vr::VREvent_Input_HapticVibration)
				continue; // Everything else is drained and ignored.

			const vr::VREvent_HapticVibration_t & haptics = event.data.hapticVibration;

			ipc::DeviceId device{};
			if (left_ && left_->owns_haptic(haptics.containerHandle, haptics.componentHandle))
				device = ipc::DeviceId::left_controller;
			else if (right_ && right_->owns_haptic(haptics.containerHandle, haptics.componentHandle))
				device = ipc::DeviceId::right_controller;
			else
				continue;

			ipc::Haptic haptic{};
			haptic.device = static_cast<uint8_t>(device);
			haptic.duration_s = haptics.fDurationSeconds;
			haptic.frequency_hz = haptics.fFrequency;
			haptic.amplitude = haptics.fAmplitude;

			if (!ipc_.queue_haptic(haptic))
			{
				// Not fatal: either no helper is connected or the queue is
				// saturated. Log sparsely so a stuck helper cannot spam.
				if ((haptic_drop_count_++ % 100) == 0)
					WNX_LOG("provider: dropped a haptic pulse (%llu so far)",
					        static_cast<unsigned long long>(haptic_drop_count_));
			}
		}
	}

	// All TrackedDeviceAdded calls in the driver happen here, on vrserver's
	// thread, exactly one device per frame so a burst of DeviceAdds cannot turn
	// one RunFrame into a long re-entrant stall.
	void register_pending_devices(vr::IVRServerDriverHost * host) noexcept
	{
		if (hmd_ && hmd_->consume_registration_request())
		{
			breadcrumb("provider: registering the HMD with vrserver (serial %s)", HmdDevice::kSerialNumber);
			if (!host->TrackedDeviceAdded(HmdDevice::kSerialNumber,
			                              vr::TrackedDeviceClass_HMD,
			                              static_cast<vr::ITrackedDeviceServerDriver *>(hmd_.get())))
			{
				WNX_LOG("HMD: TrackedDeviceAdded(%s) failed", HmdDevice::kSerialNumber);
				breadcrumb("provider: TrackedDeviceAdded for the HMD failed, will retry");
				hmd_->registration_failed();
			}
			else
			{
				WNX_LOG("HMD: registered with vrserver as %s", HmdDevice::kSerialNumber);
				breadcrumb("provider: HMD registered");
			}
			return;
		}

		for (ControllerDevice * controller: {left_.get(), right_.get()})
		{
			if (controller == nullptr || !controller->consume_registration_request())
				continue;

			breadcrumb("provider: registering controller %s with vrserver", controller->serial());
			if (!host->TrackedDeviceAdded(controller->serial(),
			                              vr::TrackedDeviceClass_Controller,
			                              static_cast<vr::ITrackedDeviceServerDriver *>(controller)))
			{
				WNX_LOG("controller %s: TrackedDeviceAdded failed", controller->serial());
				breadcrumb("provider: TrackedDeviceAdded for %s failed, will retry", controller->serial());
				controller->registration_failed();
			}
			else
			{
				WNX_LOG("controller %s: registered with vrserver", controller->serial());
				breadcrumb("provider: controller %s registered", controller->serial());
			}
			return;
		}
	}

	void idle_pose_loop() noexcept
	{
		while (idle_running_.load(std::memory_order_acquire))
		{
			// The context can go away underneath us during Cleanup; tick_idle
			// itself is null-safe but skipping the work entirely is cheaper.
			if (vrctx::live() && hmd_)
				hmd_->tick_idle();
			std::this_thread::sleep_for(kIdlePosePeriod);
		}
		idle_exited_.store(true, std::memory_order_release);
	}

	// nullptr for the HMD and for anything the helper invents later.
	ControllerDevice * controller_for(uint8_t device) const noexcept
	{
		switch (static_cast<ipc::DeviceId>(device))
		{
			case ipc::DeviceId::left_controller:
				return left_.get();
			case ipc::DeviceId::right_controller:
				return right_.get();
			default:
				return nullptr;
		}
	}

	std::unique_ptr<HmdDevice> hmd_;
	std::unique_ptr<ControllerDevice> left_;
	std::unique_ptr<ControllerDevice> right_;
	IpcClient ipc_;
	std::thread idle_thread_;
	std::atomic<bool> idle_running_{false};
	std::atomic<bool> idle_exited_{true};
	uint64_t haptic_drop_count_ = 0;
};

// Constructed on the first factory call rather than at DLL load: nothing with a
// non-trivial constructor should run under the loader lock, where an allocation
// or a thread creation can deadlock instead of merely failing.
DriverProvider & provider() noexcept
{
	static DriverProvider instance;
	return instance;
}

} // namespace
} // namespace wivrnnx

// SteamVR resolves this by its plain, undecorated name. extern "C" plus
// __declspec(dllexport) yields "HmdDriverFactory" with no decoration on x64 for
// both MSVC and llvm-mingw.
#if defined(_MSC_VER)
#define WNX_DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define WNX_DLL_EXPORT extern "C" __attribute__((visibility("default"))) __declspec(dllexport)
#endif

WNX_DLL_EXPORT void * HmdDriverFactory(const char * interface_name, int * return_code)
{
	try
	{
		wivrnnx::breadcrumb_begin_session();
		wivrnnx::breadcrumb("factory: HmdDriverFactory(\"%s\")",
		                    interface_name != nullptr ? interface_name : "(null)");

		if (interface_name != nullptr &&
		    std::strcmp(interface_name, vr::IServerTrackedDeviceProvider_Version) == 0)
		{
			if (return_code != nullptr)
				*return_code = vr::VRInitError_None;
			wivrnnx::breadcrumb("factory: returning the server device provider");
			return &wivrnnx::provider();
		}

		if (return_code != nullptr)
			*return_code = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}
	catch (...)
	{
		// A throwing factory would unwind into vrserver's loader path.
		if (return_code != nullptr)
			*return_code = vr::VRInitError_Init_InterfaceNotFound;
		return nullptr;
	}
}
