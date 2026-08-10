// The IVRDriverDirectModeComponent_009 SteamVR fetches from the HMD via
// GetComponent.
//
// WHY THIS EXISTS (Phase 2, step 1). With no direct-mode component the
// compositor treats the headset as an extended-desktop display: it takes
// Prop_GraphicsAdapterLuid / adapter 0, asks it for output 0, gets
// DXGI_ERROR_NOT_FOUND (887A0022) because our display is fictional, and dies
// with
//
//     Headset display is on desktop -> Invalid adapter output specified! (0)
//     -> VRInitError_Compositor_InvalidOutputDesktop
//
// Answering GetComponent("IVRDriverDirectModeComponent_009") with this object
// tells the compositor the driver owns scanout itself, so it never looks for a
// desktop output. IsDisplayOnDesktop() already returns false
// (display_component.cpp); that alone is not enough.
//
// WHAT IT DOES. It allocates the shared textures the compositor renders into,
// honours the swap-chain rotation and the present handshake, drives vsync, and
// -- Phase 2 step 2 -- flattens the two submitted eye textures into a
// side-by-side staging texture shared with the helper (frame_staging.h,
// protocol v3) and announces it with FrameReady. A frame is dropped, never
// queued, whenever the helper still owns every staging slot: that is the
// contract's backpressure, and it is also what keeps a stalled encoder from
// ever blocking the compositor's present thread.
//
// Everything about the transport degrades to the old behaviour -- drop the
// frame -- rather than failing: no helper, no D3D device, no NT handles, an
// unexpected texture format, all of them just mean frames stop flowing.
//
// Single inheritance, like DisplayComponent, per rule 1 of msvc_abi.h. None of
// the _009 methods returns a struct by value, so no WNX_SRET_THUNK is needed
// here -- but a second base on HmdDevice would still put a this-adjusting thunk
// in front of the methods that do, so this stays its own object.
//
// THREADING. CreateSwapTextureSet / SubmitLayer / Present / PostPresent are
// called by the compositor, which in some SteamVR builds is a separate PROCESS
// from vrserver; the breadcrumb file is shared between them, which is why every
// line carries a pid (breadcrumb.cpp). The vsync thread is ours.
#pragma once

#include "frame_staging.h"
#include "openvr_driver_wrap.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// Kept out of this header on purpose: <d3d11.h> pulls in a large part of the
// Windows SDK and only direct_mode.cpp needs it.
struct ID3D11Device;
struct ID3D11DeviceContext;
struct ID3D11Texture2D;

namespace wivrnnx
{

class HmdDevice;
class IpcClient;

class DirectModeComponent final : public vr::IVRDriverDirectModeComponent
{
public:
	explicit DirectModeComponent(HmdDevice * owner) noexcept;
	// Not virtual: the OpenVR interfaces have no virtual destructor, and this
	// object is a member of HmdDevice, destroyed through its static type.
	~DirectModeComponent();

	DirectModeComponent(const DirectModeComponent &) = delete;
	DirectModeComponent & operator=(const DirectModeComponent &) = delete;

	HmdDevice * owner() const noexcept
	{
		return owner_;
	}

	// Stops the vsync thread and releases every D3D object. Safe to call more
	// than once, and safe to call with the driver context already down.
	//
	// Latching, and therefore called from the destructor only -- deliberately
	// NOT from HmdDevice::Deactivate, which SteamVR may follow with another
	// Activate. The ordering that matters is in DriverProvider::cleanup_impl:
	// hmd_.reset() runs before vrctx::set_live(false), so the vsync thread is
	// joined while the driver context is still up, exactly like the fallback
	// pose thread.
	void shutdown() noexcept;

	// --- video transport wiring ------------------------------------------

	// Set once by the driver provider, before the IPC client thread starts and
	// while nothing else can be running. The client outlives this component
	// (DriverProvider::cleanup_impl stops it before releasing the HMD), so the
	// pointer never dangles.
	void attach_ipc(IpcClient * ipc) noexcept;

	// All three are called from the IPC client thread.
	void on_frame_done(const ipc::FrameDone & done) noexcept;
	// A helper session became usable: re-announce the ring we already have.
	void on_ipc_connected() noexcept;
	// The helper vanished: the staging keyed mutexes are in an unknown state,
	// so the ring is rebuilt (with a new generation) on the next Present.
	void on_ipc_disconnected() noexcept;

	// --- vr::IVRDriverDirectModeComponent (_009) -------------------------

	void CreateSwapTextureSet(uint32_t unPid,
	                          const SwapTextureSetDesc_t * pSwapTextureSetDesc,
	                          SwapTextureSet_t * pOutSwapTextureSet) override;
	void DestroySwapTextureSet(vr::SharedTextureHandle_t sharedTextureHandle) override;
	void DestroyAllSwapTextureSets(uint32_t unPid) override;
	void GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t sharedTextureHandles[2],
	                                uint32_t (*pIndices)[2]) override;
	void SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2]) override;
	void Present(vr::SharedTextureHandle_t syncTexture) override;
	void PostPresent(const Throttling_t * pThrottling) override;
	void GetFrameTiming(vr::DriverDirectMode_FrameTiming * pFrameTiming) override;

private:
	// Defined in direct_mode.cpp: holds the three textures of one swap chain.
	struct TextureSet;
	struct SharedTextureEntry;
	// Everything Present needs out of the last SubmitLayer, copied out under
	// present_mutex_ so the rest of the frame runs without holding it.
	struct FrameSnapshot;

	// --- D3D11, guarded by device_mutex_ ---------------------------------

	// Creates the device on first use. Returns false once it has given up.
	bool ensure_device_locked() noexcept;
	// OpenSharedResource + cache, mirroring CD3DRender::GetSharedTexture
	// (reference/alvr/.../platform/win32/shared/d3drender.cpp:218).
	ID3D11Texture2D * shared_texture_locked(void * handle) noexcept;
	void release_device_locked() noexcept;

	// --- swap texture sets, guarded by sets_mutex_ -----------------------

	TextureSet * find_set_locked(vr::SharedTextureHandle_t handle) noexcept;
	// The one texture of a set a submitted handle names, rather than the set.
	ID3D11Texture2D * find_texture_locked(vr::SharedTextureHandle_t handle) noexcept;
	void destroy_set_locked(size_t index) noexcept;

	// --- video transport, device_mutex_ held ------------------------------

	// Flattens the frame into a staging slot and hands the slot to the helper.
	// Returns true and fills `out` when a FrameReady should be sent; false
	// means the frame was dropped (already counted, and breadcrumbed at a
	// sensible rate).
	bool transport_frame_locked(const FrameSnapshot & frame, ipc::FrameReady & out) noexcept;
	// Counts a dropped frame and breadcrumbs every 100th one.
	void count_drop(const char * reason) noexcept;
	// Queues the current ring's StagingConfig, if there is a helper to take it.
	void send_staging_config() noexcept;

	// --- vsync -----------------------------------------------------------

	void start_vsync_thread() noexcept;
	void vsync_loop() noexcept;
	// Seconds per frame from the HmdConfig the shim already has, clamped.
	double frame_interval_seconds() const noexcept;

	HmdDevice * owner_ = nullptr;

	mutable std::mutex device_mutex_;
	ID3D11Device * device_ = nullptr;
	ID3D11DeviceContext * context_ = nullptr;
	std::vector<SharedTextureEntry> shared_textures_;
	// D3D11CreateDevice attempts that failed. After kMaxDeviceAttempts the
	// component stays alive but inert rather than retrying every frame.
	int device_attempts_ = 0;

	mutable std::mutex sets_mutex_;
	std::vector<std::unique_ptr<TextureSet>> sets_;

	// Everything the present handshake touches, ALVR's m_presentMutex.
	mutable std::mutex present_mutex_;
	vr::SharedTextureHandle_t last_layer_texture_[2] = {0, 0};
	vr::VRTextureBounds_t last_layer_bounds_[2] = {};
	vr::HmdMatrix34_t last_layer_pose_ = {};
	float last_layer_predict_ = 0.0f;
	uint32_t submitted_layers_ = 0;

	// --- video transport --------------------------------------------------

	// Written once by attach_ipc() before any thread that reads it exists.
	std::atomic<IpcClient *> ipc_{nullptr};
	FrameStagingRing ring_;
	// Present thread (device_mutex_): ring creations that failed. Like
	// device_attempts_, this stops us retrying a hopeless allocation every
	// frame for the rest of the session.
	int ring_attempts_ = 0;
	// Set by on_ipc_connected(); consumed by the next Present, so that the only
	// thread ever touching the ring or the IPC send path for video is the
	// compositor's present thread.
	std::atomic<bool> resend_config_{false};

	mutable std::mutex vsync_mutex_;
	std::condition_variable vsync_cv_;
	uint64_t vsync_counter_ = 0;

	std::thread vsync_thread_;
	std::atomic<bool> vsync_started_{false};
	std::atomic<bool> vsync_running_{false};
	std::atomic<bool> vsync_exited_{true};
	std::atomic<bool> shut_down_{false};

	// Counters, for the rate-limited breadcrumbs and for GetFrameTiming.
	std::atomic<uint64_t> submit_count_{0};
	std::atomic<uint64_t> present_count_{0};
	std::atomic<uint64_t> post_present_count_{0};
	std::atomic<uint64_t> frame_timing_count_{0};
	std::atomic<uint64_t> vsync_count_{0};
	std::atomic<uint64_t> acquire_failures_{0};
	std::atomic<uint64_t> frame_id_{0};
	std::atomic<uint64_t> frames_sent_{0};
	std::atomic<uint64_t> frames_dropped_{0};
	std::atomic<uint64_t> frame_done_stale_{0};
	// Last reason passed to count_drop, as a string literal pointer.
	std::atomic<const char *> last_drop_reason_{nullptr};

	// One-shot breadcrumbs: things we want to learn about the real compositor
	// exactly once, not ninety times a second.
	std::atomic<bool> logged_bounds_{false};
	std::atomic<bool> logged_flip_{false};
	std::atomic<bool> logged_msaa_{false};
	std::atomic<bool> logged_format_{false};
	std::atomic<bool> logged_mismatch_{false};
};

} // namespace wivrnnx
