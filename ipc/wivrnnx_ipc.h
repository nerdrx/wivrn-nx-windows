// WiVRn NX Windows — shim <-> helper IPC protocol.
//
// FROZEN CONTRACT. Both driver_wivrnnx.dll (the in-vrserver shim) and
// wivrnnx-helper.exe include this header and nothing else from each other.
// Bump kProtocolVersion on any wire change; both sides refuse mismatches.
//
// Transport: one duplex named pipe, message mode (PIPE_TYPE_MESSAGE |
// PIPE_READMODE_MESSAGE). The HELPER is the pipe server, the SHIM is the
// client and reconnects with backoff — vrserver may outlive or predate the
// helper and both orders must work.
//
// Every message is exactly one pipe message: a 4-byte MessageType followed by
// the fixed-size POD payload for that type. No variable-length payloads.
// All structs are little-endian, packed, and static_assert-sized.
//
// Handshake: on connect the shim sends ShimHello; the helper replies
// HelperHello then HmdConfig. The shim must not register its HMD with
// vrserver until HmdConfig arrives (SteamVR queries the display component
// immediately on activation). After that, PoseUpdate streams helper->shim at
// the helper's cadence; KeepAlive flows shim->helper at ~1 Hz.
//
// Reconnect: the handshake (including HmdConfig) runs again on every
// reconnect. A shim whose HMD is already registered treats any config after
// its first as a properties-only refresh — vrserver cannot re-register.
// KeepAlive carries no timeout obligation in protocol v1: the helper counts
// them but must not drop a silent client (a watchdog is a v2 decision).
//
// Phase 2 (video) message types are reserved below but their payloads are
// NOT yet frozen — do not implement them.

#pragma once

#include <cstdint>

namespace wivrnnx::ipc {

inline constexpr const char * kPipeName = "\\\\.\\pipe\\wivrnnx-driver";
inline constexpr const wchar_t * kPipeNameW = L"\\\\.\\pipe\\wivrnnx-driver";
inline constexpr uint32_t kProtocolVersion = 3;
inline constexpr uint32_t kShimMagic = 0x53584E57;   // 'WNXS'
inline constexpr uint32_t kHelperMagic = 0x48584E57; // 'WNXH'
inline constexpr uint32_t kMaxMessageSize = 512;

enum class MessageType : uint32_t
{
	// shim -> helper
	shim_hello = 1,
	keep_alive = 2,
	haptic = 3,

	// helper -> shim
	helper_hello = 100,
	hmd_config = 101,
	pose_update = 102,
	device_add = 103,
	device_remove = 104,
	input_update = 105,

	// Video frame transport (v3).
	staging_config = 200, // shim -> helper
	frame_ready = 201,    // shim -> helper
	frame_done = 202,     // helper -> shim
};

#pragma pack(push, 1)

// shim -> helper, first message after connect.
struct ShimHello
{
	uint32_t magic;   // kShimMagic
	uint32_t version; // kProtocolVersion
	uint32_t vrserver_pid; // for DuplicateHandle in Phase 2
};
static_assert(sizeof(ShimHello) == 12);

// shim -> helper, ~1 Hz; lets the helper notice a hung vrserver.
struct KeepAlive
{
	uint64_t time_qpc; // QueryPerformanceCounter at send
};
static_assert(sizeof(KeepAlive) == 8);

// helper -> shim, reply to ShimHello.
struct HelperHello
{
	uint32_t magic;   // kHelperMagic
	uint32_t version; // kProtocolVersion
};
static_assert(sizeof(HelperHello) == 8);

// helper -> shim, once after HelperHello (and again if the client headset's
// advertised mode changes — the shim may ignore runtime changes in Phase 0/1).
struct HmdConfig
{
	uint32_t eye_width;   // per-eye render target, pixels
	uint32_t eye_height;
	float refresh_hz;
	float ipd_m; // informational; eye geometry authority is eye_to_head below
	// Raw projection half-angle tangents per eye, SteamVR convention:
	// GetProjectionRaw(&left, &right, &top, &bottom). Index 0 = left eye.
	float proj_left[2];
	float proj_right[2];
	float proj_top[2];
	float proj_bottom[2];
	// Eye-to-head transform per eye (v2): rotation (w,x,y,z) + translation,
	// metres, for SetDisplayEyeToHead / GetEyeToHeadTransform. A plain
	// non-canted headset sends identity rotations and (∓ipd/2, 0, 0).
	float eye_to_head_q[2][4];
	float eye_to_head_p[2][3];
};
static_assert(sizeof(HmdConfig) == 104);

enum class DeviceId : uint8_t
{
	hmd = 0,
	left_controller = 1,
	right_controller = 2,
};

// helper -> shim, streamed. Right-handed, +y up, metres — the OpenVR
// tracking-universe convention; the shim copies fields into DriverPose_t
// verbatim (identity world/head offsets).
struct PoseUpdate
{
	uint64_t time_qpc;     // helper's QPC timestamp for this sample
	uint8_t device;        // DeviceId
	uint8_t connected;     // 0 = report device disconnected / pose invalid
	uint8_t _pad[6];
	double qw, qx, qy, qz; // orientation
	double px, py, pz;     // position, metres
	double vx, vy, vz;     // linear velocity, m/s
	double wx, wy, wz;     // angular velocity, rad/s
};
static_assert(sizeof(PoseUpdate) == 120);

// helper -> shim (v2). Sent when a device becomes available; the shim calls
// TrackedDeviceAdded on first sight of a DeviceId and treats a repeat add
// (e.g. after an IPC reconnect) as "mark connected again" — vrserver cannot
// unregister a device, so DeviceRemove only marks it disconnected.
struct DeviceAdd
{
	uint8_t device; // DeviceId; hmd is implicit via HmdConfig, never sent here
	uint8_t _pad[3];
};
static_assert(sizeof(DeviceAdd) == 4);

struct DeviceRemove
{
	uint8_t device;
	uint8_t _pad[3];
};
static_assert(sizeof(DeviceRemove) == 4);

// Button bits for InputUpdate. Fixed Pico-controller-shaped set for v2;
// the shim maps them onto its input profile components.
enum class Button : uint32_t
{
	system = 1u << 0,
	menu = 1u << 1,
	a = 1u << 2, // right controller
	b = 1u << 3,
	x = 1u << 4, // left controller
	y = 1u << 5,
	trigger_click = 1u << 6,
	trigger_touch = 1u << 7,
	grip_click = 1u << 8,
	thumbstick_click = 1u << 9,
	thumbstick_touch = 1u << 10,
};

// helper -> shim (v2), streamed at the client's input cadence.
struct InputUpdate
{
	uint64_t time_qpc;
	uint8_t device; // DeviceId, controllers only
	uint8_t _pad[3];
	uint32_t pressed; // Button bitmask
	uint32_t touched; // Button bitmask
	float trigger;    // 0..1
	float grip;       // 0..1
	float thumbstick_x; // -1..1
	float thumbstick_y;
};
static_assert(sizeof(InputUpdate) == 36);

// shim -> helper (v2): vrserver haptic event, forwarded to the client.
struct Haptic
{
	uint8_t device;
	uint8_t _pad[7];
	float duration_s;
	float frequency_hz;
	float amplitude; // 0..1
};
static_assert(sizeof(Haptic) == 20);

// ---------------------------------------------------------------------------
// Video frame transport (v3).
//
// The shim owns a ring of `count` side-by-side staging textures (full frame,
// both eyes flattened, width = 2*eye_width) created with
// D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX | D3D11_RESOURCE_MISC_SHARED_NTHANDLE.
// At Present it blits the submitted eye textures (applying the layer bounds)
// into the next free staging slot, releases the keyed mutex with key 1, and
// sends FrameReady. The helper duplicates the NT handles out of vrserver
// (ShimHello.vrserver_pid + PROCESS_DUP_HANDLE, same user), opens them on its
// own D3D11 device via OpenSharedResource1, and for each frame acquires the
// mutex with key 1, encodes, releases with key 0, then sends FrameDone.
// The shim acquires with key 0 before reusing a slot; a slot whose FrameDone
// has not arrived is skipped (natural encoder backpressure: frames drop in
// the shim, never queue). The helper MAY send FrameDone flags=1 without ever
// having acquired the mutex (no client, stale generation) — that leaves the
// mutex released with key 1, so the shim's reuse acquire falls back to
// AcquireSync(1) to reclaim its own unconsumed release.
//
// StagingConfig is re-sent whenever the ring is recreated (mode change,
// device loss); `generation` increments and all older slots become invalid.
// A FrameReady/FrameDone whose generation is stale must be ignored.
// ---------------------------------------------------------------------------

// shim -> helper.
struct StagingConfig
{
	uint32_t generation;
	uint32_t width;  // full side-by-side width (2x per-eye)
	uint32_t height;
	uint32_t dxgi_format; // DXGI_FORMAT of the staging textures
	uint32_t count;       // ring size, <= 3
	uint32_t _pad;
	uint64_t handles[3]; // NT handle values valid in the vrserver process
};
static_assert(sizeof(StagingConfig) == 48);

// shim -> helper, one per flattened frame.
struct FrameReady
{
	uint64_t frame_id;        // monotonically increasing
	uint64_t sample_time_qpc; // when Present ran
	uint32_t generation;
	uint32_t staging_index;
	// HMD pose this frame was rendered with (from SubmitLayer), for the
	// client's reprojection.
	float qw, qx, qy, qz;
	float px, py, pz;
	float predict_s; // frame pose prediction interval SteamVR used
};
static_assert(sizeof(FrameReady) == 56);

// helper -> shim.
struct FrameDone
{
	uint64_t frame_id;
	uint32_t staging_index;
	uint32_t flags; // 0 = encoded+sent; 1 = dropped by encoder
};
static_assert(sizeof(FrameDone) == 16);

#pragma pack(pop)

} // namespace wivrnnx::ipc
