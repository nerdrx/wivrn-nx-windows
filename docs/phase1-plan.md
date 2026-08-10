# Phase 1 — tracked HMD from a real Pico (concrete plan)

Goal: the actual Pico client (unmodified NX APK) pairs with `wivrnnx-helper.exe`
over WiFi, and SteamVR shows a head-tracked (then controller-tracked) headset.
Video stays fake/absent; SteamVR's "headset" renders nothing yet.

## What moves from the Linux repo (via `WIVRNNX_LINUX_REPO`)

Compiled ~as-is (portability shims noted):

| Piece | Files (wivrn-nx repo) | Port note |
|---|---|---|
| Wire protocol | `common/wivrn_packets.h`, `wivrn_serialization*.h` | byte-compatible, do not fork — same APK must pair |
| Sockets | `common/wivrn_sockets.{h,cpp}` | Winsock2 + WSAPoll shim; `MSG_ZEROCOPY`/`SO_TXTIME`-style Linux-isms behind `#ifdef _WIN32` fallbacks in a *new* `helper/net/` wrapper, keeping `common/` clean |
| Pairing/crypto | `common/crypto.*`, `secrets.*`, `smp.*` | OpenSSL via vcpkg |
| Handshake | `server/accept_connection.cpp` (99 lines) | small; copy-adapt into helper (it is server/-side, not common/) |
| Connection | `server/driver/wivrn_connection.*` | copy-adapt; drop Monado includes |
| Clock sync | `server/driver/clock_offset.*` | needed to map client timestamps → QPC for the shim |
| Discovery | `external/mdns.h` (already vendored in wivrn-nx) | replaces avahi; works on Winsock |

Deliberately NOT ported in Phase 1: pose_list/view_list prediction (xrt-typed —
the shim gets raw latest-sample poses first; prediction/interpolation comes with
Phase 2 timing), audio, encoders, multipath, bitrate control.

## New helper code

1. `helper/net/` — Winsock bootstrap (WSAStartup RAII), the `wivrn_sockets`
   portability wrapper, mdns announcer (same service name/TXT records as the
   Linux server so the client's lobby lists it identically).
2. `helper/session.cpp` — accept → pairing (reuse smp/secrets flow) → stream
   tracking packets. Replaces `wivrn_session.cpp`'s tracking half only.
3. `helper/tracking_bridge.cpp` — `from_headset::tracking` → sanitize
   (port the NX standby-pose/NaN boundary from `server/driver/pose_sanitize.h`,
   re-expressed on raw quats/vec3 instead of `xrt_space_relation`) →
   `ipc::PoseUpdate` per device → pipe. Clock-offset applied here.
4. Config plumbing: derive `HmdConfig` from the client's `headset_info_packet`
   (real resolution/refresh/FOV instead of Phase 0 constants).

## Shim additions (small)

- Controller devices: `DeviceAdd`-style IPC messages (extend `wivrnnx_ipc.h` —
  version bump to 2) → `TrackedDeviceAdded` with a minimal input profile
  (system/trigger/grip/joystick/clicks), `vr::VRDriverInput()` updates.
  Reference: ALVR `Controller.cpp`.
- Haptics: `IVRServerDriverHost` haptic events → IPC → helper → client
  (`to_headset::haptics` already exists in the protocol).

## Validation ladder (each step independently testable)

1. Helper announces on mDNS → Pico lobby shows the server (no SteamVR needed).
2. Pairing completes; helper logs tracking packet rate (~500/s on Pico).
3. `vrmonitor` shows the HMD tracking (shim + helper + SteamVR, no game).
4. Controllers appear and buttons fire in SteamVR's input debugger.
5. Standby test: let the Pico sleep/wake — poses must stay sane (the NX fix).

## Risks

- `wivrn_sockets` Linux-isms run deeper than expected → budget for the wrapper
  growing; keep all `#ifdef _WIN32` out of `common/`.
- Pairing UX with no dashboard: Phase 1 uses a console prompt for the PIN;
  tray/UI is Phase 4.
- vcpkg deps enter here (openssl, maybe boost-pfr if serialization needs it) —
  first real test of the vs2022 preset.
