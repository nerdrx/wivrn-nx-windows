# Smoke test round 2 (Windows + SteamVR) — hardened build

The round-1 crash is root-caused and fixed: the cross-compiler and MSVC
disagree on the calling convention for struct-returning virtuals (`GetPose`),
so the driver was corrupting its own objects when real vrserver called it.
This build carries ABI thunks (codegen-verified), plus hardening: the driver
now cannot crash vrserver — worst case it goes inert and writes down why.

## Install

1. Replace the old folder with this one (e.g. `C:\wivrnnx`), keep the path.
2. If the path changed, re-register:
   `vrpathreg.exe adddriver C:\wivrnnx\driver\wivrnnx` (vrpathreg lives in
   `Steam\steamapps\common\SteamVR\bin\win64\`).
3. Re-enable the blocked addon: SteamVR → Settings → Startup / Shutdown →
   Manage SteamVR Add-Ons → enable `wivrnnx` (decline "safe mode" if offered).

## Test A — SteamVR only (no headset)

1. Start `wivrnnx-helper.exe --fake` (note the flag — it replays Phase 0's
   synthetic orbit; without it the helper waits for a real Pico instead).
2. Start SteamVR. Expect: headset icon, green tracking, slow orbit + yaw sway
   in the VR View. Kill/restart the helper → static pose, then orbit resumes.

## Test B — real Pico (can run without SteamVR too)

1. Start `wivrnnx-helper.exe` (no flag). It prints a pairing PIN.
2. Pico: open the WiVRn NX client → this PC appears in the lobby → connect,
   enter PIN. Headset will sit at "waiting for video" — correct for Phase 1 —
   while real head/controller tracking drives SteamVR.

## Logs — collect these regardless of outcome

- **`C:\ProgramData\wivrnnx\driver.log`** (new): the driver's own breadcrumb
  file — records factory call, which SteamVR interfaces resolved, every
  GetComponent request, activation, first pose. Exists even when SteamVR's
  logs say nothing. Fallback location: `%TEMP%\wivrnnx-driver.log`.
- `C:\Program Files (x86)\Steam\logs\vrserver.txt` and **`vrserver.prev.txt`**
  (after a crash the interesting log is the `.prev` one) — grep `wivrnnx`.
- Same folder: `vrcompositor.txt` — needed for the black-screen/stall case.
- SteamVR version: `Steam\steamapps\common\SteamVR\bin\version.txt`.

## Expected imperfections at this stage

- Compositor may stall or show black even with tracking green: we ship no
  direct-mode component yet (that's Phase 2). Tracking working while video
  is absent is a PASS for this round.
- Legacy-input-only games see no bindings (known, Phase 4).
- First helper start: allow the Windows Firewall prompts (mDNS + port 9757).
