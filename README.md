# WiVRn NX — Windows (real SteamVR)

Streams a real SteamVR session to the Pico with the WiVRn NX stack. Two halves:

- **`shim/`** → `driver_wivrnnx.dll`, a thin OpenVR driver loaded by `vrserver.exe`.
  Presents the Pico as SteamVR's HMD (poses in, composited frames out). No WiVRn
  logic, no networking, no encoding.
- **`helper/`** → `wivrnnx-helper.exe`, the fat part: the WiVRn NX server stack
  (encode, stream, adaptive transport) minus Monado. Talks to the shim over the
  IPC contract in [`ipc/wivrnnx_ipc.h`](ipc/wivrnnx_ipc.h).

The Linux server lives in the `wivrn-nx` repo (branch `nx-patches`); from Phase 1
on this tree reuses its `common/` via the `WIVRNNX_LINUX_REPO` CMake variable.
Design brief: `wivrn-nx/docs/windows-steamvr-feasibility.md` + the thin-shim plan.

## Building

- Windows (real build): `cmake --preset vs2022 && cmake --build --preset vs2022`
- Linux (compile-check only): `tools/compile_check.sh` (needs an extracted
  [llvm-mingw](https://github.com/mstorsjo/llvm-mingw) release; see
  `cmake/llvm-mingw-toolchain.cmake`)

## Installing the driver (Windows)

The build produces a SteamVR driver tree at `build/<preset>/driver/wivrnnx/`.
Register it with:

```
"C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe" adddriver <path-to>\driver\wivrnnx
```

then start `wivrnnx-helper.exe` and SteamVR (either order works).

## Status

Phase 0: skeleton — shim loads in SteamVR and registers a static HMD; helper
streams fake poses over the pipe. Reference implementation for the driver
surface: ALVR `alvr/server_openvr/cpp` (sparse-cloned under `reference/`,
not committed).

## License

The driver (shim) and helper are licensed under GPL-3.0 — they link code from
[WiVRn](https://github.com/WiVRn/WiVRn) (GPL-3.0). See [LICENSE](LICENSE).
Bundled third-party notices (AMD AMF headers, Boost.PFR) ship in the release
zip under `licenses/`.
