// --amf-probe: a standalone AMF encoder-init diagnostic.
//
// Creates the helper's own D3D11 device and walks the H.264 (AVC) component
// through the exact property set configure_h264 applies, in ever-longer
// prefixes and with single properties left out, Init()ing after each variant.
// Exists because the Polaris legacy driver refuses the production property set
// with a bare AMF_ACCESS_DENIED that names no property, and reproducing that
// under SteamVR needs a headset mid-connect; this needs nothing but the GPU.
#pragma once

#include <cstdint>

namespace wivrnnx::helper
{

// Process exit code: 0 when the probe ran (whatever the per-variant results),
// 1 when there is no D3D11 device or no AMF runtime to probe.
int run_amf_probe(uint32_t width, uint32_t height, float refresh_hz, uint32_t bitrate_bps);

} // namespace wivrnnx::helper
