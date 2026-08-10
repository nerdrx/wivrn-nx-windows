// Phase 0 synthetic tracking: a slow, obviously-fake HMD orbit so a human can
// tell at a glance in SteamVR that poses are flowing end to end.
#pragma once

#include <cstdint>

#include "wivrnnx_ipc.h"

namespace wivrnnx::helper
{

// Pose streaming rate, Hz. Deliberately decoupled from the advertised display
// refresh rate - Phase 0 does not do frame pacing.
inline constexpr double kPoseRateHz = 100.0;

// The config the shim hands to SteamVR. Values are typical of a Pico 4.
ipc::HmdConfig make_hmd_config();

// Sample the synthetic HMD trajectory. `t` is seconds since the stream
// started; `time_qpc` is stamped into the message verbatim.
ipc::PoseUpdate make_hmd_pose(double t, uint64_t time_qpc);

class Bridge;

// --fake: push make_hmd_pose() into the bridge at kPoseRateHz until the
// shutdown event fires. Runs on its own thread and takes the place the WiVRn
// session has in the default mode, so the pipe server cannot tell the two
// apart.
void run_fake_tracker(Bridge & bridge, void * shutdown_event);

} // namespace wivrnnx::helper
