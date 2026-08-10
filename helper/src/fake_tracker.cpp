#include "fake_tracker.h"

#include <windows.h>

#include <chrono>
#include <cmath>
#include <cstring>

#include "bridge.h"

namespace wivrnnx::helper
{

namespace
{

// Orbit: 15 cm radius in the xz plane, ~1.6 m eye height with a gentle bob.
constexpr double kOrbitRadiusM = 0.15;
constexpr double kOrbitOmega = 0.5; // rad/s
constexpr double kEyeHeightM = 1.6;
constexpr double kBobAmpM = 0.05;
constexpr double kBobOmega = 1.3; // rad/s

// Yaw oscillates +-20 degrees about the vertical axis.
constexpr double kPi = 3.14159265358979323846;
constexpr double kYawAmpRad = 20.0 * kPi / 180.0;
constexpr double kYawOmega = 0.3; // rad/s

} // namespace

ipc::HmdConfig make_hmd_config()
{
	ipc::HmdConfig cfg{};
	cfg.eye_width = 2160;
	cfg.eye_height = 2160;
	cfg.refresh_hz = 90.0f;
	cfg.ipd_m = 0.063f;

	// Symmetric 90-degree frustum per eye, in the sign convention SteamVR's
	// GetProjectionRaw(left, right, top, bottom) expects: x grows to the
	// right (left < 0 < right) and the *top* edge is the negative tangent.
	// Cross-checked against ALVR, which feeds tan(fov.down) - an OpenXR
	// angleDown, i.e. negative - into `top` and tan(fov.up) into `bottom`
	// (reference/alvr/.../alvr_server/HMD.cpp GetProjectionRaw +
	// Utils.h fov_to_tangents).
	for (int eye = 0; eye < 2; ++eye)
	{
		cfg.proj_left[eye] = -1.0f;
		cfg.proj_right[eye] = 1.0f;
		cfg.proj_top[eye] = -1.0f;
		cfg.proj_bottom[eye] = 1.0f;
	}

	// Protocol v2 added the per-eye eye-to-head transforms, and they must be
	// filled in even here: a zero-initialised quaternion is not a rotation, and
	// the shim would hand vrserver a degenerate GetEyeToHeadTransform. A plain
	// non-canted headset is identity rotation and a pure lateral offset of half
	// the IPD, which is exactly what this synthetic one is.
	//
	// This is not only the --fake config: it also seeds the bridge in the real
	// mode, so it is what a shim that connects before any headset gets.
	for (int eye = 0; eye < 2; ++eye)
	{
		cfg.eye_to_head_q[eye][0] = 1.0f; // w
		cfg.eye_to_head_q[eye][1] = 0.0f;
		cfg.eye_to_head_q[eye][2] = 0.0f;
		cfg.eye_to_head_q[eye][3] = 0.0f;

		cfg.eye_to_head_p[eye][0] = (eye == 0 ? -0.5f : 0.5f) * cfg.ipd_m;
		cfg.eye_to_head_p[eye][1] = 0.0f;
		cfg.eye_to_head_p[eye][2] = 0.0f;
	}

	return cfg;
}

ipc::PoseUpdate make_hmd_pose(double t, uint64_t time_qpc)
{
	ipc::PoseUpdate pose{};
	pose.time_qpc = time_qpc;
	pose.device = static_cast<uint8_t>(ipc::DeviceId::hmd);
	pose.connected = 1;

	// Position and its analytic derivative.
	pose.px = kOrbitRadiusM * std::sin(kOrbitOmega * t);
	pose.py = kEyeHeightM + kBobAmpM * std::sin(kBobOmega * t);
	pose.pz = kOrbitRadiusM * std::cos(kOrbitOmega * t);

	pose.vx = kOrbitRadiusM * kOrbitOmega * std::cos(kOrbitOmega * t);
	pose.vy = kBobAmpM * kBobOmega * std::cos(kBobOmega * t);
	pose.vz = -kOrbitRadiusM * kOrbitOmega * std::sin(kOrbitOmega * t);

	// Orientation: pure yaw about +y (up), right-handed.
	const double yaw = kYawAmpRad * std::sin(kYawOmega * t);
	pose.qw = std::cos(yaw * 0.5);
	pose.qx = 0.0;
	pose.qy = std::sin(yaw * 0.5);
	pose.qz = 0.0;

	// Angular velocity is d(yaw)/dt about the same axis.
	pose.wx = 0.0;
	pose.wy = kYawAmpRad * kYawOmega * std::cos(kYawOmega * t);
	pose.wz = 0.0;

	return pose;
}

void run_fake_tracker(Bridge & bridge, void * shutdown_event)
{
	using clock = std::chrono::steady_clock;

	const auto period = std::chrono::duration_cast<clock::duration>(
	        std::chrono::duration<double>(1.0 / kPoseRateHz));

	const auto start = clock::now();
	auto next_tick = start;

	bridge.set_config(make_hmd_config());
	bridge.set_present(ipc::DeviceId::hmd, true);

	for (;;)
	{
		const auto now = clock::now();
		if (now < next_tick)
		{
			const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(next_tick - now).count();
			const DWORD wait_ms = static_cast<DWORD>((ns + 999999) / 1000000);
			if (WaitForSingleObject(static_cast<HANDLE>(shutdown_event), wait_ms) == WAIT_OBJECT_0)
				return;
			continue;
		}

		LARGE_INTEGER qpc{};
		QueryPerformanceCounter(&qpc);

		const double t = std::chrono::duration<double>(now - start).count();
		bridge.set_pose(make_hmd_pose(t, static_cast<uint64_t>(qpc.QuadPart)));

		next_tick += period;
		if (next_tick < now)
			next_tick = now + period; // resync after a stall, no catch-up burst
	}
}

} // namespace wivrnnx::helper
