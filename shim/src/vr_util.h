// Small conversion helpers shared by the HMD and the controller devices.
//
// Everything here is header-inline and free of state so both translation units
// can use it without another object file.
#pragma once

#include "openvr_driver_wrap.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <chrono>
#include <cmath>
#include <cstdint>

namespace wivrnnx::util
{

// Clamp for anything we hand to the compositor as a time offset. A helper with a
// broken clock must not be able to push SteamVR's prediction off a cliff.
inline constexpr double kMaxTimeOffsetSeconds = 0.1;

inline vr::HmdQuaternion_t make_quaternion(double w, double x, double y, double z) noexcept
{
	vr::HmdQuaternion_t q;
	q.w = w;
	q.x = x;
	q.y = y;
	q.z = z;
	return q;
}

// A fully zeroed DriverPose_t is *not* a valid pose (the quaternions must be
// unit), so every pose we submit starts from this.
inline vr::DriverPose_t make_base_pose() noexcept
{
	vr::DriverPose_t pose{};
	pose.poseTimeOffset = 0.0;
	pose.qWorldFromDriverRotation = make_quaternion(1.0, 0.0, 0.0, 0.0);
	pose.qDriverFromHeadRotation = make_quaternion(1.0, 0.0, 0.0, 0.0);
	pose.qRotation = make_quaternion(1.0, 0.0, 0.0, 0.0);
	for (int i = 0; i < 3; ++i)
	{
		pose.vecWorldFromDriverTranslation[i] = 0.0;
		pose.vecDriverFromHeadTranslation[i] = 0.0;
		pose.vecPosition[i] = 0.0;
		pose.vecVelocity[i] = 0.0;
		pose.vecAcceleration[i] = 0.0;
		pose.vecAngularVelocity[i] = 0.0;
		pose.vecAngularAcceleration[i] = 0.0;
	}
	pose.result = vr::TrackingResult_Running_OK;
	pose.poseIsValid = true;
	pose.willDriftInYaw = false;
	pose.shouldApplyHeadModel = false;
	pose.deviceIsConnected = true;
	return pose;
}

inline double qpc_frequency() noexcept
{
	static const double frequency = [] {
		LARGE_INTEGER f{};
		::QueryPerformanceFrequency(&f);
		return f.QuadPart != 0 ? static_cast<double>(f.QuadPart) : 1.0;
	}();
	return frequency;
}

inline uint64_t query_qpc() noexcept
{
	LARGE_INTEGER counter{};
	::QueryPerformanceCounter(&counter);
	return static_cast<uint64_t>(counter.QuadPart);
}

inline int64_t steady_ticks() noexcept
{
	return std::chrono::steady_clock::now().time_since_epoch().count();
}

// Turns a helper QPC stamp into the "seconds from now" convention OpenVR uses
// for DriverPose_t::poseTimeOffset and for the input update time offsets:
// negative means the sample is already in the past. A zero stamp means "the
// helper did not timestamp this", which degrades to 0.0.
inline double qpc_offset_seconds(uint64_t sample_qpc) noexcept
{
	if (sample_qpc == 0)
		return 0.0;

	double offset = (static_cast<double>(sample_qpc) - static_cast<double>(query_qpc())) / qpc_frequency();
	if (offset > kMaxTimeOffsetSeconds)
		offset = kMaxTimeOffsetSeconds;
	else if (offset < -kMaxTimeOffsetSeconds)
		offset = -kMaxTimeOffsetSeconds;
	return offset;
}

// Rotation (w,x,y,z) + translation -> HmdMatrix34_t, the row-major 3x4 SteamVR
// wants for SetDisplayEyeToHead. Same expansion as ALVR's pose_to_mat
// (alvr_server/Utils.h:83-103).
inline vr::HmdMatrix34_t make_matrix34(const float q[4], const float p[3]) noexcept
{
	const float w = q[0];
	const float x = q[1];
	const float y = q[2];
	const float z = q[3];

	vr::HmdMatrix34_t mat{};

	mat.m[0][0] = 1.0f - 2.0f * (y * y + z * z);
	mat.m[0][1] = 2.0f * (x * y - w * z);
	mat.m[0][2] = 2.0f * (x * z + w * y);
	mat.m[1][0] = 2.0f * (x * y + w * z);
	mat.m[1][1] = 1.0f - 2.0f * (x * x + z * z);
	mat.m[1][2] = 2.0f * (y * z - w * x);
	mat.m[2][0] = 2.0f * (x * z - w * y);
	mat.m[2][1] = 2.0f * (y * z + w * x);
	mat.m[2][2] = 1.0f - 2.0f * (x * x + y * y);

	mat.m[0][3] = p[0];
	mat.m[1][3] = p[1];
	mat.m[2][3] = p[2];

	return mat;
}

// The inverse of make_matrix34: HmdMatrix34_t -> rotation (w,x,y,z) +
// translation. Needed for the frame transport, where the only pose the
// compositor gives us is SubmitLayerPerEye_t::mHmdPose, a 3x4 matrix, and the
// wire format (ipc::FrameReady) carries a quaternion.
//
// Shepperd's method: pick the branch whose divisor is largest so the square root
// never lands near zero. A matrix that is not a rotation at all (all zeros is
// the realistic case -- a compositor that submitted before it had a pose)
// degrades to identity rather than to NaN, because a NaN quaternion on the wire
// would poison the client's reprojection.
inline void decompose_matrix34(const vr::HmdMatrix34_t & m, float out_q[4], float out_p[3]) noexcept
{
	out_q[0] = 1.0f;
	out_q[1] = 0.0f;
	out_q[2] = 0.0f;
	out_q[3] = 0.0f;
	out_p[0] = 0.0f;
	out_p[1] = 0.0f;
	out_p[2] = 0.0f;

	for (int row = 0; row < 3; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			if (!std::isfinite(m.m[row][col]))
				return; // identity + origin
		}
	}

	out_p[0] = m.m[0][3];
	out_p[1] = m.m[1][3];
	out_p[2] = m.m[2][3];

	const float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w,x,y,z

	if (trace > 0.0f)
	{
		const float s = std::sqrt(trace + 1.0f) * 2.0f;
		q[0] = 0.25f * s;
		q[1] = (m.m[2][1] - m.m[1][2]) / s;
		q[2] = (m.m[0][2] - m.m[2][0]) / s;
		q[3] = (m.m[1][0] - m.m[0][1]) / s;
	}
	else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2])
	{
		const float s = std::sqrt(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0f;
		q[0] = (m.m[2][1] - m.m[1][2]) / s;
		q[1] = 0.25f * s;
		q[2] = (m.m[0][1] + m.m[1][0]) / s;
		q[3] = (m.m[0][2] + m.m[2][0]) / s;
	}
	else if (m.m[1][1] > m.m[2][2])
	{
		const float s = std::sqrt(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0f;
		q[0] = (m.m[0][2] - m.m[2][0]) / s;
		q[1] = (m.m[0][1] + m.m[1][0]) / s;
		q[2] = 0.25f * s;
		q[3] = (m.m[1][2] + m.m[2][1]) / s;
	}
	else
	{
		const float s = std::sqrt(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0f;
		q[0] = (m.m[1][0] - m.m[0][1]) / s;
		q[1] = (m.m[0][2] + m.m[2][0]) / s;
		q[2] = (m.m[1][2] + m.m[2][1]) / s;
		q[3] = 0.25f * s;
	}

	const float norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (!std::isfinite(norm) || norm < 1e-6f)
		return; // identity, translation already stored

	for (int i = 0; i < 4; ++i)
		out_q[i] = q[i] / norm;
}

} // namespace wivrnnx::util
