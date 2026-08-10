// Last line of defence between the headset's tracking packets and the shim.
//
// Two guards, both ported from the Linux driver but re-expressed on the raw
// wire types (from_headset::tracking::pose) rather than on Monado's xrt_
// structures, which do not exist here:
//
//  1. NaN/Inf rejection. server/driver/pose_sanitize.h (is_finite /
//     is_valid_orientation / relation_sanitizer) and its ingest-side twin
//     pose_list::is_sane (server/driver/pose_list.cpp:145). A quaternion is
//     usable only if it is finite *and* far enough from zero to normalise —
//     an all-zero quaternion is the usual shape of "the runtime never filled
//     this in", and normalising it gives 0/0.
//
//  2. The NX standby freeze. pose_list::update_tracked_state /
//     component_tracked (server/driver/pose_list.h:73-145). Some runtimes,
//     the Pico's among them, keep reporting a pose as *valid* but no longer
//     *tracked* once a controller goes to standby, and that pose drifts —
//     which is what makes a resting controller teleport across the room.
//     Once a component has ever been tracked, a valid-but-untracked sample
//     for it is dropped and the last tracked value is held instead.
//
// The two components of a pose are tracked independently, exactly as they are
// on Linux: a controller can lose positional tracking while its IMU keeps the
// orientation live.
#pragma once

#include <cstdint>
#include <limits>

#include "is_finite.h" // common/is_finite.h - bit-pattern checks, fast-math safe
#include "wivrn_packets.h"

namespace wivrnnx::helper
{

using wivrn::is_finite;

inline bool is_finite_vec(const XrVector3f & v)
{
	return is_finite(v.x) and is_finite(v.y) and is_finite(v.z);
}

inline bool is_valid_orientation(const XrQuaternionf & q)
{
	if (not(is_finite(q.x) and is_finite(q.y) and is_finite(q.z) and is_finite(q.w)))
		return false;

	const float norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	return is_finite(norm2) and norm2 > 0.1f and norm2 < 10.f;
}

// Only call on a quaternion that passed is_valid_orientation().
inline void normalize_orientation(XrQuaternionf & q)
{
	const float norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
	const float inv = 1.f / __builtin_sqrtf(norm2);
	q.x *= inv;
	q.y *= inv;
	q.z *= inv;
	q.w *= inv;
}

// True if every float the flags claim to be valid is finite and the orientation,
// if claimed valid, can be normalised. Verbatim policy of pose_list::is_sane.
inline bool is_sane(const wivrn::from_headset::tracking::pose & pose)
{
	using flags = wivrn::from_headset::pose_flags;

	if ((pose.flags & flags::orientation_valid) and not is_valid_orientation(pose.pose.orientation))
		return false;

	if ((pose.flags & flags::position_valid) and not is_finite_vec(pose.pose.position))
		return false;

	if ((pose.flags & flags::linear_velocity_valid) and not is_finite_vec(pose.linear_velocity))
		return false;

	if ((pose.flags & flags::angular_velocity_valid) and not is_finite_vec(pose.angular_velocity))
		return false;

	return true;
}

// Per-device state. One instance per DeviceId; not thread safe, and only ever
// touched from the network thread.
class PoseSanitizer
{
public:
	// The composed result: last-good values substituted for anything this
	// sample could not supply.
	struct Result
	{
		XrQuaternionf orientation{0, 0, 0, 1};
		XrVector3f position{0, 0, 0};
		XrVector3f linear_velocity{0, 0, 0};
		XrVector3f angular_velocity{0, 0, 0};
		// False until both components have been seen at least once. The shim
		// reports the device as disconnected while this is false rather than
		// claiming an identity pose is real tracking.
		bool usable = false;
		// True when this sample was taken as-is; false when any part of it was
		// rejected or frozen.
		bool clean = false;
	};

	// `freeze` is the headset's standby_freeze setting
	// (from_headset::settings_changed::standby_freeze, default true — the NX
	// behaviour). The tracked state is maintained whatever it says, so that
	// turning the freeze back on picks up at the right sample rather than at
	// whatever happens to arrive next; that is deliberate on Linux too.
	Result sanitize(const wivrn::from_headset::tracking::pose & pose,
	                int64_t production_timestamp,
	                int64_t timestamp,
	                bool freeze);

	uint64_t rejected() const
	{
		return rejected_;
	}

	uint64_t frozen() const
	{
		return frozen_;
	}

	void reset()
	{
		*this = PoseSanitizer{};
	}

private:
	// Port of pose_list::tracked_state.
	struct TrackedState
	{
		bool ever_tracked = false;
		bool currently_tracked = false;
		// Production timestamp of the sample that last updated this state, so
		// that an out-of-order sample cannot undo a newer one.
		int64_t production_timestamp = std::numeric_limits<int64_t>::lowest();
	};

	// Port of pose_list::update_tracked_state. Returns true if the sample
	// should be taken for this component.
	static bool update_tracked_state(TrackedState & state,
	                                 int64_t production_timestamp,
	                                 bool valid,
	                                 bool tracked,
	                                 bool freeze);

	TrackedState position_state_;
	TrackedState orientation_state_;

	XrQuaternionf last_orientation_{0, 0, 0, 1};
	XrVector3f last_position_{0, 0, 0};
	bool have_orientation_ = false;
	bool have_position_ = false;

	uint64_t rejected_ = 0;
	uint64_t frozen_ = 0;
};

inline bool PoseSanitizer::update_tracked_state(TrackedState & state,
                                                int64_t production_timestamp,
                                                bool valid,
                                                bool tracked,
                                                bool freeze)
{
	// A sample without valid data is forwarded as-is; it invalidates the pose.
	if (not valid)
		return true;

	const bool up_to_date = production_timestamp >= state.production_timestamp;

	if (tracked)
	{
		state.ever_tracked = true;
		if (up_to_date)
		{
			state.currently_tracked = true;
			state.production_timestamp = production_timestamp;
		}
		return true;
	}

	if (up_to_date)
	{
		state.currently_tracked = false;
		state.production_timestamp = production_timestamp;
	}

	// Runtimes that never report the tracked flag at all keep the upstream
	// behaviour; for the others the sample is dropped so the pose is held at
	// the last tracked one.
	return not freeze or not state.ever_tracked;
}

inline PoseSanitizer::Result PoseSanitizer::sanitize(const wivrn::from_headset::tracking::pose & pose,
                                                     int64_t production_timestamp,
                                                     int64_t /*timestamp*/,
                                                     bool freeze)
{
	using flags = wivrn::from_headset::pose_flags;

	Result result;

	// A sample carrying a non-finite float is not partially usable: reject the
	// whole thing and hold whatever we had.
	if (not is_sane(pose))
	{
		++rejected_;
		result.orientation = last_orientation_;
		result.position = last_position_;
		result.usable = have_orientation_ and have_position_;
		result.clean = false;
		return result;
	}

	const bool orientation_valid = (pose.flags & flags::orientation_valid) != 0;
	const bool position_valid = (pose.flags & flags::position_valid) != 0;

	const bool take_orientation = update_tracked_state(orientation_state_,
	                                                   production_timestamp,
	                                                   orientation_valid,
	                                                   (pose.flags & flags::orientation_tracked) != 0,
	                                                   freeze);
	const bool take_position = update_tracked_state(position_state_,
	                                                production_timestamp,
	                                                position_valid,
	                                                (pose.flags & flags::position_tracked) != 0,
	                                                freeze);

	bool frozen_any = false;

	if (orientation_valid and take_orientation)
	{
		last_orientation_ = pose.pose.orientation;
		normalize_orientation(last_orientation_);
		have_orientation_ = true;
	}
	else if (orientation_valid)
	{
		frozen_any = true;
	}

	if (position_valid and take_position)
	{
		last_position_ = pose.pose.position;
		have_position_ = true;
	}
	else if (position_valid)
	{
		frozen_any = true;
	}

	result.orientation = last_orientation_;
	result.position = last_position_;
	result.usable = have_orientation_ and have_position_;

	// Velocities are only claimed when this very sample supplied them and
	// nothing about it was frozen. A frozen pose that still reported motion
	// would make the runtime extrapolate away from the pose we just chose to
	// hold, which is the exact drift the freeze exists to stop.
	const bool moving = not frozen_any and take_orientation and take_position;
	if (moving and (pose.flags & flags::linear_velocity_valid))
		result.linear_velocity = pose.linear_velocity;
	if (moving and (pose.flags & flags::angular_velocity_valid))
		result.angular_velocity = pose.angular_velocity;

	if (frozen_any)
		++frozen_;

	result.clean = not frozen_any and orientation_valid and position_valid;
	return result;
}

} // namespace wivrnnx::helper
