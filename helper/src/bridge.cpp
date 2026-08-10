#include "bridge.h"

#include <cstring>

namespace wivrnnx::helper
{

namespace
{

Outgoing frame(ipc::MessageType type, const void * payload, size_t size)
{
	Outgoing msg{};
	msg.type = type;
	msg.size = static_cast<uint32_t>(size);
	std::memcpy(msg.payload, payload, size);
	return msg;
}

} // namespace

Bridge::Bridge(const ipc::HmdConfig & initial_config) :
        config_(initial_config)
{
	for (size_t i = 0; i < kDeviceCount; ++i)
	{
		pose_[i].device = static_cast<uint8_t>(i);
		pose_[i].connected = 0;
		pose_[i].qw = 1.0;
		input_[i].device = static_cast<uint8_t>(i);
	}
}

void Bridge::set_config(const ipc::HmdConfig & config)
{
	std::lock_guard lock(mutex_);
	if (std::memcmp(&config_, &config, sizeof(config)) == 0)
		return;
	config_ = config;
	++config_gen_;
}

void Bridge::set_pose(const ipc::PoseUpdate & pose)
{
	const size_t i = pose.device;
	if (i >= kDeviceCount)
		return;

	std::lock_guard lock(mutex_);
	pose_[i] = pose;
	++pose_gen_[i];
	++poses_received_;
}

void Bridge::set_input(const ipc::InputUpdate & input)
{
	const size_t i = input.device;
	if (i >= kDeviceCount)
		return;

	std::lock_guard lock(mutex_);
	input_[i] = input;
	++input_gen_[i];
}

void Bridge::set_present(ipc::DeviceId device, bool present)
{
	const size_t i = device_index(device);
	if (i >= kDeviceCount)
		return;

	std::lock_guard lock(mutex_);
	if (present_[i] == present)
		return;
	present_[i] = present;
	++presence_gen_[i];
}

void Bridge::on_client_gone()
{
	std::lock_guard lock(mutex_);
	for (size_t i = 0; i < kDeviceCount; ++i)
	{
		if (present_[i])
		{
			present_[i] = false;
			++presence_gen_[i];
		}
		if (pose_[i].connected)
		{
			pose_[i].connected = 0;
			++pose_gen_[i];
		}
	}
	haptics_.clear();
}

void Bridge::collect(Cursor & cursor, std::vector<Outgoing> & out) const
{
	std::lock_guard lock(mutex_);

	if (cursor.config != config_gen_)
	{
		cursor.config = config_gen_;
		out.push_back(frame(ipc::MessageType::hmd_config, &config_, sizeof(config_)));
	}

	// The HMD is implicit in HmdConfig and never announced with DeviceAdd, which
	// is what the frozen header says; only the controllers are.
	for (size_t i = 1; i < kDeviceCount; ++i)
	{
		if (cursor.presence[i] == presence_gen_[i])
			continue;
		cursor.presence[i] = presence_gen_[i];

		if (present_[i])
		{
			ipc::DeviceAdd add{};
			add.device = static_cast<uint8_t>(i);
			out.push_back(frame(ipc::MessageType::device_add, &add, sizeof(add)));
		}
		else
		{
			ipc::DeviceRemove rm{};
			rm.device = static_cast<uint8_t>(i);
			out.push_back(frame(ipc::MessageType::device_remove, &rm, sizeof(rm)));
		}
	}

	for (size_t i = 0; i < kDeviceCount; ++i)
	{
		if (cursor.pose[i] == pose_gen_[i])
			continue;
		cursor.pose[i] = pose_gen_[i];
		out.push_back(frame(ipc::MessageType::pose_update, &pose_[i], sizeof(pose_[i])));
	}

	for (size_t i = 0; i < kDeviceCount; ++i)
	{
		if (cursor.input[i] == input_gen_[i])
			continue;
		cursor.input[i] = input_gen_[i];
		out.push_back(frame(ipc::MessageType::input_update, &input_[i], sizeof(input_[i])));
	}
}

void Bridge::push_haptic(const ipc::Haptic & haptic)
{
	std::lock_guard lock(mutex_);
	if (haptics_.size() >= kMaxHaptics)
		haptics_.pop_front();
	haptics_.push_back(haptic);
}

bool Bridge::pop_haptic(ipc::Haptic & out)
{
	std::lock_guard lock(mutex_);
	if (haptics_.empty())
		return false;
	out = haptics_.front();
	haptics_.pop_front();
	return true;
}

uint64_t Bridge::poses_received() const
{
	std::lock_guard lock(mutex_);
	return poses_received_;
}

} // namespace wivrnnx::helper
