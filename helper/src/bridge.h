// The seam between the WiVRn session and the shim pipe.
//
// Two producers write here — the WiVRn network thread in the default mode, the
// synthetic tracker thread under --fake — and one consumer (PipeServer) reads.
// Nothing else is shared between the two halves of the helper, which is what
// lets pipe_server.cpp stay a plain Win32 translation unit: this header pulls in
// only <cstdint>, the standard library and the frozen ipc/ contract, never
// winsock2.h and never the POSIX shadow headers that come with
// wivrn-common-net.
//
// Everything is "latest value wins", per device. A pose that was superseded
// before the pipe got round to it is of no use to anyone: SteamVR wants the
// newest sample, not a backlog. Haptics are the one exception and are queued,
// because a missed pulse is a missed pulse.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

#include "wivrnnx_ipc.h"

namespace wivrnnx::helper
{

// DeviceId is dense from 0; the arrays below are indexed by it.
inline constexpr size_t kDeviceCount = 3;

inline constexpr size_t device_index(ipc::DeviceId d)
{
	return static_cast<size_t>(d);
}

// One framed message waiting for the pipe. Sized for the largest payload the
// contract defines so it can live in a vector with no allocation per message.
struct Outgoing
{
	ipc::MessageType type;
	uint32_t size;
	unsigned char payload[sizeof(ipc::PoseUpdate)];
};

static_assert(sizeof(ipc::PoseUpdate) >= sizeof(ipc::HmdConfig));
static_assert(sizeof(ipc::PoseUpdate) >= sizeof(ipc::InputUpdate));

class Bridge
{
public:
	// What one consumer has already seen. The pipe server keeps one of these per
	// connected shim and throws it away on disconnect, which is what makes the
	// config (and every device's presence) be re-sent on reconnect — protocol v2
	// says the handshake, HmdConfig included, runs again every time.
	struct Cursor
	{
		uint64_t config = 0;
		std::array<uint64_t, kDeviceCount> pose{};
		std::array<uint64_t, kDeviceCount> input{};
		std::array<uint64_t, kDeviceCount> presence{};
	};

	explicit Bridge(const ipc::HmdConfig & initial_config);

	// --- producer side ----------------------------------------------------

	// Replaces the advertised display configuration. Cheap to call with an
	// unchanged value: an identical config does not bump the generation, so a
	// connected shim is not spammed with redundant refreshes.
	void set_config(const ipc::HmdConfig & config);

	void set_pose(const ipc::PoseUpdate & pose);
	void set_input(const ipc::InputUpdate & input);

	// Device arrival/departure. Idempotent; the generation only moves on a real
	// change, so a shim sees one DeviceAdd per arrival.
	void set_present(ipc::DeviceId device, bool present);

	// The client went away: every device is marked absent and the last poses are
	// flagged disconnected, so the shim stops claiming tracked poses rather than
	// freezing on the last sample it got.
	void on_client_gone();

	// --- consumer side ----------------------------------------------------

	// Appends every message the cursor has not seen yet and advances it.
	// Ordering within one call is deliberate: config, then presence, then poses,
	// then inputs — the shim may not register a device before its DeviceAdd, and
	// may not register the HMD before HmdConfig.
	void collect(Cursor & cursor, std::vector<Outgoing> & out) const;

	// --- haptics (shim -> client), the one queued channel ------------------

	void push_haptic(const ipc::Haptic & haptic);
	bool pop_haptic(ipc::Haptic & out);

	// --- counters, for the heartbeat log ----------------------------------

	uint64_t poses_received() const;

private:
	mutable std::mutex mutex_;

	ipc::HmdConfig config_{};
	uint64_t config_gen_ = 1;

	std::array<ipc::PoseUpdate, kDeviceCount> pose_{};
	std::array<uint64_t, kDeviceCount> pose_gen_{};

	std::array<ipc::InputUpdate, kDeviceCount> input_{};
	std::array<uint64_t, kDeviceCount> input_gen_{};

	std::array<bool, kDeviceCount> present_{};
	std::array<uint64_t, kDeviceCount> presence_gen_{};

	// Bounded: a shim that stops reading must not grow this without limit. Ten
	// pulses is far more than the ~1 in flight a working session ever has.
	static constexpr size_t kMaxHaptics = 10;
	std::deque<ipc::Haptic> haptics_;

	uint64_t poses_received_ = 0;
};

} // namespace wivrnnx::helper
