// A fake encoder, for the loopback harness only.
//
// The transport stack this phase grafted on — the bitrate controller, the shard
// pacer and the parity builder — sits between the encoder and the socket, and
// none of it needs a GPU to be exercised. What it does need is a stream of
// plausibly sized frames arriving at the refresh rate, which on a real session
// comes out of AMF and cannot be produced under Wine at all.
//
// So --synthetic-video puts frames into the same VideoBridge the AMF intake
// writes to: two eye frames per refresh period, each carrying half of whatever
// bitrate the controller currently asks for, with an IDR whenever one is
// requested. Everything downstream of the bridge — the pacing, the parity, the
// shard boundaries, the feedback loop — is then the production path, byte for
// byte, and src/tests/fake_client.cpp measures it over a real socket.
//
// Never enabled by accident: the flag has no effect unless a headset is
// connected, and the AMF intake is not started when it is in force.
#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include "../video_bridge.h"

namespace wivrnnx::helper
{

class SyntheticVideo
{
public:
	SyntheticVideo(VideoBridge & bridge, uint32_t eye_width, uint32_t eye_height);
	~SyntheticVideo();

	SyntheticVideo(const SyntheticVideo &) = delete;
	SyntheticVideo & operator=(const SyntheticVideo &) = delete;

	void start();
	void stop();

private:
	void run();

	VideoBridge & bridge_;
	uint32_t width_;
	uint32_t height_;
	std::atomic<bool> stop_{false};
	std::thread thread_;
};

} // namespace wivrnnx::helper
