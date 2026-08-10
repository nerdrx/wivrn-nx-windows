// The real video encoder: the helper's D3D11 device plus two AMF components, one
// per eye.
//
// Two, not one, because the WiVRn wire protocol has no side-by-side stream. The
// headset creates one decoder per stream item and does not start rendering until
// items 0 and 1 have both produced a frame with a common frame index
// (client/scenes/stream.cpp:651 and :872), and the description it sizes those
// decoders from is per eye (common/wivrn_packets.h:1082-1085). The shim hands us
// one flattened side-by-side texture, so each eye is a crop out of it.
#pragma once

#include <cstdint>
#include <vector>

#include "amf_encoder.h"
#include "d3d11_stage.h"
#include "video_encoder.h"

namespace wivrnnx::helper
{

class AmfVideoEncoder final : public IVideoEncoder
{
public:
	AmfVideoEncoder() = default;
	~AmfVideoEncoder() override;

	bool configure(const ipc::StagingConfig & staging,
	               uint32_t vrserver_pid,
	               const EncoderConfig & config) override;
	void shutdown() override;

	EncoderStreamInfo stream_info() const override
	{
		return info_;
	}

	EncodeResult encode(const ipc::FrameReady & frame,
	                    bool force_idr,
	                    std::vector<EncodedFrame> & out) override;

private:
	static constexpr uint32_t kStreamCount = 2;
	// The shim releases the slot with key 1 at Present; if it has not by now, the
	// frame is not coming and the slot is handed straight back.
	static constexpr uint32_t kAcquireTimeoutMs = 100;

	D3D11Stage stage_;
	AmfContext context_;
	AmfStreamEncoder encoders_[kStreamCount];
	EncoderStreamInfo info_{};
	bool ready_ = false;
	uint64_t frames_encoded_ = 0;
	uint64_t frames_dropped_ = 0;
};

} // namespace wivrnnx::helper
