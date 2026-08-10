// One AMF encode component: the AMF half of the video path, with no D3D11 in it.
//
// The split is deliberate. Everything below needs an AMF runtime and nothing
// else — the D3D11 device is passed through to AMFContext::InitDX11 as an opaque
// pointer and the per-frame pixel copy is a callback the caller supplies. That
// makes the whole AMF ABI (loader, context, component creation, property set,
// submit/poll loop, IDR forcing) drivable against the stub runtime in
// src/tests/amf_stub.cpp, which is the only place any of it can be exercised
// short of the RX 580.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "video_encoder.h"

namespace amf
{
class AMFContext;
class AMFComponent;
} // namespace amf

namespace wivrnnx::helper
{

// The AMF context, one per D3D11 device, shared by both eye encoders.
class AmfContext
{
public:
	~AmfContext();

	AmfContext() = default;
	AmfContext(const AmfContext &) = delete;
	AmfContext & operator=(const AmfContext &) = delete;

	// `d3d11_device` is an ID3D11Device *, kept opaque here. May be null, which
	// is only ever the case under the ABI stub.
	bool init(void * d3d11_device);
	void terminate();

	amf::AMFContext * get() const
	{
		return context_;
	}

private:
	amf::AMFContext * context_ = nullptr;
};

struct AmfEncodeParams
{
	uint32_t width = 0;
	uint32_t height = 0;
	float refresh_hz = 90.f;
	uint32_t bitrate_bps = 50'000'000;
	// AMF_SURFACE_FORMAT of the surfaces this encoder is fed, as an int so the
	// AMF headers stay out of this one. See amf_surface_format_for_dxgi().
	int surface_format = 0;
	// Try HEVC first; H.264 is the fallback when the HEVC component cannot be
	// created (VCE 3.4 on Polaris does have HEVC, but a driver that disagrees
	// must not cost the whole video path).
	bool allow_h265 = true;
	bool allow_h264 = true;
	// How long one frame may sit in the encoder before it is given up on.
	uint32_t poll_timeout_ms = 40;
};

// AMF_SURFACE_FORMAT for a DXGI_FORMAT, or 0 (AMF_SURFACE_UNKNOWN) if the
// staging textures are in something this phase cannot encode.
int amf_surface_format_for_dxgi(uint32_t dxgi_format);
const char * amf_surface_format_name(int surface_format);

class AmfStreamEncoder
{
public:
	~AmfStreamEncoder();

	AmfStreamEncoder() = default;
	AmfStreamEncoder(const AmfStreamEncoder &) = delete;
	AmfStreamEncoder & operator=(const AmfStreamEncoder &) = delete;

	// Creates the component and applies the whole property set. `label` only
	// shows up in the log ("left"/"right").
	bool open(AmfContext & context, const AmfEncodeParams & params, const char * label);
	void close();

	bool is_open() const
	{
		return component_ != nullptr;
	}

	VideoCodec codec() const
	{
		return codec_;
	}

	// A new target for the running component, bits per second for this one eye.
	//
	// The three properties are the ones the automatic bitrate has to move together:
	// TARGET_BITRATE and PEAK_BITRATE (equal, this is CBR) and VBV_BUFFER_SIZE,
	// which is sized from the bitrate and would otherwise still describe the old
	// one — a VBV left at the old number is either an encoder that cannot use the
	// bandwidth it was given or one that is allowed to spend several frames paying
	// back a spike, which is the latency this path cannot afford.
	//
	// AMF takes all three on a live component: they are AMF_PROPERTY_ACCESS_FULL
	// on the encoder (see components/VideoEncoderHEVC.h and VideoEncoderVCE.h in
	// external/amf), so no Terminate/Init is needed and no parameter set is
	// re-emitted, which is the whole point — a rebuild would cost an IDR every
	// time the controller moved.
	void set_bitrate(uint32_t bitrate_bps);

	// Allocates an input surface, hands its native DX11 texture to `fill`, and
	// runs it through the encoder. `fill` returns false to abandon the frame
	// (the surface is dropped and nothing is submitted).
	//
	// Appends the annex-B bitstream to `out` and sets `out_idr`. Returns
	// EncodeResult::dropped when the encoder produced nothing within
	// poll_timeout_ms, and ::fatal when the component has to be rebuilt.
	using FillFn = std::function<bool(void * native_texture)>;
	EncodeResult encode(const FillFn & fill, bool force_idr, std::vector<uint8_t> & out, bool & out_idr);

	// The two halves of encode(), split so that the caller can submit both eyes
	// before waiting on either. That matters: the staging slot's keyed mutex is
	// held across the submits and nothing else, rather than across two whole
	// encodes, so the shim gets the slot back in microseconds instead of
	// milliseconds.
	EncodeResult submit(const FillFn & fill, bool force_idr);
	EncodeResult retrieve(std::vector<uint8_t> & out, bool & out_idr);

	// Submit-to-output latency of the last frame, milliseconds. Log only.
	double last_encode_ms() const
	{
		return last_encode_ms_;
	}

private:
	bool create_component(AmfContext & context, const AmfEncodeParams & params, VideoCodec codec);
	bool configure_h265(const AmfEncodeParams & params);
	bool configure_h264(const AmfEncodeParams & params);
	void apply_frame_properties(void * surface, bool force_idr);
	bool output_is_idr(void * data) const;

	amf::AMFContext * context_ = nullptr; // not owned
	amf::AMFComponent * component_ = nullptr;
	VideoCodec codec_ = VideoCodec::h265;
	AmfEncodeParams params_{};
	bool has_query_timeout_ = false;
	double last_encode_ms_ = 0.0;
	// QPC ticks at the last submit(), so retrieve() can bound its own wait.
	int64_t submitted_at_ = 0;
	const char * label_ = "";
};

} // namespace wivrnnx::helper
