#include "amf_encoder.h"

#include <windows.h>

#include <dxgiformat.h>

#include <cstdio>

#include "components/VideoEncoderHEVC.h"
#include "components/VideoEncoderVCE.h"
#include "core/Factory.h"

#include "../log.h"
#include "amf_loader.h"

namespace wivrnnx::helper
{

namespace
{

// Carried on the input surface and read back off the output buffer, so an
// asynchronous encoder's output can be matched to the frame that produced it.
// Same trick, same purpose as ALVR's START_TIME_PROPERTY / FRAME_INDEX_PROPERTY
// (reference/alvr/alvr/server_openvr/cpp/platform/win32/VideoEncoderAMF.cpp:15).
constexpr const wchar_t * kSubmitTicksProperty = L"WivrnnxSubmitTicks";

int64_t qpc_now()
{
	LARGE_INTEGER t{};
	QueryPerformanceCounter(&t);
	return t.QuadPart;
}

double qpc_to_ms(int64_t ticks)
{
	static const double scale = [] {
		LARGE_INTEGER f{};
		QueryPerformanceFrequency(&f);
		return f.QuadPart > 0 ? 1000.0 / static_cast<double>(f.QuadPart) : 0.0;
	}();
	return static_cast<double>(ticks) * scale;
}

} // namespace

const char * video_codec_name(VideoCodec codec)
{
	return codec == VideoCodec::h265 ? "HEVC" : "H.264";
}

IVideoEncoder::~IVideoEncoder() = default;

int amf_surface_format_for_dxgi(uint32_t dxgi_format)
{
	// Only the two 8-bit packed layouts SteamVR actually submits for a colour
	// swapchain. An sRGB view of one of them is the same memory: the encoder is
	// told full-range BT.709 either way and the client's sampler is hardwired to
	// that (client/decoder/android/android_decoder.cpp:368-369), so the gamma
	// difference between the two spellings is not something this layer can act
	// on. CopySubresourceRegion accepts the pair because they share a typeless
	// parent, which is the only thing that has to hold here.
	switch (static_cast<DXGI_FORMAT>(dxgi_format))
	{
		case DXGI_FORMAT_R8G8B8A8_UNORM:
		case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
		case DXGI_FORMAT_R8G8B8A8_TYPELESS:
			return amf::AMF_SURFACE_RGBA;
		case DXGI_FORMAT_B8G8R8A8_UNORM:
		case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
		case DXGI_FORMAT_B8G8R8A8_TYPELESS:
			return amf::AMF_SURFACE_BGRA;
		default:
			return amf::AMF_SURFACE_UNKNOWN;
	}
}

const char * amf_surface_format_name(int surface_format)
{
	switch (static_cast<amf::AMF_SURFACE_FORMAT>(surface_format))
	{
		case amf::AMF_SURFACE_RGBA:
			return "RGBA";
		case amf::AMF_SURFACE_BGRA:
			return "BGRA";
		case amf::AMF_SURFACE_NV12:
			return "NV12";
		default:
			return "unknown";
	}
}

// ---------------------------------------------------------------------------
// AmfContext
// ---------------------------------------------------------------------------

AmfContext::~AmfContext()
{
	terminate();
}

bool AmfContext::init(void * d3d11_device)
{
	AmfLoader & loader = AmfLoader::instance();
	if (not loader.load())
		return false;

	amf::AMFContext * context = nullptr;
	AMF_RESULT res = loader.factory()->CreateContext(&context);
	if (res != AMF_OK || context == nullptr)
	{
		log_line("AMF: CreateContext failed (%s)", amf_result_name(res));
		return false;
	}

	// The helper's own device, not vrserver's: the staging textures are opened
	// on it with OpenSharedResource1 and everything the encoder touches lives
	// there. Sharing vrserver's device would mean handing AMF a device another
	// process drives, which is exactly what the shared-texture handshake exists
	// to avoid.
	res = context->InitDX11(d3d11_device);
	if (res != AMF_OK)
	{
		log_line("AMF: InitDX11 failed (%s)", amf_result_name(res));
		context->Terminate();
		context->Release();
		return false;
	}

	context_ = context;
	return true;
}

void AmfContext::terminate()
{
	if (context_ == nullptr)
		return;
	context_->Terminate();
	context_->Release();
	context_ = nullptr;
}

// ---------------------------------------------------------------------------
// AmfStreamEncoder
// ---------------------------------------------------------------------------

AmfStreamEncoder::~AmfStreamEncoder()
{
	close();
}

bool AmfStreamEncoder::open(AmfContext & context, const AmfEncodeParams & params, const char * label)
{
	close();

	if (context.get() == nullptr)
		return false;

	context_ = context.get();
	params_ = params;
	label_ = label;

	// HEVC first: VCE 3.4 on Polaris encodes it, it costs about 20% fewer bits
	// than H.264 at the same quality, and the client asks for it first on every
	// Pico. H.264 is only there so that a driver that refuses to build the HEVC
	// component does not take the whole video path with it.
	if (params.allow_h265 && create_component(context, params, VideoCodec::h265))
		return true;

	if (params.allow_h264 && create_component(context, params, VideoCodec::h264))
	{
		log_line("AMF[%s]: HEVC unavailable, fell back to H.264", label_);
		return true;
	}

	log_line("AMF[%s]: no encoder component could be created", label_);
	context_ = nullptr;
	return false;
}

bool AmfStreamEncoder::create_component(AmfContext & context, const AmfEncodeParams & params, VideoCodec codec)
{
	const wchar_t * id = codec == VideoCodec::h265 ? AMFVideoEncoder_HEVC : AMFVideoEncoderVCE_AVC;

	if (create_component_once(context, params, codec, id, true))
		return true;

	// The Polaris legacy runtime (amfrt64 1.4.31, RX 580) takes the AVC
	// ULTRA_LOW_LATENCY usage preset at SetProperty time and then answers
	// everything after it — further SetProperty, GetCaps, Init — with
	// AMF_ACCESS_DENIED. Same context, same dimensions: HEVC's
	// ULTRA_LOW_LATENCY preset is fine, and so is every other AVC usage. So
	// one retry with LOW_LATENCY, which that runtime accepts and which is
	// still a one-in-one-out low-latency mode; drivers where ULTRA works
	// never reach this. Pointless when the component itself could not be
	// created — a second CreateComponent would only fail the same way.
	if (codec == VideoCodec::h264 && component_was_created_ &&
	    create_component_once(context, params, codec, id, false))
	{
		log_line("AMF[%s]: this runtime refuses the H.264 ULTRA_LOW_LATENCY usage, using LOW_LATENCY",
		         label_);
		return true;
	}
	return false;
}

bool AmfStreamEncoder::create_component_once(AmfContext & context,
                                             const AmfEncodeParams & params,
                                             VideoCodec codec,
                                             const wchar_t * id,
                                             bool ultra_low_latency)
{
	component_was_created_ = false;

	amf::AMFComponent * component = nullptr;
	AMF_RESULT res = AmfLoader::instance().factory()->CreateComponent(context.get(), id, &component);
	if (res != AMF_OK || component == nullptr)
	{
		log_line("AMF[%s]: CreateComponent(%s) failed (%s)",
		         label_,
		         video_codec_name(codec),
		         amf_result_name(res));
		return false;
	}
	component_was_created_ = true;

	component_ = component;
	codec_ = codec;
	has_query_timeout_ = false;

	const bool configured = codec == VideoCodec::h265 ? configure_h265(params)
	                                                  : configure_h264(params, ultra_low_latency);
	if (not configured)
	{
		component_->Terminate();
		component_->Release();
		component_ = nullptr;
		return false;
	}

	res = component_->Init(static_cast<amf::AMF_SURFACE_FORMAT>(params.surface_format),
	                       static_cast<amf_int32>(params.width),
	                       static_cast<amf_int32>(params.height));
	if (res != AMF_OK)
	{
		log_line("AMF[%s]: %s Init(%s, %ux%u) failed (%s)",
		         label_,
		         video_codec_name(codec),
		         amf_surface_format_name(params.surface_format),
		         params.width,
		         params.height,
		         amf_result_name(res));
		component_->Terminate();
		component_->Release();
		component_ = nullptr;
		return false;
	}

	log_line("AMF[%s]: %s %ux%u @ %.1f Hz, CBR %u kbit/s, %s in, query timeout %s",
	         label_,
	         video_codec_name(codec),
	         params.width,
	         params.height,
	         static_cast<double>(params.refresh_hz),
	         params.bitrate_bps / 1000,
	         amf_surface_format_name(params.surface_format),
	         has_query_timeout_ ? "supported" : "unsupported");
	return true;
}

// The property set. Everything here is either named in the task (50 Mbit CBR,
// low latency, no B pictures, IDR on demand) or copied from ALVR's
// VideoEncoderAMF.cpp, which is the reference for what a Radeon actually accepts
// — line numbers below are into
// reference/alvr/alvr/server_openvr/cpp/platform/win32/VideoEncoderAMF.cpp.
//
// Not copied from it: ALVR's switch falls through between the three codec cases
// (there is no break after the H.264 block at :306 or the HEVC one at :470), so
// an H.264 encoder there is also handed every HEVC and AV1 property. That is
// harmless only because SetProperty on an unknown name returns an error nobody
// checks. Here the two configurations are separate functions.
// Every configure-time SetProperty goes through SET below, which logs the
// per-call result when it is not AMF_OK. SetProperty rejections are otherwise
// silent — the component happily Init()s without the property, or refuses to
// Init at all, and nothing says which call was the problem. On the Polaris
// legacy driver that difference is live: its AVC component refused Init with
// AMF_ACCESS_DENIED while every SetProperty looked fine.
#define SET(name, ...) \
	log_set_result(#name, e->SetProperty(name, __VA_ARGS__), rejected)

bool AmfStreamEncoder::configure_h265(const AmfEncodeParams & params)
{
	amf::AMFComponent * e = component_;
	const amf_int64 bitrate = params.bitrate_bps;
	const amf_int32 fps = static_cast<amf_int32>(params.refresh_hz > 1.f ? params.refresh_hz : 90.f);
	int rejected = 0;

	// :308 - the usage preset is what puts the encoder in one-in-one-out mode
	// with no lookahead, which is the whole reason this is usable for VR.
	SET(AMF_VIDEO_ENCODER_HEVC_USAGE, AMF_VIDEO_ENCODER_HEVC_USAGE_ULTRA_LOW_LATENCY);

	// :313 - CBR. Filler data (:318) is deliberately *not* enabled: it pads every
	// frame out to the full bitrate budget, which on a Wi-Fi link is bandwidth
	// spent on nothing. WiVRn's own encoders run CBR without it.
	SET(AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_METHOD,
	    AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_METHOD_CBR);
	SET(AMF_VIDEO_ENCODER_HEVC_FILLER_DATA_ENABLE, false);

	// :329
	SET(AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE, bitrate);
	SET(AMF_VIDEO_ENCODER_HEVC_PEAK_BITRATE, bitrate);
	SET(AMF_VIDEO_ENCODER_HEVC_FRAMESIZE,
	    ::AMFConstructSize(static_cast<amf_int32>(params.width),
	                       static_cast<amf_int32>(params.height)));
	SET(AMF_VIDEO_ENCODER_HEVC_FRAMERATE, ::AMFConstructRate(fps, 1));

	// :352 - speed over quality. A Polaris VCE has to encode two eye images per
	// frame here; anything else is not going to hold 72 Hz.
	SET(AMF_VIDEO_ENCODER_HEVC_QUALITY_PRESET,
	    AMF_VIDEO_ENCODER_HEVC_QUALITY_PRESET_SPEED);

	// :364
	SET(AMF_VIDEO_ENCODER_HEVC_COLOR_BIT_DEPTH, AMF_COLOR_BIT_DEPTH_8);
	SET(AMF_VIDEO_ENCODER_HEVC_PROFILE, AMF_VIDEO_ENCODER_HEVC_PROFILE_MAIN);

	// :396 and :427 - full-range BT.709. Not a preference: the client builds its
	// YCbCr sampler with eItuFull and eYcbcr709 hardcoded, overriding whatever
	// the decoder suggests (client/decoder/android/android_decoder.cpp:368-369),
	// and the Linux encoders match that (server/encoder/ffmpeg/video_encoder_va.cpp:285
	// sets AVCOL_RANGE_JPEG). Anything else here is a washed-out picture.
	SET(AMF_VIDEO_ENCODER_HEVC_NOMINAL_RANGE, AMF_VIDEO_ENCODER_HEVC_NOMINAL_RANGE_FULL);
	SET(AMF_VIDEO_ENCODER_HEVC_OUTPUT_COLOR_PROFILE,
	    AMF_VIDEO_CONVERTER_COLOR_PROFILE_FULL_709);
	SET(AMF_VIDEO_ENCODER_HEVC_OUTPUT_TRANSFER_CHARACTERISTIC,
	    AMF_COLOR_TRANSFER_CHARACTERISTIC_GAMMA22);
	SET(AMF_VIDEO_ENCODER_HEVC_OUTPUT_COLOR_PRIMARIES, AMF_COLOR_PRIMARIES_BT709);

	// :453 - infinite GOP: no periodic IDR at all. Every IDR this stream ever
	// carries is one the idr tracker asked for, either because a client just
	// connected or because the headset lost a frame.
	SET(AMF_VIDEO_ENCODER_HEVC_NUM_GOPS_PER_IDR, 0);
	SET(AMF_VIDEO_ENCODER_HEVC_GOP_SIZE, 0);

	// :459 - no access unit delimiters. The client's MediaCodec does not need
	// them and they are bytes on the wire.
	SET(AMF_VIDEO_ENCODER_HEVC_INSERT_AUD, false);

	// :461 - a VBV of a little over one frame's worth of bits. Larger buffers
	// let the encoder spend several frames paying back a spike, which is exactly
	// the latency this path cannot afford.
	SET(AMF_VIDEO_ENCODER_HEVC_VBV_BUFFER_SIZE,
	    static_cast<amf_int64>(static_cast<double>(bitrate) / fps * 1.1));

	// :465 - one reference frame, so a P frame only ever depends on the frame
	// before it. With no FEC yet, that is what keeps a single lost frame from
	// poisoning an arbitrarily long chain. Legacy runtimes (Polaris) call 0
	// OUT_OF_RANGE; there the explicit 1 says the same thing in their dialect.
	if (e->SetProperty(AMF_VIDEO_ENCODER_HEVC_MAX_NUM_REFRAMES, 0) != AMF_OK)
		SET(AMF_VIDEO_ENCODER_HEVC_MAX_NUM_REFRAMES, 1);

	// :370 - QueryOutput can block instead of being spun on, if the runtime
	// says so.
	amf::AMFCaps * caps = nullptr;
	const AMF_RESULT caps_res = e->GetCaps(&caps);
	if (caps_res == AMF_OK && caps != nullptr)
	{
		amf_bool supported = false;
		if (caps->GetProperty(AMF_VIDEO_ENCODER_CAPS_HEVC_QUERY_TIMEOUT_SUPPORT, &supported) == AMF_OK)
			has_query_timeout_ = supported;
		caps->Release();
	}
	else
		log_line("AMF[%s]: %s GetCaps failed (%s)", label_, video_codec_name(codec_), amf_result_name(caps_res));
	if (has_query_timeout_)
		SET(AMF_VIDEO_ENCODER_HEVC_QUERY_TIMEOUT,
		    static_cast<amf_int64>(params.poll_timeout_ms));

	if (rejected != 0)
		log_line("AMF[%s]: %s %d configure propert%s rejected (see above)",
		         label_,
		         video_codec_name(codec_),
		         rejected,
		         rejected == 1 ? "y" : "ies");
	return true;
}

bool AmfStreamEncoder::configure_h264(const AmfEncodeParams & params, bool ultra_low_latency)
{
	amf::AMFComponent * e = component_;
	const amf_int64 bitrate = params.bitrate_bps;
	const amf_int32 fps = static_cast<amf_int32>(params.refresh_hz > 1.f ? params.refresh_hz : 90.f);
	int rejected = 0;

	// :154 - ULTRA_LOW_LATENCY, except on the retry create_component makes for
	// runtimes whose AVC component is poisoned by it (see there).
	SET(AMF_VIDEO_ENCODER_USAGE,
	    ultra_low_latency ? AMF_VIDEO_ENCODER_USAGE_ULTRA_LOW_LATENCY
	                      : AMF_VIDEO_ENCODER_USAGE_LOW_LATENCY);
	// :163 - High profile, level 4.2 as ALVR does at :167. Every Pico decodes it.
	SET(AMF_VIDEO_ENCODER_PROFILE, AMF_VIDEO_ENCODER_PROFILE_HIGH);
	SET(AMF_VIDEO_ENCODER_PROFILE_LEVEL, 42);

	// :170, filler off for the same reason as HEVC above.
	SET(AMF_VIDEO_ENCODER_RATE_CONTROL_METHOD, AMF_VIDEO_ENCODER_RATE_CONTROL_METHOD_CBR);
	SET(AMF_VIDEO_ENCODER_FILLER_DATA_ENABLE, false);
	// :188 - CABAC; it is worth a few percent and every decoder here has it.
	SET(AMF_VIDEO_ENCODER_CABAC_ENABLE, AMF_VIDEO_ENCODER_CABAC);

	// :195
	SET(AMF_VIDEO_ENCODER_TARGET_BITRATE, bitrate);
	SET(AMF_VIDEO_ENCODER_PEAK_BITRATE, bitrate);
	SET(AMF_VIDEO_ENCODER_FRAMESIZE,
	    ::AMFConstructSize(static_cast<amf_int32>(params.width),
	                       static_cast<amf_int32>(params.height)));
	SET(AMF_VIDEO_ENCODER_FRAMERATE, ::AMFConstructRate(fps, 1));
	// :199 - no B pictures. They reorder, and reordering is latency.
	SET(AMF_VIDEO_ENCODER_B_PIC_PATTERN, 0);

	// :214
	SET(AMF_VIDEO_ENCODER_QUALITY_PRESET, AMF_VIDEO_ENCODER_QUALITY_PRESET_SPEED);

	// :244 and :271 - full-range BT.709, see configure_h265.
	SET(AMF_VIDEO_ENCODER_FULL_RANGE_COLOR, true);
	SET(AMF_VIDEO_ENCODER_OUTPUT_COLOR_PROFILE, AMF_VIDEO_CONVERTER_COLOR_PROFILE_FULL_709);
	SET(AMF_VIDEO_ENCODER_OUTPUT_TRANSFER_CHARACTERISTIC,
	    AMF_COLOR_TRANSFER_CHARACTERISTIC_GAMMA22);
	SET(AMF_VIDEO_ENCODER_OUTPUT_COLOR_PRIMARIES, AMF_COLOR_PRIMARIES_BT709);

	// :293 - no periodic IDR.
	SET(AMF_VIDEO_ENCODER_IDR_PERIOD, 0);
	// :297
	SET(AMF_VIDEO_ENCODER_INSERT_AUD, false);
	// :299
	SET(AMF_VIDEO_ENCODER_VBV_BUFFER_SIZE,
	    static_cast<amf_int64>(static_cast<double>(bitrate) / fps * 1.1));
	// :301 - one reference frame; 0 is OUT_OF_RANGE on legacy runtimes, where
	// the explicit 1 means the same. See the HEVC twin above.
	if (e->SetProperty(AMF_VIDEO_ENCODER_MAX_NUM_REFRAMES, 0) != AMF_OK)
		SET(AMF_VIDEO_ENCODER_MAX_NUM_REFRAMES, 1);

	// :220
	amf::AMFCaps * caps = nullptr;
	const AMF_RESULT caps_res = e->GetCaps(&caps);
	if (caps_res == AMF_OK && caps != nullptr)
	{
		amf_bool supported = false;
		if (caps->GetProperty(AMF_VIDEO_ENCODER_CAPS_QUERY_TIMEOUT_SUPPORT, &supported) == AMF_OK)
			has_query_timeout_ = supported;
		caps->Release();
	}
	else
		log_line("AMF[%s]: %s GetCaps failed (%s)", label_, video_codec_name(codec_), amf_result_name(caps_res));
	if (has_query_timeout_)
		SET(AMF_VIDEO_ENCODER_QUERY_TIMEOUT, static_cast<amf_int64>(params.poll_timeout_ms));

	if (rejected != 0)
		log_line("AMF[%s]: %s %d configure propert%s rejected (see above)",
		         label_,
		         video_codec_name(codec_),
		         rejected,
		         rejected == 1 ? "y" : "ies");
	return true;
}

#undef SET

void AmfStreamEncoder::log_set_result(const char * property, int result, int & rejected)
{
	if (result == AMF_OK)
		return;
	++rejected;
	log_line("AMF[%s]: %s SetProperty(%s) -> %s",
	         label_,
	         video_codec_name(codec_),
	         property,
	         amf_result_name(result));
}

void AmfStreamEncoder::close()
{
	if (component_ != nullptr)
	{
		component_->Drain();
		component_->Terminate();
		component_->Release();
		component_ = nullptr;
	}
	context_ = nullptr;
}

// VideoEncoderAMF.cpp:790 ApplyFrameProperties. The parameter sets have to be
// repeated on every IDR: the client may have started its decoder after the last
// one, and a MediaCodec with no VPS/SPS/PPS in front of the IDR produces nothing.
void AmfStreamEncoder::apply_frame_properties(void * surface_ptr, bool force_idr)
{
	auto * surface = static_cast<amf::AMFSurface *>(surface_ptr);

	if (codec_ == VideoCodec::h265)
	{
		surface->SetProperty(AMF_VIDEO_ENCODER_HEVC_INSERT_AUD, false);
		if (force_idr)
		{
			// :812 - one property for VPS+SPS+PPS on HEVC, unlike H.264.
			surface->SetProperty(AMF_VIDEO_ENCODER_HEVC_INSERT_HEADER, true);
			surface->SetProperty(AMF_VIDEO_ENCODER_HEVC_FORCE_PICTURE_TYPE,
			                     AMF_VIDEO_ENCODER_HEVC_PICTURE_TYPE_IDR);
		}
	}
	else
	{
		surface->SetProperty(AMF_VIDEO_ENCODER_INSERT_AUD, false);
		if (force_idr)
		{
			// :797
			surface->SetProperty(AMF_VIDEO_ENCODER_INSERT_SPS, true);
			surface->SetProperty(AMF_VIDEO_ENCODER_INSERT_PPS, true);
			surface->SetProperty(AMF_VIDEO_ENCODER_FORCE_PICTURE_TYPE,
			                     AMF_VIDEO_ENCODER_PICTURE_TYPE_IDR);
		}
	}
}

// VideoEncoderAMF.cpp:777.
bool AmfStreamEncoder::output_is_idr(void * data_ptr) const
{
	auto * data = static_cast<amf::AMFData *>(data_ptr);
	amf_int64 type = 0;

	if (codec_ == VideoCodec::h265)
	{
		if (data->GetProperty(AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE, &type) != AMF_OK)
			return false;
		return type == AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE_IDR;
	}

	if (data->GetProperty(AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE, &type) != AMF_OK)
		return false;
	return type == AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE_IDR;
}

EncodeResult AmfStreamEncoder::encode(const FillFn & fill,
                                      bool force_idr,
                                      std::vector<uint8_t> & out,
                                      bool & out_idr)
{
	const EncodeResult submitted = submit(fill, force_idr);
	if (submitted != EncodeResult::ok)
	{
		out_idr = false;
		return submitted;
	}
	return retrieve(out, out_idr);
}

EncodeResult AmfStreamEncoder::submit(const FillFn & fill, bool force_idr)
{
	if (component_ == nullptr || context_ == nullptr)
		return EncodeResult::fatal;

	// One AMF-allocated surface per frame rather than a surface created from the
	// staging texture. See the note in amf_video_encoder.cpp: the staging texture
	// is a side-by-side pair under a keyed mutex and each eye needs a crop out of
	// it, which no zero-copy wrapper can express.
	amf::AMFSurface * surface = nullptr;
	AMF_RESULT res = context_->AllocSurface(amf::AMF_MEMORY_DX11,
	                                        static_cast<amf::AMF_SURFACE_FORMAT>(params_.surface_format),
	                                        static_cast<amf_int32>(params_.width),
	                                        static_cast<amf_int32>(params_.height),
	                                        &surface);
	if (res != AMF_OK || surface == nullptr)
	{
		log_line("AMF[%s]: AllocSurface failed (%s)", label_, amf_result_name(res));
		return EncodeResult::fatal;
	}

	// GetNative() hands out the ID3D11Texture2D without a reference; it must not
	// be released. VideoEncoderAMF.cpp:747 says the same.
	amf::AMFPlane * plane = surface->GetPlaneAt(0);
	void * native = plane != nullptr ? plane->GetNative() : nullptr;

	if (not fill(native))
	{
		surface->Release();
		return EncodeResult::dropped;
	}

	submitted_at_ = qpc_now();
	surface->SetProperty(kSubmitTicksProperty, submitted_at_);
	apply_frame_properties(surface, force_idr);

	res = component_->SubmitInput(surface);
	surface->Release();

	if (res == AMF_INPUT_FULL)
	{
		// Nothing was consumed, and the queue is one frame deep in low-latency
		// mode, so this means the previous frame is still inside the encoder.
		// Give up on this one rather than queue behind it.
		log_line("AMF[%s]: input queue full, dropping frame", label_);
		return EncodeResult::dropped;
	}
	if (res != AMF_OK && res != AMF_NEED_MORE_INPUT)
	{
		log_line("AMF[%s]: SubmitInput failed (%s)", label_, amf_result_name(res));
		return EncodeResult::fatal;
	}

	return EncodeResult::ok;
}

EncodeResult AmfStreamEncoder::retrieve(std::vector<uint8_t> & out, bool & out_idr)
{
	out_idr = false;
	if (component_ == nullptr)
		return EncodeResult::fatal;

	AMF_RESULT res = AMF_OK;
	const int64_t submitted_at = submitted_at_;

	// The poll loop. With QUERY_TIMEOUT the first QueryOutput blocks for us; if
	// the runtime does not support it, this is ALVR's 1 ms sleep spin
	// (VideoEncoderAMF.cpp:41-44) with a much shorter budget, because a frame
	// that is not out within a couple of frame periods is of no use to anybody.
	LARGE_INTEGER freq{};
	QueryPerformanceFrequency(&freq);
	const int64_t timeout_ticks = freq.QuadPart / 1000 * static_cast<int64_t>(params_.poll_timeout_ms);

	amf::AMFData * data = nullptr;
	for (;;)
	{
		res = component_->QueryOutput(&data);
		if (res == AMF_OK && data != nullptr)
			break;

		if (res != AMF_OK && res != AMF_REPEAT && res != AMF_NEED_MORE_INPUT)
		{
			log_line("AMF[%s]: QueryOutput failed (%s)", label_, amf_result_name(res));
			return EncodeResult::fatal;
		}

		if (qpc_now() - submitted_at > timeout_ticks)
		{
			log_line("AMF[%s]: no output within %u ms, dropping frame",
			         label_,
			         params_.poll_timeout_ms);
			return EncodeResult::dropped;
		}

		if (not has_query_timeout_)
			Sleep(1);
	}

	amf::AMFBufferPtr buffer(data);
	if (not buffer)
	{
		log_line("AMF[%s]: output is not a buffer", label_);
		data->Release();
		return EncodeResult::fatal;
	}

	out_idr = output_is_idr(data);

	const auto * bytes = static_cast<const uint8_t *>(buffer->GetNative());
	const size_t size = buffer->GetSize();
	if (bytes != nullptr && size != 0)
		out.insert(out.end(), bytes, bytes + size);

	// Round-trip the submit timestamp through the surface so the number in the
	// log is the encoder's own latency and not the caller's wall clock, which on
	// an asynchronous encoder are not the same thing.
	amf_int64 submit_ticks = 0;
	if (data->GetProperty(kSubmitTicksProperty, &submit_ticks) == AMF_OK && submit_ticks != 0)
		last_encode_ms_ = qpc_to_ms(qpc_now() - submit_ticks);
	else
		last_encode_ms_ = qpc_to_ms(qpc_now() - submitted_at);

	data->Release();

	if (size == 0)
	{
		log_line("AMF[%s]: encoder returned an empty buffer", label_);
		return EncodeResult::dropped;
	}
	return EncodeResult::ok;
}

} // namespace wivrnnx::helper
