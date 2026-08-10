#include "amf_probe.h"

#include <windows.h>

#include <d3d11.h>
#include <dxgi.h>

#include <functional>
#include <string>
#include <vector>

#include "components/VideoEncoderHEVC.h"
#include "components/VideoEncoderVCE.h"
#include "core/Factory.h"

#include "../log.h"
#include "amf_encoder.h"
#include "amf_loader.h"

namespace wivrnnx::helper
{

namespace
{

// One configure-time property of the AVC encoder: its symbolic name for the
// log, and the SetProperty call bound to the probe's parameters.
struct AvcProp
{
	const char * sym;
	std::function<AMF_RESULT(amf::AMFComponent *)> set;
};

// The H.264 property list in exactly configure_h264's order, QUERY_TIMEOUT
// included unconditionally (the probe wants its result, not a caps-gated
// skip). Kept in step with configure_h264 by hand; the probe is a diagnostic,
// not a second implementation to keep honest forever.
std::vector<AvcProp> avc_props(uint32_t width, uint32_t height, float refresh_hz, uint32_t bitrate_bps)
{
	const amf_int64 bitrate = bitrate_bps;
	const amf_int32 fps = static_cast<amf_int32>(refresh_hz > 1.f ? refresh_hz : 90.f);

#define PROP(name, ...)                          \
	AvcProp                                      \
	{                                            \
		#name, [=](amf::AMFComponent * e) {      \
			return e->SetProperty(name, __VA_ARGS__); \
		}                                        \
	}
	return {
	        PROP(AMF_VIDEO_ENCODER_USAGE, AMF_VIDEO_ENCODER_USAGE_ULTRA_LOW_LATENCY),
	        PROP(AMF_VIDEO_ENCODER_PROFILE, AMF_VIDEO_ENCODER_PROFILE_HIGH),
	        PROP(AMF_VIDEO_ENCODER_PROFILE_LEVEL, 42),
	        PROP(AMF_VIDEO_ENCODER_RATE_CONTROL_METHOD, AMF_VIDEO_ENCODER_RATE_CONTROL_METHOD_CBR),
	        PROP(AMF_VIDEO_ENCODER_FILLER_DATA_ENABLE, false),
	        PROP(AMF_VIDEO_ENCODER_CABAC_ENABLE, AMF_VIDEO_ENCODER_CABAC),
	        PROP(AMF_VIDEO_ENCODER_TARGET_BITRATE, bitrate),
	        PROP(AMF_VIDEO_ENCODER_PEAK_BITRATE, bitrate),
	        PROP(AMF_VIDEO_ENCODER_FRAMESIZE,
	             ::AMFConstructSize(static_cast<amf_int32>(width), static_cast<amf_int32>(height))),
	        PROP(AMF_VIDEO_ENCODER_FRAMERATE, ::AMFConstructRate(fps, 1)),
	        PROP(AMF_VIDEO_ENCODER_B_PIC_PATTERN, 0),
	        PROP(AMF_VIDEO_ENCODER_QUALITY_PRESET, AMF_VIDEO_ENCODER_QUALITY_PRESET_SPEED),
	        PROP(AMF_VIDEO_ENCODER_FULL_RANGE_COLOR, true),
	        PROP(AMF_VIDEO_ENCODER_OUTPUT_COLOR_PROFILE, AMF_VIDEO_CONVERTER_COLOR_PROFILE_FULL_709),
	        PROP(AMF_VIDEO_ENCODER_OUTPUT_TRANSFER_CHARACTERISTIC,
	             AMF_COLOR_TRANSFER_CHARACTERISTIC_GAMMA22),
	        PROP(AMF_VIDEO_ENCODER_OUTPUT_COLOR_PRIMARIES, AMF_COLOR_PRIMARIES_BT709),
	        PROP(AMF_VIDEO_ENCODER_IDR_PERIOD, 0),
	        PROP(AMF_VIDEO_ENCODER_INSERT_AUD, false),
	        PROP(AMF_VIDEO_ENCODER_VBV_BUFFER_SIZE,
	             static_cast<amf_int64>(static_cast<double>(bitrate) / fps * 1.1)),
	        PROP(AMF_VIDEO_ENCODER_MAX_NUM_REFRAMES, 0),
	        PROP(AMF_VIDEO_ENCODER_QUERY_TIMEOUT, static_cast<amf_int64>(40)),
	};
#undef PROP
}

// One fresh component: apply `use[i]`-selected properties, Init, tear down.
// Per-SetProperty failures are logged; the Init result is returned.
AMF_RESULT try_avc_init(amf::AMFContext * context,
                        const std::vector<AvcProp> & props,
                        const std::vector<bool> & use,
                        amf::AMF_SURFACE_FORMAT format,
                        uint32_t width,
                        uint32_t height)
{
	amf::AMFComponent * component = nullptr;
	AMF_RESULT res = AmfLoader::instance().factory()->CreateComponent(context, AMFVideoEncoderVCE_AVC, &component);
	if (res != AMF_OK || component == nullptr)
	{
		log_line("amf-probe:   CreateComponent(AVC) failed (%s)", amf_result_name(res));
		return res != AMF_OK ? res : AMF_FAIL;
	}

	for (size_t i = 0; i < props.size(); ++i)
	{
		if (not use[i])
			continue;
		const AMF_RESULT set_res = props[i].set(component);
		if (set_res != AMF_OK)
			log_line("amf-probe:   SetProperty(%s) -> %s", props[i].sym, amf_result_name(set_res));
	}

	res = component->Init(format, static_cast<amf_int32>(width), static_cast<amf_int32>(height));
	component->Terminate();
	component->Release();
	return res;
}

void log_adapter(ID3D11Device * device)
{
	IDXGIDevice * dxgi_device = nullptr;
	if (FAILED(device->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void **>(&dxgi_device))))
		return;
	IDXGIAdapter * adapter = nullptr;
	if (SUCCEEDED(dxgi_device->GetAdapter(&adapter)) && adapter != nullptr)
	{
		DXGI_ADAPTER_DESC desc{};
		if (SUCCEEDED(adapter->GetDesc(&desc)))
		{
			// The description is UTF-16; the names in question are plain ASCII.
			char name[128] = {};
			for (size_t i = 0; i + 1 < sizeof(name) && desc.Description[i] != L'\0'; ++i)
				name[i] = desc.Description[i] < 128 ? static_cast<char>(desc.Description[i]) : '?';
			log_line("amf-probe: adapter \"%s\" (vendor %04x device %04x)",
			         name,
			         desc.VendorId,
			         desc.DeviceId);
		}
		adapter->Release();
	}
	dxgi_device->Release();
}

} // namespace

int run_amf_probe(uint32_t width, uint32_t height, float refresh_hz, uint32_t bitrate_bps)
{
	log_line("amf-probe: %ux%u @ %.0f Hz, %u kbit/s", width, height, static_cast<double>(refresh_hz), bitrate_bps / 1000);

	ID3D11Device * device = nullptr;
	ID3D11DeviceContext * immediate = nullptr;
	D3D_FEATURE_LEVEL level{};
	HRESULT hr = D3D11CreateDevice(nullptr,
	                               D3D_DRIVER_TYPE_HARDWARE,
	                               nullptr,
	                               0,
	                               nullptr,
	                               0,
	                               D3D11_SDK_VERSION,
	                               &device,
	                               &level,
	                               &immediate);
	if (FAILED(hr) || device == nullptr)
	{
		log_line("amf-probe: D3D11CreateDevice failed (0x%08lx)", static_cast<unsigned long>(hr));
		return 1;
	}
	log_adapter(device);

	{
		AmfContext context;
		if (not context.init(device))
		{
			log_line("amf-probe: no AMF context, nothing to probe");
			immediate->Release();
			device->Release();
			return 1;
		}

		AmfEncodeParams params{};
		params.width = width;
		params.height = height;
		params.refresh_hz = refresh_hz;
		params.bitrate_bps = bitrate_bps;
		params.surface_format = amf_surface_format_for_dxgi(DXGI_FORMAT_R8G8B8A8_UNORM);

		// Control: the production HEVC path, which is expected to come up.
		{
			AmfEncodeParams control = params;
			control.allow_h264 = false;
			AmfStreamEncoder encoder;
			log_line("amf-probe: --- control: production HEVC open ---");
			log_line("amf-probe: HEVC open %s", encoder.open(context, control, "probe") ? "succeeded" : "FAILED");
			encoder.close();
		}

		// The production H.264 path, same code the streamer runs.
		{
			AmfEncodeParams avc = params;
			avc.allow_h265 = false;
			AmfStreamEncoder encoder;
			log_line("amf-probe: --- production H.264 open ---");
			log_line("amf-probe: H.264 open %s", encoder.open(context, avc, "probe") ? "succeeded" : "FAILED");
			encoder.close();
		}

		const std::vector<AvcProp> props = avc_props(width, height, refresh_hz, bitrate_bps);
		const auto rgba = static_cast<amf::AMF_SURFACE_FORMAT>(params.surface_format);

		// Prefix ladder: variant k applies the first k properties. The first k
		// whose Init result differs from k-1's names the culprit — if the fault
		// is a single property at all.
		log_line("amf-probe: --- prefix ladder, RGBA %ux%u ---", width, height);
		for (size_t k = 0; k <= props.size(); ++k)
		{
			std::vector<bool> use(props.size(), false);
			for (size_t i = 0; i < k; ++i)
				use[i] = true;
			const AMF_RESULT res = try_avc_init(context.get(), props, use, rgba, width, height);
			log_line("amf-probe: first %2zu%s%s: Init -> %s",
			         k,
			         k == 0 ? " (none)" : " up to ",
			         k == 0 ? "" : props[k - 1].sym,
			         amf_result_name(res));
		}

		// Leave-one-out: the full set minus one property each. Catches the case
		// where the prefix ladder pins nothing because two properties only fail
		// in combination with what comes later.
		log_line("amf-probe: --- leave-one-out, RGBA %ux%u ---", width, height);
		for (size_t skip = 0; skip < props.size(); ++skip)
		{
			std::vector<bool> use(props.size(), true);
			use[skip] = false;
			const AMF_RESULT res = try_avc_init(context.get(), props, use, rgba, width, height);
			log_line("amf-probe: all but %s: Init -> %s", props[skip].sym, amf_result_name(res));
		}

		// Full set again, NV12 in: separates "this property set" from "AVC
		// refuses to take RGBA and do its own colour conversion here".
		{
			std::vector<bool> use(props.size(), true);
			const AMF_RESULT res = try_avc_init(context.get(), props, use, amf::AMF_SURFACE_NV12, width, height);
			log_line("amf-probe: full set, NV12 in: Init -> %s", amf_result_name(res));
		}

		// Replacements for the usage preset, found broken on the Polaris legacy
		// runtime: the rest of the set with the first property (USAGE) swapped
		// for each candidate. LOWLATENCY_MODE is the documented knob behind the
		// preset's one-in-one-out behaviour; MAX_NUM_REFRAMES=1 probes the
		// OUT_OF_RANGE seen for 0.
		log_line("amf-probe: --- usage variants, RGBA %ux%u ---", width, height);
		struct UsageVariant
		{
			const char * what;
			AvcProp replacement;
		};
		const UsageVariant variants[] = {
		        {"USAGE=LOW_LATENCY",
		         {"AMF_VIDEO_ENCODER_USAGE", [](amf::AMFComponent * e) {
			          return e->SetProperty(AMF_VIDEO_ENCODER_USAGE, AMF_VIDEO_ENCODER_USAGE_LOW_LATENCY);
		          }}},
		        {"USAGE=TRANSCODING",
		         {"AMF_VIDEO_ENCODER_USAGE", [](amf::AMFComponent * e) {
			          return e->SetProperty(AMF_VIDEO_ENCODER_USAGE, AMF_VIDEO_ENCODER_USAGE_TRANSCODING);
		          }}},
		        {"no usage, LOWLATENCY_MODE=true",
		         {"AMF_VIDEO_ENCODER_LOWLATENCY_MODE", [](amf::AMFComponent * e) {
			          return e->SetProperty(AMF_VIDEO_ENCODER_LOWLATENCY_MODE, true);
		          }}},
		        {"no usage, MAX_NUM_REFRAMES=1",
		         {"AMF_VIDEO_ENCODER_MAX_NUM_REFRAMES", [](amf::AMFComponent * e) {
			          return e->SetProperty(AMF_VIDEO_ENCODER_MAX_NUM_REFRAMES, 1);
		          }}},
		};
		for (const UsageVariant & variant: variants)
		{
			std::vector<AvcProp> modified = props;
			modified[0] = variant.replacement;
			std::vector<bool> use(modified.size(), true);
			const AMF_RESULT res = try_avc_init(context.get(), modified, use, rgba, width, height);
			log_line("amf-probe: %s: Init -> %s", variant.what, amf_result_name(res));
		}
	}

	immediate->Release();
	device->Release();
	log_line("amf-probe: done");
	return 0;
}

} // namespace wivrnnx::helper
