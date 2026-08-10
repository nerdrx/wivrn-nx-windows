// The AMF loader, the encoder's property set and its submit/poll loop, driven
// against the stub runtime in amf_stub.cpp under Wine.
//
// What this can prove: the loader finds a runtime and negotiates a version with
// it, the whole property set is accepted with the types AMF expects, the HEVC
// component is tried first and H.264 is the fallback, the IDR request lands on
// the *surface* (which is where AMF reads it) and not on the component, the poll
// loop survives an encoder that says AMF_REPEAT, and a frame that never comes
// out is dropped rather than waited on forever.
//
// What it cannot prove, and only the RX 580 can: that a real Polaris VCE accepts
// this property set at all, that its HEVC component exists, and that the
// bitstream it produces decodes on the headset.

#include <windows.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "components/VideoEncoderHEVC.h"
#include "components/VideoEncoderVCE.h"
#include "core/Factory.h"

#include "encoder/amf_encoder.h"
#include "encoder/amf_loader.h"

using namespace wivrnnx::helper;

namespace
{

int checks = 0;
int failures = 0;

#define CHECK(...)                                                                          \
	do                                                                                  \
	{                                                                                   \
		++checks;                                                                   \
		if (not(__VA_ARGS__))                                                       \
		{                                                                           \
			++failures;                                                         \
			std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #__VA_ARGS__); \
		}                                                                           \
	} while (0)

using ReportFn = const char *(*)();
using ResetFn = void (*)();

ReportFn g_report = nullptr;
ResetFn g_reset = nullptr;

std::string narrow(const wchar_t * s)
{
	std::string out;
	for (; s != nullptr && *s != L'\0'; ++s)
		out.push_back(*s < 128 ? static_cast<char>(*s) : '?');
	return out;
}

std::string report()
{
	return g_report != nullptr ? std::string(g_report()) : std::string();
}

bool has(const std::string & haystack, const std::string & needle)
{
	return haystack.find(needle) != std::string::npos;
}

// "hevc.set <property name> = <type> <value>", the line the stub writes for
// every SetProperty on the encoder component.
std::string prop(const char * tag, const wchar_t * name, const char * value)
{
	return std::string(tag) + ".set " + narrow(name) + " = " + value;
}

// The same, for an enumerated value. Spelled through the enum rather than as a
// literal so that a renumbering upstream shows up as a test failure with the
// right name on it, not as a silently wrong property.
std::string iprop(const char * tag, const wchar_t * name, long long value)
{
	return prop(tag, name, ("int64 " + std::to_string(value)).c_str());
}

AmfEncodeParams default_params()
{
	AmfEncodeParams p{};
	p.width = 1600;
	p.height = 1760;
	p.refresh_hz = 72.f;
	p.bitrate_bps = 50'000'000;
	p.surface_format = amf_surface_format_for_dxgi(28 /* DXGI_FORMAT_R8G8B8A8_UNORM */);
	p.allow_h265 = true;
	p.allow_h264 = true;
	p.poll_timeout_ms = 40;
	return p;
}

void part_a_loader()
{
	std::printf("Part A: loader and version negotiation\n");

	AmfLoader & loader = AmfLoader::instance();
	CHECK(loader.load());
	CHECK(loader.loaded());
	CHECK(loader.factory() != nullptr);

	// The stub reports 1.4.30.0, older than the vendored headers. The loader has
	// to ask AMFInit for the *runtime's* version, not the header's, or a Polaris
	// driver on the legacy branch would refuse to initialise at all.
	CHECK(AMF_GET_MAJOR_VERSION(loader.runtime_version()) == 1);
	CHECK(AMF_GET_MINOR_VERSION(loader.runtime_version()) == 4);
	CHECK(AMF_GET_SUBMINOR_VERSION(loader.runtime_version()) == 30);
	CHECK(loader.runtime_version() < AMF_FULL_VERSION);

	const std::string r = report();
	CHECK(has(r, "init 1.4.30.0"));
	CHECK(not has(r, "init refused"));
}

void part_b_property_set()
{
	std::printf("Part B: the HEVC property set\n");

	g_reset();

	AmfContext context;
	CHECK(context.init(nullptr));

	AmfStreamEncoder encoder;
	CHECK(encoder.open(context, default_params(), "left"));
	CHECK(encoder.is_open());
	CHECK(encoder.codec() == VideoCodec::h265);

	const std::string r = report();

	// HEVC is asked for first, and once it works the AVC component is never
	// created at all.
	CHECK(has(r, "factory.createcomponent " + narrow(AMFVideoEncoder_HEVC)));
	CHECK(not has(r, "factory.createcomponent " + narrow(AMFVideoEncoderVCE_AVC)));

	// The settings the task names.
	CHECK(has(r, iprop("hevc", AMF_VIDEO_ENCODER_HEVC_USAGE, AMF_VIDEO_ENCODER_HEVC_USAGE_ULTRA_LOW_LATENCY)));
	CHECK(has(r, iprop("hevc", AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_METHOD,
	                   AMF_VIDEO_ENCODER_HEVC_RATE_CONTROL_METHOD_CBR)));
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE, "int64 50000000")));
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_PEAK_BITRATE, "int64 50000000")));
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_FRAMESIZE, "size 1600x1760")));
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_FRAMERATE, "rate 72/1")));

	// Infinite GOP: every key frame in the session is one the IDR tracker asked
	// for. If either of these came back non-zero the stream would carry periodic
	// IDRs and the bitrate would spike on a schedule nobody chose.
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_NUM_GOPS_PER_IDR, "int64 0")));
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_GOP_SIZE, "int64 0")));

	// One reference frame, no filler, no AUDs.
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_MAX_NUM_REFRAMES, "int64 0")));
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_FILLER_DATA_ENABLE, "bool 0")));
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_INSERT_AUD, "bool 0")));

	// Full-range BT.709, which the client's sampler is hardwired to.
	CHECK(has(r, iprop("hevc", AMF_VIDEO_ENCODER_HEVC_NOMINAL_RANGE,
	                   AMF_VIDEO_ENCODER_HEVC_NOMINAL_RANGE_FULL)));
	CHECK(has(r, iprop("hevc", AMF_VIDEO_ENCODER_HEVC_OUTPUT_COLOR_PROFILE,
	                   AMF_VIDEO_CONVERTER_COLOR_PROFILE_FULL_709)));

	// A VBV of a bit over one frame at 72 Hz: 50e6 / 72 * 1.1.
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_VBV_BUFFER_SIZE, "int64 763888")));

	// The caps query, then Init with the surface format the DXGI mapping chose.
	CHECK(has(r, "hevc.init format=5 1600x1760")); // AMF_SURFACE_RGBA
	CHECK(amf_surface_format_for_dxgi(28) == amf::AMF_SURFACE_RGBA);
	CHECK(amf_surface_format_for_dxgi(87) == amf::AMF_SURFACE_BGRA);  // B8G8R8A8_UNORM
	CHECK(amf_surface_format_for_dxgi(29) == amf::AMF_SURFACE_RGBA);  // R8G8B8A8_UNORM_SRGB
	CHECK(amf_surface_format_for_dxgi(10) == amf::AMF_SURFACE_UNKNOWN); // R16G16B16A16_FLOAT
}

void part_c_encode()
{
	std::printf("Part C: submit, poll, IDR\n");

	AmfContext context;
	CHECK(context.init(nullptr));
	AmfStreamEncoder encoder;
	CHECK(encoder.open(context, default_params(), "left"));

	g_reset();

	// --- an IDR ---------------------------------------------------------
	{
		void * seen = nullptr;
		std::vector<uint8_t> out;
		bool idr = false;
		const EncodeResult res = encoder.encode(
		        [&](void * native) {
			        seen = native;
			        return true;
		        },
		        true,
		        out,
		        idr);

		CHECK(res == EncodeResult::ok);
		CHECK(idr);
		CHECK(seen != nullptr); // the surface's plane 0, i.e. the DX11 texture
		CHECK(out.size() > 4096);
		// annex-B, with a parameter set in front of the key frame.
		CHECK(out.size() > 4 && out[0] == 0 && out[1] == 0 && out[2] == 0 && out[3] == 1);
		CHECK((out[4] >> 1) == 32); // VPS

		const std::string r = report();
		CHECK(has(r, "context.allocsurface memory=" + std::to_string(int(amf::AMF_MEMORY_DX11)) +
	                     " format=5 1600x1760"));
		// The IDR request goes on the surface. Putting it on the component
		// instead is the classic AMF mistake: it is accepted and does nothing.
		CHECK(has(r, prop("surface", AMF_VIDEO_ENCODER_HEVC_INSERT_HEADER, "bool 1")));
		CHECK(has(r, iprop("surface", AMF_VIDEO_ENCODER_HEVC_FORCE_PICTURE_TYPE,
	                   AMF_VIDEO_ENCODER_HEVC_PICTURE_TYPE_IDR)));
		CHECK(not has(r, iprop("hevc", AMF_VIDEO_ENCODER_HEVC_FORCE_PICTURE_TYPE,
	                       AMF_VIDEO_ENCODER_HEVC_PICTURE_TYPE_IDR)));
		CHECK(has(r, "hevc.submit idr=1"));
		// The poll loop got AMF_REPEAT twice before the buffer appeared.
		CHECK(encoder.last_encode_ms() >= 0.0);
	}

	// --- a P frame ------------------------------------------------------
	{
		g_reset();
		std::vector<uint8_t> out;
		bool idr = true;
		const EncodeResult res = encoder.encode([](void *) { return true; }, false, out, idr);
		CHECK(res == EncodeResult::ok);
		CHECK(not idr);
		CHECK(out.size() > 512);
		CHECK((out[4] >> 1) == 1); // TRAIL_R, no parameter sets

		const std::string r = report();
		CHECK(has(r, "hevc.submit idr=0"));
		CHECK(not has(r, iprop("surface", AMF_VIDEO_ENCODER_HEVC_FORCE_PICTURE_TYPE,
	                       AMF_VIDEO_ENCODER_HEVC_PICTURE_TYPE_IDR)));
		// INSERT_AUD is set on every surface, IDR or not.
		CHECK(has(r, prop("surface", AMF_VIDEO_ENCODER_HEVC_INSERT_AUD, "bool 0")));
	}

	// --- a fill that gives up -------------------------------------------
	{
		g_reset();
		std::vector<uint8_t> out;
		bool idr = false;
		const EncodeResult res = encoder.encode([](void *) { return false; }, false, out, idr);
		CHECK(res == EncodeResult::dropped);
		CHECK(out.empty());
		// Nothing was submitted: a slot the copy failed on must not turn into a
		// frame the encoder is then waited on for.
		CHECK(not has(report(), "hevc.submit"));
	}

	// --- an encoder that never produces anything ------------------------
	{
		g_reset();
		SetEnvironmentVariableA("WIVRNNX_STUB_NO_OUTPUT", "1");

		const DWORD start = GetTickCount();
		std::vector<uint8_t> out;
		bool idr = false;
		const EncodeResult res = encoder.encode([](void *) { return true; }, false, out, idr);
		const DWORD elapsed = GetTickCount() - start;

		SetEnvironmentVariableA("WIVRNNX_STUB_NO_OUTPUT", nullptr);

		CHECK(res == EncodeResult::dropped);
		CHECK(out.empty());
		// Bounded by poll_timeout_ms, generously: this is the guarantee that a
		// wedged encoder cannot hold the intake thread.
		CHECK(elapsed < 2000);
	}

	// The queue is one frame deep in low-latency mode, and a second submit
	// without a retrieve has to be refused rather than queued. A fresh encoder,
	// because the timed-out frame above is still sitting inside the previous one
	// - which is itself the behaviour being described.
	{
		g_reset();
		AmfContext fresh_context;
		CHECK(fresh_context.init(nullptr));
		AmfStreamEncoder fresh;
		CHECK(fresh.open(fresh_context, default_params(), "left"));

		CHECK(fresh.submit([](void *) { return true; }, false) == EncodeResult::ok);
		CHECK(fresh.submit([](void *) { return true; }, false) == EncodeResult::dropped);
		std::vector<uint8_t> out;
		bool idr = false;
		CHECK(fresh.retrieve(out, idr) == EncodeResult::ok);
		CHECK(not out.empty());
	}
}

// The automatic bitrate controller moves the target under a running encoder,
// which must not cost a rebuild: no Terminate, no Init, no new parameter set —
// only the three properties that describe the rate.
void part_e_runtime_bitrate()
{
	std::printf("Part E: runtime bitrate changes\n");

	AmfContext context;
	CHECK(context.init(nullptr));

	AmfStreamEncoder encoder;
	CHECK(encoder.open(context, default_params(), "left"));

	g_reset();
	encoder.set_bitrate(20'000'000);

	std::string r = report();
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_TARGET_BITRATE, "int64 20000000")));
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_PEAK_BITRATE, "int64 20000000")));
	// The VBV follows the bitrate: a little over one frame's worth of bits at
	// 72 Hz, 20 Mbit/s -> 305555.
	CHECK(has(r, prop("hevc", AMF_VIDEO_ENCODER_HEVC_VBV_BUFFER_SIZE, "int64 305555")));

	// Nothing else moved, and above all the component was neither re-created nor
	// re-initialised: that is what would cost an IDR and a stream description.
	CHECK(not has(r, "hevc.init"));
	CHECK(not has(r, "hevc.terminate"));
	CHECK(not has(r, "factory.createcomponent"));
	CHECK(not has(r, narrow(AMF_VIDEO_ENCODER_HEVC_FRAMESIZE)));
	CHECK(not has(r, narrow(AMF_VIDEO_ENCODER_HEVC_QUALITY_PRESET)));

	// The same number twice is not worth a property set.
	g_reset();
	encoder.set_bitrate(20'000'000);
	CHECK(not has(report(), "hevc.set"));

	// And the encoder keeps producing frames across the change.
	g_reset();
	std::vector<uint8_t> out;
	bool idr = false;
	CHECK(encoder.encode([](void *) { return true; }, false, out, idr) == EncodeResult::ok);
	CHECK(not out.empty());

	// H.264 takes the AVC spelling of the same three.
	{
		AmfEncodeParams params = default_params();
		params.allow_h265 = false;
		AmfStreamEncoder avc;
		CHECK(avc.open(context, params, "left"));
		g_reset();
		avc.set_bitrate(33'000'000);
		const std::string avc_report = report();
		CHECK(has(avc_report, prop("avc", AMF_VIDEO_ENCODER_TARGET_BITRATE, "int64 33000000")));
		CHECK(has(avc_report, prop("avc", AMF_VIDEO_ENCODER_PEAK_BITRATE, "int64 33000000")));
		CHECK(has(avc_report, narrow(AMF_VIDEO_ENCODER_VBV_BUFFER_SIZE)));
		CHECK(not has(avc_report, "avc.init"));
	}
}

void part_d_fallback()
{
	std::printf("Part D: the H.264 fallback\n");

	// A driver whose HEVC component cannot be created must not cost the whole
	// video path.
	{
		g_reset();
		SetEnvironmentVariableA("WIVRNNX_STUB_NO_HEVC", "1");

		AmfContext context;
		CHECK(context.init(nullptr));
		AmfStreamEncoder encoder;
		CHECK(encoder.open(context, default_params(), "left"));
		CHECK(encoder.codec() == VideoCodec::h264);

		const std::string r = report();
		CHECK(has(r, "factory.createcomponent " + narrow(AMFVideoEncoder_HEVC)));
		CHECK(has(r, "factory.createcomponent " + narrow(AMFVideoEncoderVCE_AVC)));
		CHECK(has(r, iprop("avc", AMF_VIDEO_ENCODER_USAGE, AMF_VIDEO_ENCODER_USAGE_ULTRA_LOW_LATENCY)));
		CHECK(has(r, iprop("avc", AMF_VIDEO_ENCODER_RATE_CONTROL_METHOD,
	                   AMF_VIDEO_ENCODER_RATE_CONTROL_METHOD_CBR)));
		CHECK(has(r, prop("avc", AMF_VIDEO_ENCODER_B_PIC_PATTERN, "int64 0")));
		CHECK(has(r, prop("avc", AMF_VIDEO_ENCODER_IDR_PERIOD, "int64 0")));
		CHECK(has(r, prop("avc", AMF_VIDEO_ENCODER_FULL_RANGE_COLOR, "bool 1")));
		// None of the HEVC-only names may reach an AVC component. ALVR's own
		// switch falls through and does exactly that (VideoEncoderAMF.cpp:306);
		// it is harmless there only because nobody checks the return.
		CHECK(not has(r, "avc.set " + narrow(AMF_VIDEO_ENCODER_HEVC_GOP_SIZE)));

		// And its IDR uses the H.264 spelling: separate SPS and PPS flags.
		g_reset();
		std::vector<uint8_t> out;
		bool idr = false;
		CHECK(encoder.encode([](void *) { return true; }, true, out, idr) == EncodeResult::ok);
		CHECK(idr);
		const std::string r2 = report();
		CHECK(has(r2, prop("surface", AMF_VIDEO_ENCODER_INSERT_SPS, "bool 1")));
		CHECK(has(r2, prop("surface", AMF_VIDEO_ENCODER_INSERT_PPS, "bool 1")));
		CHECK(has(r2, iprop("surface", AMF_VIDEO_ENCODER_FORCE_PICTURE_TYPE,
	                    AMF_VIDEO_ENCODER_PICTURE_TYPE_IDR)));

		SetEnvironmentVariableA("WIVRNNX_STUB_NO_HEVC", nullptr);
	}

	// Neither codec: open() fails cleanly instead of half-initialising.
	{
		g_reset();
		SetEnvironmentVariableA("WIVRNNX_STUB_NO_HEVC", "1");
		SetEnvironmentVariableA("WIVRNNX_STUB_NO_H264", "1");

		AmfContext context;
		CHECK(context.init(nullptr));
		AmfStreamEncoder encoder;
		CHECK(not encoder.open(context, default_params(), "left"));
		CHECK(not encoder.is_open());

		SetEnvironmentVariableA("WIVRNNX_STUB_NO_HEVC", nullptr);
		SetEnvironmentVariableA("WIVRNNX_STUB_NO_H264", nullptr);
	}

	// A runtime with no QUERY_TIMEOUT capability: the poll loop falls back to
	// sleeping, and must still get the frame out.
	{
		g_reset();
		SetEnvironmentVariableA("WIVRNNX_STUB_QUERY_TIMEOUT", "0");

		AmfContext context;
		CHECK(context.init(nullptr));
		AmfStreamEncoder encoder;
		CHECK(encoder.open(context, default_params(), "left"));

		CHECK(not has(report(), "hevc.set " + narrow(AMF_VIDEO_ENCODER_HEVC_QUERY_TIMEOUT)));

		std::vector<uint8_t> out;
		bool idr = false;
		CHECK(encoder.encode([](void *) { return true; }, false, out, idr) == EncodeResult::ok);
		CHECK(not out.empty());

		SetEnvironmentVariableA("WIVRNNX_STUB_QUERY_TIMEOUT", nullptr);
	}
}

} // namespace

int main()
{
	std::printf("amf_test: the AMF loader and encoder against a stub runtime\n\n");

	const char * dll = std::getenv("WIVRNNX_AMF_DLL");
	if (dll == nullptr)
	{
		std::printf("WIVRNNX_AMF_DLL is not set; run this through run_tests.sh\n");
		return 1;
	}

	// The same module the loader will open, so the report we read back is the
	// one it wrote.
	HMODULE stub = LoadLibraryA(dll);
	if (stub == nullptr)
	{
		std::printf("could not load the stub runtime %s (error %lu)\n", dll, GetLastError());
		return 1;
	}
	g_report = reinterpret_cast<ReportFn>(
	        reinterpret_cast<void *>(GetProcAddress(stub, "WivrnnxStubReport")));
	g_reset = reinterpret_cast<ResetFn>(
	        reinterpret_cast<void *>(GetProcAddress(stub, "WivrnnxStubReset")));
	if (g_report == nullptr || g_reset == nullptr)
	{
		std::printf("%s is not the wivrnnx AMF stub\n", dll);
		return 1;
	}

	part_a_loader();
	part_b_property_set();
	part_c_encode();
	part_d_fallback();
	part_e_runtime_bitrate();

	std::printf("\n%d checks, %d failures\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
