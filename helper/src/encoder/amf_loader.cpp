#include "amf_loader.h"

#include <windows.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "core/Factory.h"
#include "core/Result.h"

#include "../log.h"

namespace wivrnnx::helper
{

namespace
{

// The version handed to AMFInit. Passing the SDK version the headers were taken
// from is what AMF's own AMFFactoryHelper does, but that fails on a runtime older
// than the headers — and Polaris sits on a legacy driver branch whose AMF is
// several minor versions behind current master. So the runtime is asked what it
// is first, and AMFInit is called with whichever of the two is lower: an older
// runtime is then initialised at its own version rather than refused, and a newer
// one is still held to the contract these headers describe.
uint64_t init_version(uint64_t runtime_version)
{
	if (runtime_version == 0 || runtime_version > AMF_FULL_VERSION)
		return AMF_FULL_VERSION;
	return runtime_version;
}

std::wstring widen_ascii(const char * s)
{
	std::wstring w;
	for (; *s != '\0'; ++s)
		w.push_back(static_cast<wchar_t>(static_cast<unsigned char>(*s)));
	return w;
}

// Copied out of the environment once; the pointer has to outlive load().
const char * amf_dll_name()
{
	static std::string name = [] {
		const char * override_name = std::getenv("WIVRNNX_AMF_DLL");
		if (override_name != nullptr && override_name[0] != '\0')
			return std::string(override_name);
		return std::string(AMF_DLL_NAMEA);
	}();
	return name.c_str();
}

} // namespace

const char * amf_result_name(int result)
{
	switch (static_cast<AMF_RESULT>(result))
	{
		case AMF_OK:
			return "AMF_OK";
		case AMF_FAIL:
			return "AMF_FAIL";
		case AMF_INVALID_ARG:
			return "AMF_INVALID_ARG";
		case AMF_OUT_OF_MEMORY:
			return "AMF_OUT_OF_MEMORY";
		case AMF_NO_INTERFACE:
			return "AMF_NO_INTERFACE";
		case AMF_NOT_IMPLEMENTED:
			return "AMF_NOT_IMPLEMENTED";
		case AMF_NOT_SUPPORTED:
			return "AMF_NOT_SUPPORTED";
		case AMF_NOT_FOUND:
			return "AMF_NOT_FOUND";
		case AMF_ALREADY_INITIALIZED:
			return "AMF_ALREADY_INITIALIZED";
		case AMF_NOT_INITIALIZED:
			return "AMF_NOT_INITIALIZED";
		case AMF_INVALID_FORMAT:
			return "AMF_INVALID_FORMAT";
		case AMF_NO_DEVICE:
			return "AMF_NO_DEVICE";
		case AMF_EOF:
			return "AMF_EOF";
		case AMF_REPEAT:
			return "AMF_REPEAT";
		case AMF_INPUT_FULL:
			return "AMF_INPUT_FULL";
		case AMF_ENCODER_NOT_PRESENT:
			return "AMF_ENCODER_NOT_PRESENT";
		case AMF_NEED_MORE_INPUT:
			return "AMF_NEED_MORE_INPUT";
		default:
			break;
	}

	static thread_local char buf[32];
	std::snprintf(buf, sizeof(buf), "AMF_RESULT %d", result);
	return buf;
}

AmfLoader & AmfLoader::instance()
{
	static AmfLoader loader;
	return loader;
}

bool AmfLoader::load()
{
	if (attempted_)
		return factory_ != nullptr;
	attempted_ = true;

	dll_name_ = amf_dll_name();

	HMODULE module = LoadLibraryW(widen_ascii(dll_name_).c_str());
	if (module == nullptr)
	{
		const DWORD err = GetLastError();
		if (err == ERROR_MOD_NOT_FOUND || err == ERROR_FILE_NOT_FOUND)
			log_line("AMF: %s not found - no Radeon driver on this machine? video will not be encoded",
			         dll_name_);
		else
			log_win32(err, "AMF: LoadLibrary(%s) failed", dll_name_);
		return false;
	}
	module_ = module;

	auto query_version = reinterpret_cast<AMFQueryVersion_Fn>(
	        reinterpret_cast<void *>(GetProcAddress(module, AMF_QUERY_VERSION_FUNCTION_NAME)));
	auto init = reinterpret_cast<AMFInit_Fn>(
	        reinterpret_cast<void *>(GetProcAddress(module, AMF_INIT_FUNCTION_NAME)));
	if (query_version == nullptr || init == nullptr)
	{
		log_line("AMF: %s has no %s/%s export - not an AMF runtime",
		         dll_name_,
		         AMF_QUERY_VERSION_FUNCTION_NAME,
		         AMF_INIT_FUNCTION_NAME);
		FreeLibrary(module);
		module_ = nullptr;
		return false;
	}

	amf_uint64 version = 0;
	const AMF_RESULT vres = query_version(&version);
	if (vres != AMF_OK)
	{
		log_line("AMF: AMFQueryVersion failed (%s)", amf_result_name(vres));
		version = 0;
	}
	runtime_version_ = version;

	const uint64_t asked = init_version(version);
	amf::AMFFactory * factory = nullptr;
	const AMF_RESULT ires = init(asked, &factory);
	if (ires != AMF_OK || factory == nullptr)
	{
		log_line("AMF: AMFInit(%llu.%llu.%llu.%llu) failed (%s)",
		         static_cast<unsigned long long>(AMF_GET_MAJOR_VERSION(asked)),
		         static_cast<unsigned long long>(AMF_GET_MINOR_VERSION(asked)),
		         static_cast<unsigned long long>(AMF_GET_SUBMINOR_VERSION(asked)),
		         static_cast<unsigned long long>(AMF_GET_BUILD_VERSION(asked)),
		         amf_result_name(ires));
		FreeLibrary(module);
		module_ = nullptr;
		return false;
	}

	factory_ = factory;
	log_line("AMF: loaded %s, runtime %llu.%llu.%llu.%llu (headers %d.%d.%d.%d)",
	         dll_name_,
	         static_cast<unsigned long long>(AMF_GET_MAJOR_VERSION(version)),
	         static_cast<unsigned long long>(AMF_GET_MINOR_VERSION(version)),
	         static_cast<unsigned long long>(AMF_GET_SUBMINOR_VERSION(version)),
	         static_cast<unsigned long long>(AMF_GET_BUILD_VERSION(version)),
	         AMF_VERSION_MAJOR,
	         AMF_VERSION_MINOR,
	         AMF_VERSION_RELEASE,
	         AMF_VERSION_BUILD_NUM);
	return true;
}

} // namespace wivrnnx::helper
