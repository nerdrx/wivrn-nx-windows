// Dynamic load of the AMD Media Framework runtime.
//
// amfrt64.dll ships with the Radeon driver and with nothing else. The public AMF
// SDK has no import library at all — the whole ABI is two C exports, AMFInit and
// AMFQueryVersion, and everything after that is COM-shaped vtables out of
// external/amf. So the helper never links against AMF: it LoadLibrary()s it at
// the moment a frame first needs encoding, and a machine with no Radeon (or a
// Wine prefix, where the DLL simply does not exist) gets a log line and a video
// path that drops every frame instead of a failure to start.
//
// The DLL name can be overridden with WIVRNNX_AMF_DLL, which is what the ABI
// mock in src/tests/amf_stub.cpp uses to stand in for the real runtime.
#pragma once

#include <cstdint>

namespace amf
{
class AMFFactory;
}

namespace wivrnnx::helper
{

class AmfLoader
{
public:
	// One runtime per process: AMFInit hands out the same factory singleton
	// anyway, and unloading it under a live context is not supported.
	static AmfLoader & instance();

	// Idempotent. Logs once, on the first call, whether it worked and why not.
	bool load();

	bool loaded() const
	{
		return factory_ != nullptr;
	}

	amf::AMFFactory * factory() const
	{
		return factory_;
	}

	// AMFQueryVersion of the loaded runtime, in AMF_MAKE_FULL_VERSION form.
	uint64_t runtime_version() const
	{
		return runtime_version_;
	}

	// The name that was actually loaded, for the log.
	const char * dll_name() const
	{
		return dll_name_;
	}

private:
	AmfLoader() = default;

	bool attempted_ = false;
	void * module_ = nullptr; // HMODULE, kept opaque so this header stays cheap
	amf::AMFFactory * factory_ = nullptr;
	uint64_t runtime_version_ = 0;
	const char * dll_name_ = "amfrt64.dll";
};

// AMF_RESULT rendered for a log line. Returns a static string for the values
// this tree can provoke and "AMF_RESULT <n>" for anything else.
const char * amf_result_name(int result);

} // namespace wivrnnx::helper
