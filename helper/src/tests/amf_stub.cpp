// A stand-in amfrt64.dll.
//
// amfrt64.dll ships with the Radeon driver and exists nowhere else — not on a
// build machine, and not in a Wine prefix. Everything the helper does with AMF
// is therefore unreachable by any test on this side of the SSH connection unless
// the ABI itself is faked, which is what this is: the two C exports, and enough
// of the vtables behind them for src/encoder/amf_loader.cpp and
// src/encoder/amf_encoder.cpp to run to completion against it.
//
// It is not an encoder. It records what was asked of it and hands back a
// synthetic annex-B buffer. What it proves is the part that is otherwise pure
// hope: that the loader finds and initialises a runtime, that the version
// negotiation does the right thing against a runtime older than these headers,
// that every property name in the set is one the ABI accepts as a variant of the
// right type, that the HEVC-to-H.264 fallback fires on a refused component, that
// the submit/poll loop tolerates AMF_REPEAT, and that the IDR properties land on
// the surface rather than the component.
//
// Behaviour switches, all through the environment so the test can drive several
// configurations out of one build:
//
//   WIVRNNX_STUB_VERSION      runtime version as "major.minor.release.build"
//   WIVRNNX_STUB_NO_HEVC      CreateComponent refuses the HEVC encoder
//   WIVRNNX_STUB_NO_H264      ...and the AVC one
//   WIVRNNX_STUB_QUERY_TIMEOUT  "0" to report no QUERY_TIMEOUT capability
//   WIVRNNX_STUB_REPEATS      QueryOutput calls that return AMF_REPEAT first
//   WIVRNNX_STUB_NO_OUTPUT    QueryOutput never produces anything

#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "components/VideoEncoderHEVC.h"
#include "components/VideoEncoderVCE.h"
#include "core/Factory.h"

using namespace amf;

namespace
{

std::string g_report;

// GetEnvironmentVariable, not getenv: the test flips these between scenarios
// with SetEnvironmentVariableA, which updates the process block the Win32 API
// reads but not the CRT's startup copy that getenv answers from.
bool env_get(const char * name, std::string & out)
{
	char buf[64];
	const DWORD n = GetEnvironmentVariableA(name, buf, sizeof(buf));
	if (n == 0 || n >= sizeof(buf))
		return false;
	out.assign(buf, n);
	return true;
}

int env_int(const char * name, int fallback)
{
	std::string v;
	if (not env_get(name, v) || v.empty())
		return fallback;
	return std::atoi(v.c_str());
}

bool env_set(const char * name)
{
	std::string v;
	return env_get(name, v) && not v.empty();
}

std::string narrow(const wchar_t * s)
{
	std::string out;
	for (; s != nullptr && *s != L'\0'; ++s)
		out.push_back(*s < 128 ? static_cast<char>(*s) : '?');
	return out;
}

void report(const char * fmt, ...)
{
	char buf[512];
	va_list args;
	va_start(args, fmt);
	std::vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	g_report += buf;
	g_report += '\n';
}

// Rendered into the report so the test can assert on a property without knowing
// which of the variant's members AMF chose for it.
std::string variant_text(const AMFVariantStruct & v)
{
	char buf[192];
	switch (v.type)
	{
		case AMF_VARIANT_BOOL:
			std::snprintf(buf, sizeof(buf), "bool %d", v.boolValue ? 1 : 0);
			break;
		case AMF_VARIANT_INT64:
			std::snprintf(buf, sizeof(buf), "int64 %lld", static_cast<long long>(v.int64Value));
			break;
		case AMF_VARIANT_DOUBLE:
			std::snprintf(buf, sizeof(buf), "double %f", v.doubleValue);
			break;
		case AMF_VARIANT_SIZE:
			std::snprintf(buf, sizeof(buf), "size %dx%d", v.sizeValue.width, v.sizeValue.height);
			break;
		case AMF_VARIANT_RATE:
			std::snprintf(buf, sizeof(buf), "rate %u/%u", v.rateValue.num, v.rateValue.den);
			break;
		default:
			std::snprintf(buf, sizeof(buf), "type %d", static_cast<int>(v.type));
			break;
	}
	return buf;
}

// --------------------------------------------------------------------------
// The minimum AMFInterface / AMFPropertyStorage a stub needs.
// --------------------------------------------------------------------------

class StubStorage : public AMFPropertyStorage
{
public:
	explicit StubStorage(const char * tag) :
	        tag_(tag) {}
	virtual ~StubStorage() = default;

	amf_long AMF_STD_CALL Acquire() override
	{
		return ++refcount_;
	}
	amf_long AMF_STD_CALL Release() override
	{
		const amf_long n = --refcount_;
		if (n == 0)
			delete this;
		return n;
	}
	AMF_RESULT AMF_STD_CALL QueryInterface(const AMFGuid & id, void ** out) override
	{
		if (id == AMFPropertyStorage::IID() || id == AMFInterface::IID())
		{
			Acquire();
			*out = static_cast<AMFPropertyStorage *>(this);
			return AMF_OK;
		}
		*out = nullptr;
		return AMF_NO_INTERFACE;
	}

	AMF_RESULT AMF_STD_CALL SetProperty(const wchar_t * name, AMFVariantStruct value) override
	{
		properties_[narrow(name)] = value;
		report("%s.set %s = %s", tag_, narrow(name).c_str(), variant_text(value).c_str());
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL GetProperty(const wchar_t * name, AMFVariantStruct * value) const override
	{
		auto it = properties_.find(narrow(name));
		if (it == properties_.end())
			return AMF_NOT_FOUND;
		*value = it->second;
		return AMF_OK;
	}
	amf_bool AMF_STD_CALL HasProperty(const wchar_t * name) const override
	{
		return properties_.count(narrow(name)) != 0;
	}
	amf_size AMF_STD_CALL GetPropertyCount() const override
	{
		return properties_.size();
	}
	AMF_RESULT AMF_STD_CALL GetPropertyAt(amf_size, wchar_t *, amf_size, AMFVariantStruct *) const override
	{
		return AMF_NOT_IMPLEMENTED;
	}
	AMF_RESULT AMF_STD_CALL Clear() override
	{
		properties_.clear();
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL AddTo(AMFPropertyStorage *, amf_bool, amf_bool) const override
	{
		return AMF_NOT_IMPLEMENTED;
	}
	AMF_RESULT AMF_STD_CALL CopyTo(AMFPropertyStorage *, amf_bool) const override
	{
		return AMF_NOT_IMPLEMENTED;
	}
	void AMF_STD_CALL AddObserver(AMFPropertyStorageObserver *) override {}
	void AMF_STD_CALL RemoveObserver(AMFPropertyStorageObserver *) override {}

	void set_local(const wchar_t * name, const AMFVariantStruct & value)
	{
		properties_[narrow(name)] = value;
	}

protected:
	std::map<std::string, AMFVariantStruct> properties_;
	const char * tag_;
	amf_long refcount_ = 1;
};

// --------------------------------------------------------------------------

class StubPlane : public AMFPlane
{
public:
	StubPlane(amf_int32 w, amf_int32 h) :
	        width_(w), height_(h) {}
	virtual ~StubPlane() = default;

	amf_long AMF_STD_CALL Acquire() override
	{
		return ++refcount_;
	}
	amf_long AMF_STD_CALL Release() override
	{
		const amf_long n = --refcount_;
		if (n == 0)
			delete this;
		return n;
	}
	AMF_RESULT AMF_STD_CALL QueryInterface(const AMFGuid &, void ** out) override
	{
		*out = nullptr;
		return AMF_NO_INTERFACE;
	}

	AMF_PLANE_TYPE AMF_STD_CALL GetType() override
	{
		return AMF_PLANE_PACKED;
	}
	// The address the helper hands to CopySubresourceRegion. Not a texture, and
	// nothing under Wine ever dereferences it: the fill callback in the test is
	// what would have done the copy.
	void * AMF_STD_CALL GetNative() override
	{
		return reinterpret_cast<void *>(static_cast<uintptr_t>(0xD3D11000));
	}
	amf_int32 AMF_STD_CALL GetPixelSizeInBytes() override
	{
		return 4;
	}
	amf_int32 AMF_STD_CALL GetOffsetX() override
	{
		return 0;
	}
	amf_int32 AMF_STD_CALL GetOffsetY() override
	{
		return 0;
	}
	amf_int32 AMF_STD_CALL GetWidth() override
	{
		return width_;
	}
	amf_int32 AMF_STD_CALL GetHeight() override
	{
		return height_;
	}
	amf_int32 AMF_STD_CALL GetHPitch() override
	{
		return width_ * 4;
	}
	amf_int32 AMF_STD_CALL GetVPitch() override
	{
		return height_;
	}
	bool AMF_STD_CALL IsTiled() override
	{
		return false;
	}

private:
	amf_int32 width_;
	amf_int32 height_;
	amf_long refcount_ = 1;
};

class StubSurface : public AMFSurface, public StubStorage
{
public:
	StubSurface(AMF_SURFACE_FORMAT format, amf_int32 w, amf_int32 h) :
	        StubStorage("surface"), format_(format), plane_(new StubPlane(w, h)) {}
	~StubSurface() override
	{
		plane_->Release();
	}

	amf_long AMF_STD_CALL Acquire() override
	{
		return StubStorage::Acquire();
	}
	amf_long AMF_STD_CALL Release() override
	{
		return StubStorage::Release();
	}
	AMF_RESULT AMF_STD_CALL QueryInterface(const AMFGuid & id, void ** out) override
	{
		if (id == AMFSurface::IID())
		{
			Acquire();
			*out = static_cast<AMFSurface *>(this);
			return AMF_OK;
		}
		return StubStorage::QueryInterface(id, out);
	}

	AMF_RESULT AMF_STD_CALL SetProperty(const wchar_t * n, AMFVariantStruct v) override
	{
		return StubStorage::SetProperty(n, v);
	}
	AMF_RESULT AMF_STD_CALL GetProperty(const wchar_t * n, AMFVariantStruct * v) const override
	{
		return StubStorage::GetProperty(n, v);
	}
	amf_bool AMF_STD_CALL HasProperty(const wchar_t * n) const override
	{
		return StubStorage::HasProperty(n);
	}
	amf_size AMF_STD_CALL GetPropertyCount() const override
	{
		return StubStorage::GetPropertyCount();
	}
	AMF_RESULT AMF_STD_CALL GetPropertyAt(amf_size i, wchar_t * n, amf_size s, AMFVariantStruct * v) const override
	{
		return StubStorage::GetPropertyAt(i, n, s, v);
	}
	AMF_RESULT AMF_STD_CALL Clear() override
	{
		return StubStorage::Clear();
	}
	AMF_RESULT AMF_STD_CALL AddTo(AMFPropertyStorage * d, amf_bool o, amf_bool p) const override
	{
		return StubStorage::AddTo(d, o, p);
	}
	AMF_RESULT AMF_STD_CALL CopyTo(AMFPropertyStorage * d, amf_bool p) const override
	{
		return StubStorage::CopyTo(d, p);
	}
	void AMF_STD_CALL AddObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::AddObserver(o);
	}
	void AMF_STD_CALL RemoveObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::RemoveObserver(o);
	}

	// AMFData
	AMF_MEMORY_TYPE AMF_STD_CALL GetMemoryType() override
	{
		return AMF_MEMORY_DX11;
	}
	AMF_RESULT AMF_STD_CALL Duplicate(AMF_MEMORY_TYPE, AMFData **) override
	{
		return AMF_NOT_IMPLEMENTED;
	}
	AMF_RESULT AMF_STD_CALL Convert(AMF_MEMORY_TYPE) override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL Interop(AMF_MEMORY_TYPE) override
	{
		return AMF_OK;
	}
	AMF_DATA_TYPE AMF_STD_CALL GetDataType() override
	{
		return AMF_DATA_SURFACE;
	}
	amf_bool AMF_STD_CALL IsReusable() override
	{
		return true;
	}
	void AMF_STD_CALL SetPts(amf_pts pts) override
	{
		pts_ = pts;
	}
	amf_pts AMF_STD_CALL GetPts() override
	{
		return pts_;
	}
	void AMF_STD_CALL SetDuration(amf_pts d) override
	{
		duration_ = d;
	}
	amf_pts AMF_STD_CALL GetDuration() override
	{
		return duration_;
	}

	// AMFSurface
	AMF_SURFACE_FORMAT AMF_STD_CALL GetFormat() override
	{
		return format_;
	}
	amf_size AMF_STD_CALL GetPlanesCount() override
	{
		return 1;
	}
	AMFPlane * AMF_STD_CALL GetPlaneAt(amf_size index) override
	{
		return index == 0 ? plane_ : nullptr;
	}
	AMFPlane * AMF_STD_CALL GetPlane(AMF_PLANE_TYPE) override
	{
		return plane_;
	}
	AMF_FRAME_TYPE AMF_STD_CALL GetFrameType() override
	{
		return AMF_FRAME_PROGRESSIVE;
	}
	void AMF_STD_CALL SetFrameType(AMF_FRAME_TYPE) override {}
	AMF_RESULT AMF_STD_CALL SetCrop(amf_int32, amf_int32, amf_int32, amf_int32) override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL CopySurfaceRegion(AMFSurface *, amf_int32, amf_int32, amf_int32, amf_int32, amf_int32, amf_int32) override
	{
		return AMF_NOT_IMPLEMENTED;
	}
	void AMF_STD_CALL AddObserver(AMFSurfaceObserver *) override {}
	void AMF_STD_CALL RemoveObserver(AMFSurfaceObserver *) override {}

	using StubStorage::properties_;

private:
	AMF_SURFACE_FORMAT format_;
	StubPlane * plane_;
	amf_pts pts_ = 0;
	amf_pts duration_ = 0;
};

class StubBuffer : public AMFBuffer, public StubStorage
{
public:
	explicit StubBuffer(std::vector<uint8_t> && bytes) :
	        StubStorage("output"), bytes_(std::move(bytes)) {}

	amf_long AMF_STD_CALL Acquire() override
	{
		return StubStorage::Acquire();
	}
	amf_long AMF_STD_CALL Release() override
	{
		return StubStorage::Release();
	}
	AMF_RESULT AMF_STD_CALL QueryInterface(const AMFGuid & id, void ** out) override
	{
		if (id == AMFBuffer::IID())
		{
			Acquire();
			*out = static_cast<AMFBuffer *>(this);
			return AMF_OK;
		}
		if (id == AMFData::IID())
		{
			Acquire();
			*out = static_cast<AMFData *>(this);
			return AMF_OK;
		}
		return StubStorage::QueryInterface(id, out);
	}

	AMF_RESULT AMF_STD_CALL SetProperty(const wchar_t * n, AMFVariantStruct v) override
	{
		return StubStorage::SetProperty(n, v);
	}
	AMF_RESULT AMF_STD_CALL GetProperty(const wchar_t * n, AMFVariantStruct * v) const override
	{
		return StubStorage::GetProperty(n, v);
	}
	amf_bool AMF_STD_CALL HasProperty(const wchar_t * n) const override
	{
		return StubStorage::HasProperty(n);
	}
	amf_size AMF_STD_CALL GetPropertyCount() const override
	{
		return StubStorage::GetPropertyCount();
	}
	AMF_RESULT AMF_STD_CALL GetPropertyAt(amf_size i, wchar_t * n, amf_size s, AMFVariantStruct * v) const override
	{
		return StubStorage::GetPropertyAt(i, n, s, v);
	}
	AMF_RESULT AMF_STD_CALL Clear() override
	{
		return StubStorage::Clear();
	}
	AMF_RESULT AMF_STD_CALL AddTo(AMFPropertyStorage * d, amf_bool o, amf_bool p) const override
	{
		return StubStorage::AddTo(d, o, p);
	}
	AMF_RESULT AMF_STD_CALL CopyTo(AMFPropertyStorage * d, amf_bool p) const override
	{
		return StubStorage::CopyTo(d, p);
	}
	void AMF_STD_CALL AddObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::AddObserver(o);
	}
	void AMF_STD_CALL RemoveObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::RemoveObserver(o);
	}

	AMF_MEMORY_TYPE AMF_STD_CALL GetMemoryType() override
	{
		return AMF_MEMORY_HOST;
	}
	AMF_RESULT AMF_STD_CALL Duplicate(AMF_MEMORY_TYPE, AMFData **) override
	{
		return AMF_NOT_IMPLEMENTED;
	}
	AMF_RESULT AMF_STD_CALL Convert(AMF_MEMORY_TYPE) override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL Interop(AMF_MEMORY_TYPE) override
	{
		return AMF_OK;
	}
	AMF_DATA_TYPE AMF_STD_CALL GetDataType() override
	{
		return AMF_DATA_BUFFER;
	}
	amf_bool AMF_STD_CALL IsReusable() override
	{
		return false;
	}
	void AMF_STD_CALL SetPts(amf_pts pts) override
	{
		pts_ = pts;
	}
	amf_pts AMF_STD_CALL GetPts() override
	{
		return pts_;
	}
	void AMF_STD_CALL SetDuration(amf_pts d) override
	{
		duration_ = d;
	}
	amf_pts AMF_STD_CALL GetDuration() override
	{
		return duration_;
	}

	AMF_RESULT AMF_STD_CALL SetSize(amf_size n) override
	{
		bytes_.resize(n);
		return AMF_OK;
	}
	amf_size AMF_STD_CALL GetSize() override
	{
		return bytes_.size();
	}
	void * AMF_STD_CALL GetNative() override
	{
		return bytes_.data();
	}
	void AMF_STD_CALL AddObserver(AMFBufferObserver *) override {}
	void AMF_STD_CALL RemoveObserver(AMFBufferObserver *) override {}

private:
	std::vector<uint8_t> bytes_;
	amf_pts pts_ = 0;
	amf_pts duration_ = 0;
};

class StubCaps : public AMFCaps, public StubStorage
{
public:
	StubCaps() :
	        StubStorage("caps") {}

	amf_long AMF_STD_CALL Acquire() override
	{
		return StubStorage::Acquire();
	}
	amf_long AMF_STD_CALL Release() override
	{
		return StubStorage::Release();
	}
	AMF_RESULT AMF_STD_CALL QueryInterface(const AMFGuid & id, void ** out) override
	{
		if (id == AMFCaps::IID())
		{
			Acquire();
			*out = static_cast<AMFCaps *>(this);
			return AMF_OK;
		}
		return StubStorage::QueryInterface(id, out);
	}

	AMF_RESULT AMF_STD_CALL SetProperty(const wchar_t * n, AMFVariantStruct v) override
	{
		return StubStorage::SetProperty(n, v);
	}
	// Deliberately silent: reading a capability is not something to report.
	AMF_RESULT AMF_STD_CALL GetProperty(const wchar_t * n, AMFVariantStruct * v) const override
	{
		return StubStorage::GetProperty(n, v);
	}
	amf_bool AMF_STD_CALL HasProperty(const wchar_t * n) const override
	{
		return StubStorage::HasProperty(n);
	}
	amf_size AMF_STD_CALL GetPropertyCount() const override
	{
		return StubStorage::GetPropertyCount();
	}
	AMF_RESULT AMF_STD_CALL GetPropertyAt(amf_size i, wchar_t * n, amf_size s, AMFVariantStruct * v) const override
	{
		return StubStorage::GetPropertyAt(i, n, s, v);
	}
	AMF_RESULT AMF_STD_CALL Clear() override
	{
		return StubStorage::Clear();
	}
	AMF_RESULT AMF_STD_CALL AddTo(AMFPropertyStorage * d, amf_bool o, amf_bool p) const override
	{
		return StubStorage::AddTo(d, o, p);
	}
	AMF_RESULT AMF_STD_CALL CopyTo(AMFPropertyStorage * d, amf_bool p) const override
	{
		return StubStorage::CopyTo(d, p);
	}
	void AMF_STD_CALL AddObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::AddObserver(o);
	}
	void AMF_STD_CALL RemoveObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::RemoveObserver(o);
	}

	AMF_ACCELERATION_TYPE AMF_STD_CALL GetAccelerationType() const override
	{
		return AMF_ACCEL_HARDWARE;
	}
	AMF_RESULT AMF_STD_CALL GetInputCaps(AMFIOCaps **) override
	{
		return AMF_NOT_IMPLEMENTED;
	}
	AMF_RESULT AMF_STD_CALL GetOutputCaps(AMFIOCaps **) override
	{
		return AMF_NOT_IMPLEMENTED;
	}
};

class StubContext;

class StubComponent : public AMFComponent, public StubStorage
{
public:
	StubComponent(StubContext * context, bool hevc) :
	        StubStorage(hevc ? "hevc" : "avc"), context_(context), hevc_(hevc) {}

	amf_long AMF_STD_CALL Acquire() override
	{
		return StubStorage::Acquire();
	}
	amf_long AMF_STD_CALL Release() override
	{
		return StubStorage::Release();
	}
	AMF_RESULT AMF_STD_CALL QueryInterface(const AMFGuid & id, void ** out) override
	{
		if (id == AMFComponent::IID())
		{
			Acquire();
			*out = static_cast<AMFComponent *>(this);
			return AMF_OK;
		}
		return StubStorage::QueryInterface(id, out);
	}

	AMF_RESULT AMF_STD_CALL SetProperty(const wchar_t * n, AMFVariantStruct v) override
	{
		return StubStorage::SetProperty(n, v);
	}
	AMF_RESULT AMF_STD_CALL GetProperty(const wchar_t * n, AMFVariantStruct * v) const override
	{
		return StubStorage::GetProperty(n, v);
	}
	amf_bool AMF_STD_CALL HasProperty(const wchar_t * n) const override
	{
		return StubStorage::HasProperty(n);
	}
	amf_size AMF_STD_CALL GetPropertyCount() const override
	{
		return StubStorage::GetPropertyCount();
	}
	AMF_RESULT AMF_STD_CALL GetPropertyAt(amf_size i, wchar_t * n, amf_size s, AMFVariantStruct * v) const override
	{
		return StubStorage::GetPropertyAt(i, n, s, v);
	}
	AMF_RESULT AMF_STD_CALL Clear() override
	{
		return StubStorage::Clear();
	}
	AMF_RESULT AMF_STD_CALL AddTo(AMFPropertyStorage * d, amf_bool o, amf_bool p) const override
	{
		return StubStorage::AddTo(d, o, p);
	}
	AMF_RESULT AMF_STD_CALL CopyTo(AMFPropertyStorage * d, amf_bool p) const override
	{
		return StubStorage::CopyTo(d, p);
	}
	void AMF_STD_CALL AddObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::AddObserver(o);
	}
	void AMF_STD_CALL RemoveObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::RemoveObserver(o);
	}

	// AMFPropertyStorageEx
	amf_size AMF_STD_CALL GetPropertiesInfoCount() const override
	{
		return 0;
	}
	AMF_RESULT AMF_STD_CALL GetPropertyInfo(amf_size, const AMFPropertyInfo **) const override
	{
		return AMF_NOT_FOUND;
	}
	AMF_RESULT AMF_STD_CALL GetPropertyInfo(const wchar_t *, const AMFPropertyInfo **) const override
	{
		return AMF_NOT_FOUND;
	}
	AMF_RESULT AMF_STD_CALL ValidateProperty(const wchar_t *, AMFVariantStruct value, AMFVariantStruct * out) const override
	{
		*out = value;
		return AMF_OK;
	}

	// AMFComponent
	AMF_RESULT AMF_STD_CALL Init(AMF_SURFACE_FORMAT format, amf_int32 width, amf_int32 height) override
	{
		report("%s.init format=%d %dx%d", tag_, static_cast<int>(format), width, height);
		initialised_ = true;
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL ReInit(amf_int32, amf_int32) override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL Terminate() override
	{
		report("%s.terminate", tag_);
		initialised_ = false;
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL Drain() override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL Flush() override
	{
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL SubmitInput(AMFData * data) override
	{
		if (not initialised_)
			return AMF_NOT_INITIALIZED;
		if (pending_ != nullptr)
			return AMF_INPUT_FULL;

		// The IDR request rides on the surface, not on the component: pick it
		// back off exactly where the helper put it.
		bool idr = false;
		AMFVariantStruct v{};
		const wchar_t * force = hevc_ ? AMF_VIDEO_ENCODER_HEVC_FORCE_PICTURE_TYPE
		                              : AMF_VIDEO_ENCODER_FORCE_PICTURE_TYPE;
		if (data->GetProperty(force, &v) == AMF_OK)
			idr = true;

		// Carry the caller's own properties through to the output, which is what
		// a real AMF component does with anything it does not recognise.
		AMFVariantStruct submit{};
		const bool have_submit = data->GetProperty(L"WivrnnxSubmitTicks", &submit) == AMF_OK;

		report("%s.submit idr=%d", tag_, idr ? 1 : 0);

		std::vector<uint8_t> bytes = make_bitstream(idr);
		auto * buffer = new StubBuffer(std::move(bytes));

		AMFVariantStruct type{};
		type.type = AMF_VARIANT_INT64;
		if (idr)
			type.int64Value = hevc_ ? amf_int64(AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE_IDR)
			                        : amf_int64(AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE_IDR);
		else
			type.int64Value = hevc_ ? amf_int64(AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE_P)
			                        : amf_int64(AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE_P);
		buffer->set_local(hevc_ ? AMF_VIDEO_ENCODER_HEVC_OUTPUT_DATA_TYPE
		                        : AMF_VIDEO_ENCODER_OUTPUT_DATA_TYPE,
		                  type);
		if (have_submit)
			buffer->set_local(L"WivrnnxSubmitTicks", submit);

		pending_ = buffer;
		repeats_left_ = env_int("WIVRNNX_STUB_REPEATS", 2);
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL QueryOutput(AMFData ** out) override
	{
		*out = nullptr;
		if (pending_ == nullptr)
			return AMF_REPEAT;
		if (env_set("WIVRNNX_STUB_NO_OUTPUT"))
			return AMF_REPEAT;
		if (repeats_left_-- > 0)
		{
			// A real encoder is asynchronous; the helper's poll loop has to
			// tolerate being told "not yet" an arbitrary number of times.
			Sleep(1);
			return AMF_REPEAT;
		}
		*out = pending_;
		pending_ = nullptr;
		return AMF_OK;
	}

	AMFContext * AMF_STD_CALL GetContext() override;

	AMF_RESULT AMF_STD_CALL SetOutputDataAllocatorCB(AMFDataAllocatorCB *) override
	{
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL GetCaps(AMFCaps ** caps) override
	{
		auto * c = new StubCaps();
		AMFVariantStruct v{};
		v.type = AMF_VARIANT_BOOL;
		v.boolValue = env_int("WIVRNNX_STUB_QUERY_TIMEOUT", 1) != 0;
		c->set_local(hevc_ ? AMF_VIDEO_ENCODER_CAPS_HEVC_QUERY_TIMEOUT_SUPPORT
		                   : AMF_VIDEO_ENCODER_CAPS_QUERY_TIMEOUT_SUPPORT,
		             v);
		*caps = c;
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL Optimize(AMFComponentOptimizationCallback *) override
	{
		return AMF_OK;
	}

private:
	// A three-NAL annex-B access unit for a P frame, six for an IDR (parameter
	// sets in front). Only the shape matters.
	static std::vector<uint8_t> make_bitstream(bool idr)
	{
		std::vector<uint8_t> out;
		auto nal = [&](uint8_t type, size_t n) {
			out.insert(out.end(), {0x00, 0x00, 0x00, 0x01});
			out.push_back(static_cast<uint8_t>(type << 1));
			out.push_back(0x01);
			for (size_t i = 0; i < n; ++i)
				out.push_back(static_cast<uint8_t>(0x40 + (i & 0x3f)));
		};
		if (idr)
		{
			nal(32, 16); // VPS
			nal(33, 32); // SPS
			nal(34, 8);  // PPS
			nal(19, 4096);
		}
		else
		{
			nal(1, 1024);
		}
		return out;
	}

	StubContext * context_;
	bool hevc_;
	bool initialised_ = false;
	StubBuffer * pending_ = nullptr;
	int repeats_left_ = 0;
};

class StubContext : public AMFContext, public StubStorage
{
public:
	StubContext() :
	        StubStorage("context") {}

	amf_long AMF_STD_CALL Acquire() override
	{
		return StubStorage::Acquire();
	}
	amf_long AMF_STD_CALL Release() override
	{
		return StubStorage::Release();
	}
	AMF_RESULT AMF_STD_CALL QueryInterface(const AMFGuid & id, void ** out) override
	{
		if (id == AMFContext::IID())
		{
			Acquire();
			*out = static_cast<AMFContext *>(this);
			return AMF_OK;
		}
		return StubStorage::QueryInterface(id, out);
	}

	AMF_RESULT AMF_STD_CALL SetProperty(const wchar_t * n, AMFVariantStruct v) override
	{
		return StubStorage::SetProperty(n, v);
	}
	AMF_RESULT AMF_STD_CALL GetProperty(const wchar_t * n, AMFVariantStruct * v) const override
	{
		return StubStorage::GetProperty(n, v);
	}
	amf_bool AMF_STD_CALL HasProperty(const wchar_t * n) const override
	{
		return StubStorage::HasProperty(n);
	}
	amf_size AMF_STD_CALL GetPropertyCount() const override
	{
		return StubStorage::GetPropertyCount();
	}
	AMF_RESULT AMF_STD_CALL GetPropertyAt(amf_size i, wchar_t * n, amf_size s, AMFVariantStruct * v) const override
	{
		return StubStorage::GetPropertyAt(i, n, s, v);
	}
	AMF_RESULT AMF_STD_CALL Clear() override
	{
		return StubStorage::Clear();
	}
	AMF_RESULT AMF_STD_CALL AddTo(AMFPropertyStorage * d, amf_bool o, amf_bool p) const override
	{
		return StubStorage::AddTo(d, o, p);
	}
	AMF_RESULT AMF_STD_CALL CopyTo(AMFPropertyStorage * d, amf_bool p) const override
	{
		return StubStorage::CopyTo(d, p);
	}
	void AMF_STD_CALL AddObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::AddObserver(o);
	}
	void AMF_STD_CALL RemoveObserver(AMFPropertyStorageObserver * o) override
	{
		StubStorage::RemoveObserver(o);
	}

	AMF_RESULT AMF_STD_CALL Terminate() override
	{
		report("context.terminate");
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL InitDX9(void *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	void * AMF_STD_CALL GetDX9Device(AMF_DX_VERSION) override
	{
		return nullptr;
	}
	AMF_RESULT AMF_STD_CALL LockDX9() override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL UnlockDX9() override
	{
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL InitDX11(void * device, AMF_DX_VERSION) override
	{
		report("context.initdx11 device=%s", device != nullptr ? "non-null" : "null");
		dx11_ = device;
		return AMF_OK;
	}
	void * AMF_STD_CALL GetDX11Device(AMF_DX_VERSION) override
	{
		return dx11_;
	}
	AMF_RESULT AMF_STD_CALL LockDX11() override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL UnlockDX11() override
	{
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL InitOpenCL(void *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	void * AMF_STD_CALL GetOpenCLContext() override
	{
		return nullptr;
	}
	void * AMF_STD_CALL GetOpenCLCommandQueue() override
	{
		return nullptr;
	}
	void * AMF_STD_CALL GetOpenCLDeviceID() override
	{
		return nullptr;
	}
	AMF_RESULT AMF_STD_CALL GetOpenCLComputeFactory(AMFComputeFactory **) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL InitOpenCLEx(AMFComputeDevice *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL LockOpenCL() override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL UnlockOpenCL() override
	{
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL InitOpenGL(amf_handle, amf_handle, amf_handle) override
	{
		return AMF_NOT_SUPPORTED;
	}
	amf_handle AMF_STD_CALL GetOpenGLContext() override
	{
		return nullptr;
	}
	amf_handle AMF_STD_CALL GetOpenGLDrawable() override
	{
		return nullptr;
	}
	AMF_RESULT AMF_STD_CALL LockOpenGL() override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL UnlockOpenGL() override
	{
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL InitXV(void *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	void * AMF_STD_CALL GetXVDevice() override
	{
		return nullptr;
	}
	AMF_RESULT AMF_STD_CALL LockXV() override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL UnlockXV() override
	{
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL InitGralloc(void *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	void * AMF_STD_CALL GetGrallocDevice() override
	{
		return nullptr;
	}
	AMF_RESULT AMF_STD_CALL LockGralloc() override
	{
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL UnlockGralloc() override
	{
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL AllocBuffer(AMF_MEMORY_TYPE, amf_size size, AMFBuffer ** out) override
	{
		*out = new StubBuffer(std::vector<uint8_t>(size));
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL AllocSurface(AMF_MEMORY_TYPE type,
	                                     AMF_SURFACE_FORMAT format,
	                                     amf_int32 width,
	                                     amf_int32 height,
	                                     AMFSurface ** out) override
	{
		report("context.allocsurface memory=%d format=%d %dx%d",
		       static_cast<int>(type),
		       static_cast<int>(format),
		       width,
		       height);
		*out = new StubSurface(format, width, height);
		return AMF_OK;
	}
	AMF_RESULT AMF_STD_CALL AllocAudioBuffer(AMF_MEMORY_TYPE, AMF_AUDIO_FORMAT, amf_int32, amf_int32, amf_int32, AMFAudioBuffer **) override
	{
		return AMF_NOT_SUPPORTED;
	}

	AMF_RESULT AMF_STD_CALL CreateBufferFromHostNative(void *, amf_size, AMFBuffer **, AMFBufferObserver *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL CreateSurfaceFromHostNative(AMF_SURFACE_FORMAT, amf_int32, amf_int32, amf_int32, amf_int32, void *, AMFSurface **, AMFSurfaceObserver *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL CreateSurfaceFromOpenCLNative(AMF_SURFACE_FORMAT, amf_int32, amf_int32, void **, AMFSurface **, AMFSurfaceObserver *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL CreateSurfaceFromDX9Native(void *, AMFSurface **, AMFSurfaceObserver *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL CreateSurfaceFromDX11Native(void *, AMFSurface **, AMFSurfaceObserver *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL CreateSurfaceFromOpenGLNative(AMF_SURFACE_FORMAT, amf_handle, AMFSurface **, AMFSurfaceObserver *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL CreateSurfaceFromGrallocNative(amf_handle, AMFSurface **, AMFSurfaceObserver *) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL CreateBufferFromOpenCLNative(void *, amf_size, AMFBuffer **) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL GetCompute(AMF_MEMORY_TYPE, AMFCompute **) override
	{
		return AMF_NOT_SUPPORTED;
	}

private:
	void * dx11_ = nullptr;
};

AMFContext * AMF_STD_CALL StubComponent::GetContext()
{
	return context_;
}

class StubFactory : public AMFFactory
{
public:
	AMF_RESULT AMF_STD_CALL CreateContext(AMFContext ** out) override
	{
		report("factory.createcontext");
		*out = new StubContext();
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL CreateComponent(AMFContext * context, const wchar_t * id, AMFComponent ** out) override
	{
		const std::string name = narrow(id);
		const bool hevc = name == narrow(AMFVideoEncoder_HEVC);
		const bool avc = name == narrow(AMFVideoEncoderVCE_AVC);
		report("factory.createcomponent %s", name.c_str());

		if (hevc && env_set("WIVRNNX_STUB_NO_HEVC"))
			return AMF_NOT_SUPPORTED;
		if (avc && env_set("WIVRNNX_STUB_NO_H264"))
			return AMF_NOT_SUPPORTED;
		if (not hevc && not avc)
			return AMF_NOT_FOUND;

		*out = new StubComponent(static_cast<StubContext *>(context), hevc);
		return AMF_OK;
	}

	AMF_RESULT AMF_STD_CALL SetCacheFolder(const wchar_t *) override
	{
		return AMF_OK;
	}
	const wchar_t * AMF_STD_CALL GetCacheFolder() override
	{
		return L"";
	}
	AMF_RESULT AMF_STD_CALL GetDebug(AMFDebug **) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL GetTrace(AMFTrace **) override
	{
		return AMF_NOT_SUPPORTED;
	}
	AMF_RESULT AMF_STD_CALL GetPrograms(AMFPrograms **) override
	{
		return AMF_NOT_SUPPORTED;
	}
};

StubFactory g_factory;

amf_uint64 stub_version()
{
	std::string v;
	unsigned a = 1, b = 4, c = 30, d = 0;
	if (env_get("WIVRNNX_STUB_VERSION", v))
		std::sscanf(v.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d);
	return AMF_MAKE_FULL_VERSION(a, b, c, d);
}

} // namespace

extern "C"
{

__declspec(dllexport) AMF_RESULT AMF_CDECL_CALL AMFQueryVersion(amf_uint64 * version)
{
	*version = stub_version();
	return AMF_OK;
}

__declspec(dllexport) AMF_RESULT AMF_CDECL_CALL AMFInit(amf_uint64 version, AMFFactory ** factory)
{
	// A real runtime refuses a caller that asks for a contract newer than its
	// own. That is exactly the case the loader's version negotiation exists to
	// avoid, so the stub enforces it.
	if (version > stub_version())
	{
		report("init refused: asked %llu.%llu.%llu.%llu, runtime %llu.%llu.%llu.%llu",
		       (unsigned long long)AMF_GET_MAJOR_VERSION(version),
		       (unsigned long long)AMF_GET_MINOR_VERSION(version),
		       (unsigned long long)AMF_GET_SUBMINOR_VERSION(version),
		       (unsigned long long)AMF_GET_BUILD_VERSION(version),
		       (unsigned long long)AMF_GET_MAJOR_VERSION(stub_version()),
		       (unsigned long long)AMF_GET_MINOR_VERSION(stub_version()),
		       (unsigned long long)AMF_GET_SUBMINOR_VERSION(stub_version()),
		       (unsigned long long)AMF_GET_BUILD_VERSION(stub_version()));
		return AMF_NOT_SUPPORTED;
	}

	report("init %llu.%llu.%llu.%llu",
	       (unsigned long long)AMF_GET_MAJOR_VERSION(version),
	       (unsigned long long)AMF_GET_MINOR_VERSION(version),
	       (unsigned long long)AMF_GET_SUBMINOR_VERSION(version),
	       (unsigned long long)AMF_GET_BUILD_VERSION(version));
	*factory = &g_factory;
	return AMF_OK;
}

// Everything the stub was asked, newline separated, oldest first. The test reads
// it back through GetProcAddress on the same module the loader opened.
__declspec(dllexport) const char * WivrnnxStubReport()
{
	return g_report.c_str();
}

__declspec(dllexport) void WivrnnxStubReset()
{
	g_report.clear();
}

} // extern "C"
