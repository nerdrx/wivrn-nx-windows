#include "direct_mode.h"

#include "breadcrumb.h"
#include "driverlog.h"
#include "hmd_device.h"
#include "ipc_client.h"
#include "vr_context.h"
#include "vr_util.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <d3d11.h>
#include <dxgi.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <new>

namespace wivrnnx
{
namespace
{

// Give up creating the D3D11 device after this many failures rather than
// hammering the driver once a frame forever.
constexpr int kMaxDeviceAttempts = 3;

// A misbehaving compositor must not be able to make us allocate without bound.
// Two eyes times a handful of applications is well inside this.
constexpr size_t kMaxTextureSets = 64;

// ALVR uses 10 ms (OvrDirectModeComponent.cpp:245). Same here: the point of the
// acquire is to serialise against the compositor's rendering, and blocking the
// compositor's own present thread for longer than a frame would be worse than
// dropping the frame.
constexpr DWORD kAcquireSyncTimeoutMs = 10;

// Clamp for the vsync pace. A garbage refresh_hz (0, NaN, 10000) must not turn
// into a spin loop or a stall.
constexpr double kMinRefreshHz = 30.0;
constexpr double kMaxRefreshHz = 240.0;

// Breadcrumb the first call and then every kLogEvery-th one: these are per-frame
// entry points and the log file is append-and-flush.
constexpr uint64_t kLogEvery = 1000;

// Dropped frames are rarer and more interesting than presents, so they get a
// tighter rate.
constexpr uint64_t kLogDropsEvery = 100;

// Ring size. Three is the contract's maximum and the useful number: one being
// encoded by the helper, one in flight, one for us to write.
constexpr uint32_t kStagingSlots = 3;

// Ring creations to attempt before giving up for the rest of the session, same
// reasoning as kMaxDeviceAttempts.
constexpr int kMaxRingAttempts = 3;

bool should_log(uint64_t count) noexcept
{
	return count == 1 || (count % kLogEvery) == 0;
}

// True the first time it is called on a given flag, false afterwards: for the
// "tell me once what the real compositor does" breadcrumbs.
bool first_time(std::atomic<bool> & flag) noexcept
{
	return !flag.exchange(true, std::memory_order_relaxed);
}

// Where one eye's pixels actually live inside the texture that was submitted.
struct SubRect
{
	uint32_t x = 0;
	uint32_t y = 0;
	uint32_t w = 0;
	uint32_t h = 0;
};

// NaN-safe: every comparison against NaN is false, so NaN lands on 0.
float clamp01(float v) noexcept
{
	if (v > 1.0f)
		return 1.0f;
	if (v > 0.0f)
		return v;
	return 0.0f;
}

// VRTextureBounds_t is normalised (0..1) texture coordinates into the submitted
// texture. It is the full texture for every application seen so far, but SteamVR
// uses it for dynamic resolution scaling and for applications that pack both
// eyes into one target, so the sub-rect is honoured rather than assumed.
//
// `partial` says the bounds were not the whole texture; `flipped` says they were
// given bottom-up (vMin > vMax), which a plain copy cannot express -- there is no
// flip in CopySubresourceRegion. Both are reported to the caller so they can be
// breadcrumbed once, because both would show up as a subtly wrong image rather
// than as a failure.
SubRect rect_from_bounds(const vr::VRTextureBounds_t & bounds,
                         uint32_t texture_width,
                         uint32_t texture_height,
                         bool & partial,
                         bool & flipped) noexcept
{
	const float u_min = clamp01(bounds.uMin);
	const float u_max = clamp01(bounds.uMax);
	const float v_min = clamp01(bounds.vMin);
	const float v_max = clamp01(bounds.vMax);

	flipped = bounds.vMin > bounds.vMax || bounds.uMin > bounds.uMax;

	const float u0 = std::min(u_min, u_max);
	const float u1 = std::max(u_min, u_max);
	const float v0 = std::min(v_min, v_max);
	const float v1 = std::max(v_min, v_max);

	constexpr float kEpsilon = 1.0f / 4096.0f;
	partial = u0 > kEpsilon || v0 > kEpsilon || u1 < 1.0f - kEpsilon || v1 < 1.0f - kEpsilon;

	auto to_texels = [](float normalised, uint32_t extent) noexcept -> uint32_t {
		const double texel = static_cast<double>(normalised) * static_cast<double>(extent);
		if (!(texel > 0.0))
			return 0;
		const uint32_t rounded = static_cast<uint32_t>(texel + 0.5);
		return rounded > extent ? extent : rounded;
	};

	SubRect rect;
	rect.x = to_texels(u0, texture_width);
	rect.y = to_texels(v0, texture_height);
	const uint32_t x1 = to_texels(u1, texture_width);
	const uint32_t y1 = to_texels(v1, texture_height);
	rect.w = x1 > rect.x ? x1 - rect.x : 0;
	rect.h = y1 > rect.y ? y1 - rect.y : 0;
	return rect;
}

// The staging textures carry the submitted format verbatim -- the contract
// stores a DXGI_FORMAT and the helper decides how to interpret it, and
// CopySubresourceRegion converts nothing, so anything else would be a lie about
// the bits.
//
// The single exception is a TYPELESS submitted format: a typeless shared surface
// is not something the helper can build a view on, and the copy is still legal
// because a typeless format and its typed members are copy-compatible. Which
// concrete member to pick is a guess, so it is breadcrumbed when it happens.
uint32_t staging_format_for(uint32_t submitted, bool & changed) noexcept
{
	changed = true;
	switch (static_cast<DXGI_FORMAT>(submitted))
	{
		case DXGI_FORMAT_R8G8B8A8_TYPELESS:
			return DXGI_FORMAT_R8G8B8A8_UNORM;
		case DXGI_FORMAT_B8G8R8A8_TYPELESS:
			return DXGI_FORMAT_B8G8R8A8_UNORM;
		case DXGI_FORMAT_B8G8R8X8_TYPELESS:
			return DXGI_FORMAT_B8G8R8X8_UNORM;
		case DXGI_FORMAT_R10G10B10A2_TYPELESS:
			return DXGI_FORMAT_R10G10B10A2_UNORM;
		case DXGI_FORMAT_R16G16B16A16_TYPELESS:
			return DXGI_FORMAT_R16G16B16A16_FLOAT;
		default:
			changed = false;
			return submitted;
	}
}

// DXGI_ADAPTER_DESC::Description is WCHAR[128].
void narrow(const WCHAR * wide, char * out, size_t out_size) noexcept
{
	if (out == nullptr || out_size == 0)
		return;
	out[0] = '\0';
	if (wide == nullptr)
		return;
	const int written = ::WideCharToMultiByte(CP_UTF8,
	                                          0,
	                                          wide,
	                                          -1,
	                                          out,
	                                          static_cast<int>(out_size),
	                                          nullptr,
	                                          nullptr);
	if (written <= 0)
		out[0] = '\0';
	out[out_size - 1] = '\0';
}

template <typename T>
void safe_release(T *& p) noexcept
{
	if (p != nullptr)
	{
		p->Release();
		p = nullptr;
	}
}

} // namespace

// One swap chain: three textures the compositor renders into, round-robin.
struct DirectModeComponent::TextureSet
{
	uint32_t pid = 0;
	ID3D11Texture2D * textures[3] = {nullptr, nullptr, nullptr};
	vr::SharedTextureHandle_t handles[3] = {0, 0, 0};
	// Index handed back by GetNextSwapTextureSetIndex, 0 -> 1 -> 2 -> 0.
	uint32_t index = 0;
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t format = 0;
	uint32_t sample_count = 0;
};

// Cache entry for a handle the *compositor* created and we opened (the sync
// texture). Mirrors CD3DRender::SharedTextureEntry_t.
struct DirectModeComponent::SharedTextureEntry
{
	void * handle = nullptr;
	ID3D11Texture2D * texture = nullptr;
};

// The last SubmitLayer, copied out under present_mutex_ so Present can do the
// slow parts (D3D, IPC) without holding the lock SubmitLayer needs.
struct DirectModeComponent::FrameSnapshot
{
	vr::SharedTextureHandle_t texture[2] = {0, 0};
	vr::VRTextureBounds_t bounds[2] = {};
	vr::HmdMatrix34_t pose = {};
	float predict = 0.0f;
	uint64_t qpc = 0;
	uint32_t layers = 0;
};

DirectModeComponent::DirectModeComponent(HmdDevice * owner) noexcept :
        owner_(owner)
{
}

DirectModeComponent::~DirectModeComponent()
{
	shutdown();
}

// ---------------------------------------------------------------------------
// D3D11
// ---------------------------------------------------------------------------

bool DirectModeComponent::ensure_device_locked() noexcept
{
	if (device_ != nullptr)
		return true;
	if (device_attempts_ >= kMaxDeviceAttempts)
		return false;

	++device_attempts_;

	// Default adapter (nullptr + D3D_DRIVER_TYPE_HARDWARE), which is adapter 0.
	// ALVR reaches the same place the long way round, enumerating DXGI adapters
	// and taking index 0, and explains why in HMD.cpp:114-122: vrcompositor
	// always picks the first adapter, and a driver on a different adapter makes
	// it fail to open the shared textures. Prop_GraphicsAdapterLuid_Uint64 is
	// ignored for direct-mode drivers, so there is nothing to negotiate.
	// The test box is a single-GPU RX 580, so adapter 0 is the only adapter.
	const UINT flags = D3D11_CREATE_DEVICE_BGRA_SUPPORT;

	static const D3D_FEATURE_LEVEL kLevels[] = {
	        D3D_FEATURE_LEVEL_11_1,
	        D3D_FEATURE_LEVEL_11_0,
	        D3D_FEATURE_LEVEL_10_1,
	        D3D_FEATURE_LEVEL_10_0,
	};

	D3D_FEATURE_LEVEL level = D3D_FEATURE_LEVEL_11_0;
	HRESULT hr = ::D3D11CreateDevice(nullptr,
	                                 D3D_DRIVER_TYPE_HARDWARE,
	                                 nullptr,
	                                 flags,
	                                 kLevels,
	                                 static_cast<UINT>(sizeof(kLevels) / sizeof(kLevels[0])),
	                                 D3D11_SDK_VERSION,
	                                 &device_,
	                                 &level,
	                                 &context_);

	if (hr == E_INVALIDARG)
	{
		// A runtime without the 11.1 platform update rejects the whole array
		// rather than falling back. Retry without 11_1.
		breadcrumb("direct: D3D11CreateDevice rejected FEATURE_LEVEL_11_1, retrying from 11_0");
		hr = ::D3D11CreateDevice(nullptr,
		                         D3D_DRIVER_TYPE_HARDWARE,
		                         nullptr,
		                         flags,
		                         kLevels + 1,
		                         static_cast<UINT>(sizeof(kLevels) / sizeof(kLevels[0]) - 1),
		                         D3D11_SDK_VERSION,
		                         &device_,
		                         &level,
		                         &context_);
	}

	if (FAILED(hr) || device_ == nullptr)
	{
		device_ = nullptr;
		safe_release(context_);
		breadcrumb("direct: D3D11CreateDevice failed hr=0x%08lX (attempt %d of %d), "
		           "direct mode will stay inert",
		           static_cast<unsigned long>(hr),
		           device_attempts_,
		           kMaxDeviceAttempts);
		WNX_LOG("direct: D3D11CreateDevice failed (0x%08lX)", static_cast<unsigned long>(hr));
		return false;
	}

	// Say which GPU we landed on: if this ever disagrees with the adapter
	// vrcompositor picked, "failed to open shared texture" is the symptom.
	char description[256] = "?";
	UINT vendor_id = 0;
	UINT device_id = 0;
	IDXGIDevice * dxgi_device = nullptr;
	if (SUCCEEDED(device_->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void **>(&dxgi_device))) &&
	    dxgi_device != nullptr)
	{
		IDXGIAdapter * adapter = nullptr;
		if (SUCCEEDED(dxgi_device->GetAdapter(&adapter)) && adapter != nullptr)
		{
			DXGI_ADAPTER_DESC desc{};
			if (SUCCEEDED(adapter->GetDesc(&desc)))
			{
				narrow(desc.Description, description, sizeof(description));
				vendor_id = desc.VendorId;
				device_id = desc.DeviceId;
			}
			safe_release(adapter);
		}
		safe_release(dxgi_device);
	}

	breadcrumb("direct: D3D11 device created on adapter \"%s\" (vendor 0x%04X device 0x%04X), "
	           "feature level 0x%04X",
	           description,
	           vendor_id,
	           device_id,
	           static_cast<unsigned>(level));
	WNX_LOG("direct: D3D11 device on \"%s\", feature level 0x%04X",
	        description,
	        static_cast<unsigned>(level));
	return true;
}

ID3D11Texture2D * DirectModeComponent::shared_texture_locked(void * handle) noexcept
{
	if (handle == nullptr)
		return nullptr;
	if (!ensure_device_locked())
		return nullptr;

	for (const SharedTextureEntry & entry: shared_textures_)
	{
		if (entry.handle == handle)
			return entry.texture;
	}

	ID3D11Texture2D * texture = nullptr;
	const HRESULT hr = device_->OpenSharedResource(handle,
	                                               __uuidof(ID3D11Texture2D),
	                                               reinterpret_cast<void **>(&texture));
	if (FAILED(hr) || texture == nullptr)
	{
		breadcrumb("direct: OpenSharedResource(%p) failed hr=0x%08lX", handle, static_cast<unsigned long>(hr));
		return nullptr;
	}

	// Cached for the lifetime of the device, exactly like ALVR's
	// CD3DRender::GetSharedTexture: the compositor reuses a small set of sync
	// textures, and re-opening one every frame would leak an object a frame.
	try
	{
		shared_textures_.push_back(SharedTextureEntry{handle, texture});
	}
	catch (...)
	{
		// Out of memory: usable this frame, just not cached.
		breadcrumb("direct: could not cache shared texture %p", handle);
		return texture;
	}

	breadcrumb("direct: opened shared texture %p (cache now %zu)", handle, shared_textures_.size());
	return texture;
}

void DirectModeComponent::release_device_locked() noexcept
{
	for (SharedTextureEntry & entry: shared_textures_)
		safe_release(entry.texture);
	shared_textures_.clear();

	if (context_ != nullptr)
		context_->ClearState();
	safe_release(context_);
	safe_release(device_);
}

// ---------------------------------------------------------------------------
// Swap texture sets
// ---------------------------------------------------------------------------

DirectModeComponent::TextureSet * DirectModeComponent::find_set_locked(vr::SharedTextureHandle_t handle) noexcept
{
	if (handle == 0)
		return nullptr;
	for (const std::unique_ptr<TextureSet> & set: sets_)
	{
		if (!set)
			continue;
		for (int i = 0; i < 3; ++i)
		{
			if (set->handles[i] == handle)
				return set.get();
		}
	}
	return nullptr;
}

// A submitted layer names one texture of a set, not the set: the compositor
// renders into the index GetNextSwapTextureSetIndex handed it, and copying from
// any other one would be a frame or two stale. Same lookup as ALVR's
// m_handleMap, which stores the (set, index) pair per handle.
ID3D11Texture2D * DirectModeComponent::find_texture_locked(vr::SharedTextureHandle_t handle) noexcept
{
	if (handle == 0)
		return nullptr;
	for (const std::unique_ptr<TextureSet> & set: sets_)
	{
		if (!set)
			continue;
		for (int i = 0; i < 3; ++i)
		{
			if (set->handles[i] == handle)
				return set->textures[i];
		}
	}
	return nullptr;
}

void DirectModeComponent::destroy_set_locked(size_t index) noexcept
{
	if (index >= sets_.size())
		return;
	if (std::unique_ptr<TextureSet> & set = sets_[index]; set)
	{
		for (int i = 0; i < 3; ++i)
			safe_release(set->textures[i]);
	}
	sets_.erase(sets_.begin() + static_cast<ptrdiff_t>(index));
}

void DirectModeComponent::CreateSwapTextureSet(uint32_t unPid,
                                               const SwapTextureSetDesc_t * pSwapTextureSetDesc,
                                               SwapTextureSet_t * pOutSwapTextureSet)
{
	try
	{
		if (pOutSwapTextureSet == nullptr || pSwapTextureSetDesc == nullptr)
		{
			breadcrumb("direct: CreateSwapTextureSet with a null desc/out, ignoring");
			return;
		}

		// Leave the caller's struct in a defined state whatever happens next: a
		// half-filled set with stale handles would be worse than an empty one.
		pOutSwapTextureSet->rSharedTextureHandles[0] = 0;
		pOutSwapTextureSet->rSharedTextureHandles[1] = 0;
		pOutSwapTextureSet->rSharedTextureHandles[2] = 0;
		pOutSwapTextureSet->unTextureFlags = 0;

		breadcrumb("direct: CreateSwapTextureSet pid=%u %ux%u format=%u samples=%u",
		           unPid,
		           pSwapTextureSetDesc->nWidth,
		           pSwapTextureSetDesc->nHeight,
		           pSwapTextureSetDesc->nFormat,
		           pSwapTextureSetDesc->nSampleCount);

		// The compositor asks for textures before it presents anything, so this
		// is where the vsync pacing starts.
		start_vsync_thread();

		std::lock_guard<std::mutex> device_lock(device_mutex_);
		if (!ensure_device_locked())
			return; // already breadcrumbed; the compositor sees zero handles

		// Field for field ALVR's OvrDirectModeComponent::CreateSwapTextureSet
		// (OvrDirectModeComponent.cpp:32-54).
		D3D11_TEXTURE2D_DESC desc{};
		const DXGI_FORMAT format = static_cast<DXGI_FORMAT>(pSwapTextureSetDesc->nFormat);
		desc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET;
		if (format == DXGI_FORMAT_R32G8X24_TYPELESS || format == DXGI_FORMAT_R32_TYPELESS)
			desc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
		desc.ArraySize = 1;
		desc.MipLevels = 1;
		desc.SampleDesc.Count = pSwapTextureSetDesc->nSampleCount == 0 ? 1 : pSwapTextureSetDesc->nSampleCount;
		desc.SampleDesc.Quality = 0;
		desc.Usage = D3D11_USAGE_DEFAULT;
		desc.Format = format;
		// Applications routinely ask for more than GetRecommendedRenderTargetSize
		// returned. Honour the request verbatim or the output is cropped.
		desc.Width = pSwapTextureSetDesc->nWidth;
		desc.Height = pSwapTextureSetDesc->nHeight;
		// D3D11_RESOURCE_MISC_SHARED, *not* SHARED_KEYEDMUTEX. ALVR has the
		// keyed-mutex line right there and commented out
		// (OvrDirectModeComponent.cpp:52-54), and the choice is not ours to
		// make: the compositor opens these handles itself, and a legacy shared
		// handle (GetSharedHandle) and an NT/keyed-mutex handle are not
		// interchangeable. The keyed mutex in this component is on the
		// compositor's *sync* texture, which we only open -- see Present().
		// Consistent with that, unTextureFlags stays 0: VRSwapTextureFlag_
		// Shared_NTHandle would be a lie here.
		desc.MiscFlags = D3D11_RESOURCE_MISC_SHARED;

		std::lock_guard<std::mutex> sets_lock(sets_mutex_);
		if (sets_.size() >= kMaxTextureSets)
		{
			breadcrumb("direct: refusing to create a %zuth swap texture set (pid %u)",
			           sets_.size() + 1,
			           unPid);
			return;
		}

		auto set = std::unique_ptr<TextureSet>(new (std::nothrow) TextureSet());
		if (!set)
		{
			breadcrumb("direct: out of memory allocating a swap texture set");
			return;
		}
		set->pid = unPid;
		set->width = desc.Width;
		set->height = desc.Height;
		set->format = pSwapTextureSetDesc->nFormat;
		set->sample_count = desc.SampleDesc.Count;

		for (int i = 0; i < 3; ++i)
		{
			HRESULT hr = device_->CreateTexture2D(&desc, nullptr, &set->textures[i]);
			if (FAILED(hr) || set->textures[i] == nullptr)
			{
				breadcrumb("direct: CreateTexture2D[%d] failed hr=0x%08lX", i, static_cast<unsigned long>(hr));
				for (int j = 0; j < 3; ++j)
					safe_release(set->textures[j]);
				return;
			}

			IDXGIResource * resource = nullptr;
			hr = set->textures[i]->QueryInterface(__uuidof(IDXGIResource), reinterpret_cast<void **>(&resource));
			if (FAILED(hr) || resource == nullptr)
			{
				breadcrumb("direct: QueryInterface(IDXGIResource)[%d] failed hr=0x%08lX",
				           i,
				           static_cast<unsigned long>(hr));
				for (int j = 0; j < 3; ++j)
					safe_release(set->textures[j]);
				return;
			}

			HANDLE shared = nullptr;
			hr = resource->GetSharedHandle(&shared);
			safe_release(resource);
			if (FAILED(hr) || shared == nullptr)
			{
				breadcrumb("direct: GetSharedHandle[%d] failed hr=0x%08lX", i, static_cast<unsigned long>(hr));
				for (int j = 0; j < 3; ++j)
					safe_release(set->textures[j]);
				return;
			}

			set->handles[i] = reinterpret_cast<vr::SharedTextureHandle_t>(shared);
			pOutSwapTextureSet->rSharedTextureHandles[i] = set->handles[i];
		}

		breadcrumb("direct: swap texture set for pid %u ready, handles %p %p %p",
		           unPid,
		           reinterpret_cast<void *>(set->handles[0]),
		           reinterpret_cast<void *>(set->handles[1]),
		           reinterpret_cast<void *>(set->handles[2]));

		try
		{
			sets_.push_back(std::move(set));
		}
		catch (...)
		{
			// push_back is the only throwing step; the handles are already in
			// the caller's struct, so undo them rather than leaving the
			// compositor with textures we have forgotten about.
			pOutSwapTextureSet->rSharedTextureHandles[0] = 0;
			pOutSwapTextureSet->rSharedTextureHandles[1] = 0;
			pOutSwapTextureSet->rSharedTextureHandles[2] = 0;
			for (int j = 0; j < 3; ++j)
				safe_release(set->textures[j]);
			breadcrumb("direct: out of memory tracking a swap texture set, rolled back");
		}
	}
	catch (...)
	{
		breadcrumb("direct: CreateSwapTextureSet threw, swallowed at the ABI boundary");
	}
}

void DirectModeComponent::DestroySwapTextureSet(vr::SharedTextureHandle_t sharedTextureHandle)
{
	try
	{
		breadcrumb("direct: DestroySwapTextureSet(%p)", reinterpret_cast<void *>(sharedTextureHandle));

		std::lock_guard<std::mutex> lock(sets_mutex_);
		for (size_t i = 0; i < sets_.size(); ++i)
		{
			const std::unique_ptr<TextureSet> & set = sets_[i];
			if (!set)
				continue;
			// Any one of the three handles destroys the whole set.
			if (set->handles[0] == sharedTextureHandle ||
			    set->handles[1] == sharedTextureHandle ||
			    set->handles[2] == sharedTextureHandle)
			{
				destroy_set_locked(i);
				return;
			}
		}
		breadcrumb("direct: DestroySwapTextureSet for an unknown handle %p, ignoring",
		           reinterpret_cast<void *>(sharedTextureHandle));
	}
	catch (...)
	{
		breadcrumb("direct: DestroySwapTextureSet threw, swallowed at the ABI boundary");
	}
}

void DirectModeComponent::DestroyAllSwapTextureSets(uint32_t unPid)
{
	try
	{
		std::lock_guard<std::mutex> lock(sets_mutex_);
		size_t destroyed = 0;
		for (size_t i = sets_.size(); i > 0; --i)
		{
			const size_t index = i - 1;
			if (sets_[index] && sets_[index]->pid == unPid)
			{
				destroy_set_locked(index);
				++destroyed;
			}
		}
		breadcrumb("direct: DestroyAllSwapTextureSets pid=%u destroyed %zu set(s), %zu left",
		           unPid,
		           destroyed,
		           sets_.size());
	}
	catch (...)
	{
		breadcrumb("direct: DestroyAllSwapTextureSets threw, swallowed at the ABI boundary");
	}
}

void DirectModeComponent::GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t sharedTextureHandles[2],
                                                     uint32_t (*pIndices)[2])
{
	try
	{
		if (sharedTextureHandles == nullptr)
			return;

		std::lock_guard<std::mutex> lock(sets_mutex_);
		for (int eye = 0; eye < 2; ++eye)
		{
			TextureSet * set = find_set_locked(sharedTextureHandles[eye]);
			if (set == nullptr)
				continue;
			// 0 -> 1 -> 2 -> 0, per set, exactly like ALVR
			// (OvrDirectModeComponent.cpp:156-160).
			set->index = (set->index + 1) % 3;
			if (pIndices != nullptr)
				(*pIndices)[eye] = set->index;
		}
	}
	catch (...)
	{
		breadcrumb("direct: GetNextSwapTextureSetIndex threw, swallowed at the ABI boundary");
	}
}

void DirectModeComponent::SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2])
{
	try
	{
		const uint64_t count = submit_count_.fetch_add(1, std::memory_order_relaxed) + 1;

		{
			std::lock_guard<std::mutex> lock(present_mutex_);
			// Only the most recent layer is kept. The compositor submits the
			// scene layer first and overlays after it, all already composited
			// into the same eye textures by the time Present runs -- ALVR keeps
			// the whole list only because its encoder path re-composites them.
			// We copy the last one out, which is the one that names the texture
			// the compositor finished with.
			for (int eye = 0; eye < 2; ++eye)
			{
				last_layer_texture_[eye] = perEye[eye].hTexture;
				last_layer_bounds_[eye] = perEye[eye].bounds;
			}
			last_layer_pose_ = perEye[0].mHmdPose;
			// Both eyes carry the same value; ALVR reads eye 0 too.
			last_layer_predict_ = perEye[0].flHmdPosePredictionTimeInSecondsFromNow;
			++submitted_layers_;
		}

		if (should_log(count))
			breadcrumb("direct: SubmitLayer #%llu textures %p/%p bounds L(%.3f,%.3f)-(%.3f,%.3f) "
			           "pose t=(%.3f,%.3f,%.3f) predict %.4fs",
			           static_cast<unsigned long long>(count),
			           reinterpret_cast<void *>(perEye[0].hTexture),
			           reinterpret_cast<void *>(perEye[1].hTexture),
			           static_cast<double>(perEye[0].bounds.uMin),
			           static_cast<double>(perEye[0].bounds.vMin),
			           static_cast<double>(perEye[0].bounds.uMax),
			           static_cast<double>(perEye[0].bounds.vMax),
			           static_cast<double>(perEye[0].mHmdPose.m[0][3]),
			           static_cast<double>(perEye[0].mHmdPose.m[1][3]),
			           static_cast<double>(perEye[0].mHmdPose.m[2][3]),
			           static_cast<double>(perEye[0].flHmdPosePredictionTimeInSecondsFromNow));
	}
	catch (...)
	{
		breadcrumb("direct: SubmitLayer threw, swallowed at the ABI boundary");
	}
}

void DirectModeComponent::Present(vr::SharedTextureHandle_t syncTexture)
{
	try
	{
		const uint64_t count = present_count_.fetch_add(1, std::memory_order_relaxed) + 1;

		FrameSnapshot frame;
		{
			std::lock_guard<std::mutex> lock(present_mutex_);
			for (int eye = 0; eye < 2; ++eye)
			{
				frame.texture[eye] = last_layer_texture_[eye];
				frame.bounds[eye] = last_layer_bounds_[eye];
			}
			frame.pose = last_layer_pose_;
			frame.predict = last_layer_predict_;
			frame.layers = submitted_layers_;
			submitted_layers_ = 0;
		}
		// Stamped here rather than in SubmitLayer: FrameReady.sample_time_qpc is
		// documented as "when Present ran", and it is what the client's
		// reprojection measures its own latency against.
		frame.qpc = util::query_qpc();

		// The acquire/release pair around the compositor's sync texture is how
		// its rendering is ordered against ours across processes: the copies we
		// submit between them are guaranteed to see finished pixels. ALVR does
		// exactly this around its CopyTexture
		// (OvrDirectModeComponent.cpp:238-264).
		bool acquired = false;
		bool sent = false;
		ipc::FrameReady ready{};

		if (frame.layers == 0)
		{
			// Present without a SubmitLayer: nothing was rendered, so there is
			// nothing to flatten. Not a dropped frame.
			if (should_log(count))
				breadcrumb("direct: Present #%llu with no submitted layer, nothing to send",
				           static_cast<unsigned long long>(count));
		}
		else if (syncTexture == 0)
		{
			// ALVR bails out the same way (OvrDirectModeComponent.cpp:231-236).
			// Without the sync texture there is no way to know the compositor
			// has finished rendering into the eye textures.
			count_drop("no sync texture");
		}
		else
		{
			std::lock_guard<std::mutex> lock(device_mutex_);
			ID3D11Texture2D * texture = shared_texture_locked(reinterpret_cast<void *>(syncTexture));
			if (texture == nullptr)
			{
				if (should_log(count))
					breadcrumb("direct: Present #%llu could not open sync texture %p",
					           static_cast<unsigned long long>(count),
					           reinterpret_cast<void *>(syncTexture));
			}
			else
			{
				IDXGIKeyedMutex * keyed_mutex = nullptr;
				if (SUCCEEDED(texture->QueryInterface(__uuidof(IDXGIKeyedMutex),
				                                      reinterpret_cast<void **>(&keyed_mutex))) &&
				    keyed_mutex != nullptr)
				{
					const HRESULT hr = keyed_mutex->AcquireSync(0, kAcquireSyncTimeoutMs);
					if (hr == S_OK)
					{
						acquired = true;
						sent = transport_frame_locked(frame, ready);
						keyed_mutex->ReleaseSync(0);
					}
					else
					{
						const uint64_t failures = acquire_failures_.fetch_add(1, std::memory_order_relaxed) + 1;
						if (should_log(failures))
							breadcrumb("direct: AcquireSync failed hr=0x%08lX (%llu so far)",
							           static_cast<unsigned long>(hr),
							           static_cast<unsigned long long>(failures));
						count_drop("sync texture busy");
					}
					safe_release(keyed_mutex);
				}
				else
				{
					// Not fatal: a sync texture without a keyed mutex just means
					// there is nothing to serialise against. The frame still
					// goes out -- the compositor is single-threaded per present
					// and its rendering is submitted before it calls us.
					if (should_log(count))
						breadcrumb("direct: sync texture %p has no IDXGIKeyedMutex",
						           reinterpret_cast<void *>(syncTexture));
					sent = transport_frame_locked(frame, ready);
				}
			}
		}

		// Queued outside the device lock: the IPC client takes its own lock and
		// wakes its thread, neither of which has any business happening with the
		// D3D device held.
		if (sent)
		{
			IpcClient * ipc = ipc_.load(std::memory_order_acquire);
			if (ipc == nullptr || !ipc->queue_frame_ready(ready))
			{
				// The slot is already the helper's and no FrameDone will ever
				// come back for it, so the ring has to be rebuilt rather than
				// silently losing a slot.
				count_drop("FrameReady could not be queued");
				ring_.mark_stale();
			}
		}

		if (should_log(count))
			breadcrumb("direct: Present #%llu sync=%p layers=%u acquired=%d sent=%d "
			           "(frames %llu sent / %llu dropped, %u slot(s) with the helper)",
			           static_cast<unsigned long long>(count),
			           reinterpret_cast<void *>(syncTexture),
			           frame.layers,
			           acquired ? 1 : 0,
			           sent ? 1 : 0,
			           static_cast<unsigned long long>(frames_sent_.load(std::memory_order_relaxed)),
			           static_cast<unsigned long long>(frames_dropped_.load(std::memory_order_relaxed)),
			           ring_.busy_slots());
	}
	catch (...)
	{
		breadcrumb("direct: Present threw, swallowed at the ABI boundary");
	}
}

// ---------------------------------------------------------------------------
// Video transport
// ---------------------------------------------------------------------------

void DirectModeComponent::attach_ipc(IpcClient * ipc) noexcept
{
	ipc_.store(ipc, std::memory_order_release);
}

void DirectModeComponent::on_ipc_connected() noexcept
{
	// Deliberately only a flag: this runs on the IPC client thread, and the
	// Present thread owns the ring and the D3D device. The next Present picks
	// it up, which is at most one frame away while the compositor is running --
	// and if it is not running there is nothing to announce anyway.
	resend_config_.store(true, std::memory_order_release);
	breadcrumb("direct: helper session up, StagingConfig will be re-sent on the next Present");
}

void DirectModeComponent::on_ipc_disconnected() noexcept
{
	resend_config_.store(false, std::memory_order_release);
	// Every slot the helper was holding is now unreachable and its keyed mutex
	// is in an unknown state (it may have died between AcquireSync(1) and
	// ReleaseSync(0)). Rebuilding from scratch with a new generation is the only
	// state we can be sure of, and it is cheaper than reasoning about abandoned
	// mutexes. The rebuild itself happens on the Present thread.
	ring_.mark_stale();
}

void DirectModeComponent::on_frame_done(const ipc::FrameDone & done) noexcept
{
	try
	{
		if (ring_.on_frame_done(done))
			return;

		// Stale: a FrameDone for a ring generation we have already replaced, or
		// for a slot we never handed out. Harmless, but worth counting -- a
		// steady stream of them means the helper is confused about generations.
		const uint64_t stale = frame_done_stale_.fetch_add(1, std::memory_order_relaxed) + 1;
		if (stale == 1 || (stale % kLogDropsEvery) == 0)
			breadcrumb("direct: ignored a stale FrameDone (frame %llu, slot %u, flags %u), "
			           "%llu so far",
			           static_cast<unsigned long long>(done.frame_id),
			           done.staging_index,
			           done.flags,
			           static_cast<unsigned long long>(stale));
	}
	catch (...)
	{
		breadcrumb("direct: on_frame_done threw");
	}
}

void DirectModeComponent::count_drop(const char * reason) noexcept
{
	const uint64_t drops = frames_dropped_.fetch_add(1, std::memory_order_relaxed) + 1;
	// Every reason is a string literal, so comparing the pointers is enough to
	// spot "the frames are being dropped for a new reason now" -- which is the
	// line that matters when reading this back off a real headset, and which a
	// plain every-100th rule would hide behind whichever reason came first.
	const bool new_reason = last_drop_reason_.exchange(reason, std::memory_order_relaxed) != reason;
	if (drops == 1 || new_reason || (drops % kLogDropsEvery) == 0)
		breadcrumb("direct: dropped frame (%s); %llu dropped, %llu sent",
		           reason != nullptr ? reason : "?",
		           static_cast<unsigned long long>(drops),
		           static_cast<unsigned long long>(frames_sent_.load(std::memory_order_relaxed)));
}

void DirectModeComponent::send_staging_config() noexcept
{
	ipc::StagingConfig config{};
	if (!ring_.describe(config))
		return;

	IpcClient * ipc = ipc_.load(std::memory_order_acquire);
	if (ipc == nullptr || !ipc->queue_staging_config(config))
	{
		// No session yet. on_ipc_connected() will ask again when there is one.
		breadcrumb("direct: StagingConfig (generation %u) not queued, no helper session",
		           config.generation);
		return;
	}

	breadcrumb("direct: StagingConfig sent: generation %u, %u slot(s), %ux%u dxgi_format=%u, "
	           "handles 0x%llX 0x%llX 0x%llX",
	           config.generation,
	           config.count,
	           config.width,
	           config.height,
	           config.dxgi_format,
	           static_cast<unsigned long long>(config.handles[0]),
	           static_cast<unsigned long long>(config.handles[1]),
	           static_cast<unsigned long long>(config.handles[2]));
}

bool DirectModeComponent::transport_frame_locked(const FrameSnapshot & frame, ipc::FrameReady & out) noexcept
{
	IpcClient * ipc = ipc_.load(std::memory_order_acquire);
	if (ipc == nullptr || !ipc->connected())
	{
		// No helper. Not an error and not worth a breadcrumb every frame: the
		// Present line already reports the running totals.
		frames_dropped_.fetch_add(1, std::memory_order_relaxed);
		return false;
	}

	if (!ensure_device_locked())
	{
		count_drop("no D3D device");
		return false;
	}

	// Hold a reference across the copies: DestroySwapTextureSet can arrive from
	// another thread, and a set is freed the moment it does.
	ID3D11Texture2D * eyes[2] = {nullptr, nullptr};
	{
		std::lock_guard<std::mutex> lock(sets_mutex_);
		for (int eye = 0; eye < 2; ++eye)
		{
			eyes[eye] = find_texture_locked(frame.texture[eye]);
			if (eyes[eye] != nullptr)
				eyes[eye]->AddRef();
		}
	}

	struct EyeGuard
	{
		ID3D11Texture2D * (&textures)[2];
		~EyeGuard()
		{
			for (ID3D11Texture2D *& texture: textures)
				safe_release(texture);
		}
	} guard{eyes};

	if (eyes[0] == nullptr || eyes[1] == nullptr)
	{
		count_drop("submitted texture is not one of ours");
		return false;
	}

	D3D11_TEXTURE2D_DESC desc[2]{};
	eyes[0]->GetDesc(&desc[0]);
	eyes[1]->GetDesc(&desc[1]);

	if (desc[0].SampleDesc.Count > 1 || desc[1].SampleDesc.Count > 1)
	{
		if (first_time(logged_msaa_))
			breadcrumb("direct: the compositor submitted %ux MSAA eye textures; a plain copy "
			           "cannot resolve those, so no video will flow until this is handled",
			           desc[0].SampleDesc.Count);
		count_drop("multisampled eye texture");
		return false;
	}

	if (desc[0].Format != desc[1].Format)
	{
		if (first_time(logged_mismatch_))
			breadcrumb("direct: the two eyes have different formats (%u vs %u), which one "
			           "staging texture cannot carry",
			           static_cast<unsigned>(desc[0].Format),
			           static_cast<unsigned>(desc[1].Format));
		count_drop("eye formats differ");
		return false;
	}

	bool partial[2] = {false, false};
	bool flipped[2] = {false, false};
	SubRect rect[2];
	for (int eye = 0; eye < 2; ++eye)
		rect[eye] = rect_from_bounds(frame.bounds[eye], desc[eye].Width, desc[eye].Height, partial[eye], flipped[eye]);

	if ((partial[0] || partial[1]) && first_time(logged_bounds_))
		breadcrumb("direct: layer bounds are not the full texture -- L(%.4f,%.4f)-(%.4f,%.4f) "
		           "R(%.4f,%.4f)-(%.4f,%.4f) of %ux%u/%ux%u, copying %ux%u and %ux%u. Scaled "
		           "render targets are in play.",
		           static_cast<double>(frame.bounds[0].uMin),
		           static_cast<double>(frame.bounds[0].vMin),
		           static_cast<double>(frame.bounds[0].uMax),
		           static_cast<double>(frame.bounds[0].vMax),
		           static_cast<double>(frame.bounds[1].uMin),
		           static_cast<double>(frame.bounds[1].vMin),
		           static_cast<double>(frame.bounds[1].uMax),
		           static_cast<double>(frame.bounds[1].vMax),
		           desc[0].Width,
		           desc[0].Height,
		           desc[1].Width,
		           desc[1].Height,
		           rect[0].w,
		           rect[0].h,
		           rect[1].w,
		           rect[1].h);

	if ((flipped[0] || flipped[1]) && first_time(logged_flip_))
		breadcrumb("direct: layer bounds are inverted (vMin %.4f > vMax %.4f); a copy cannot "
		           "flip, the image will be upside down until a blit shader exists",
		           static_cast<double>(frame.bounds[0].vMin),
		           static_cast<double>(frame.bounds[0].vMax));

	if (rect[0].w == 0 || rect[0].h == 0 || rect[1].w == 0 || rect[1].h == 0)
	{
		count_drop("empty layer bounds");
		return false;
	}

	// The staging texture is sized off the left eye and the right eye is clamped
	// into the other half: two eyes of different sizes is not a thing SteamVR
	// does, and a mismatch must not be allowed to write outside the texture.
	const uint32_t eye_width = rect[0].w;
	const uint32_t eye_height = rect[0].h;
	rect[1].w = std::min(rect[1].w, eye_width);
	rect[1].h = std::min(rect[1].h, eye_height);

	bool format_changed = false;
	const uint32_t staging_format = staging_format_for(static_cast<uint32_t>(desc[0].Format), format_changed);
	if (format_changed && first_time(logged_format_))
		breadcrumb("direct: submitted format %u is TYPELESS; the staging ring will be created "
		           "as %u and the helper told so",
		           static_cast<unsigned>(desc[0].Format),
		           staging_format);

	const uint32_t staging_width = eye_width * 2;

	if (!ring_.matches(staging_width, eye_height, staging_format))
	{
		if (ring_attempts_ >= kMaxRingAttempts)
		{
			count_drop("staging ring could not be created");
			return false;
		}
		++ring_attempts_;

		breadcrumb("direct: building staging ring #%d for %ux%u (per eye %ux%u) dxgi_format=%u, "
		           "submitted format %u",
		           ring_attempts_,
		           staging_width,
		           eye_height,
		           eye_width,
		           eye_height,
		           staging_format,
		           static_cast<unsigned>(desc[0].Format));

		if (!ring_.create(device_, staging_width, eye_height, staging_format, kStagingSlots))
		{
			count_drop("staging ring creation failed");
			return false;
		}
		ring_attempts_ = 0;
		resend_config_.store(false, std::memory_order_release);
		send_staging_config();
	}
	else if (resend_config_.exchange(false, std::memory_order_acq_rel))
	{
		// Same ring, new helper: it has never seen these handles.
		send_staging_config();
	}

	const FrameStagingRing::Lease lease = ring_.acquire_slot();
	if (!lease.valid())
	{
		// Every slot is still being encoded. This is the contract's
		// backpressure, not a fault.
		count_drop("all staging slots busy");
		return false;
	}

	for (int eye = 0; eye < 2; ++eye)
	{
		D3D11_BOX box{};
		box.left = rect[eye].x;
		box.top = rect[eye].y;
		box.front = 0;
		box.right = rect[eye].x + rect[eye].w;
		box.bottom = rect[eye].y + rect[eye].h;
		box.back = 1;

		const UINT dst_x = eye == 0 ? 0u : eye_width;
		context_->CopySubresourceRegion(lease.texture, 0, dst_x, 0, 0, eyes[eye], 0, &box);
	}

	const uint64_t frame_id = frame_id_.fetch_add(1, std::memory_order_relaxed) + 1;

	// ReleaseSync(1) is the ordering guarantee, not a hint: the helper's
	// AcquireSync(1) on the same surface cannot complete until the GPU work this
	// device submitted before the release has finished. That is the documented
	// contract of the keyed mutex, and it is why there is no fence and no
	// GPU-side wait of our own here.
	//
	// The Flush that follows is about *submission*, not ordering. Our device
	// never presents anything, so nothing else would ever force its command
	// buffer out; without this the copies could sit in the immediate context for
	// an unbounded number of frames while the helper waits on a mutex that has
	// been released only on paper. ALVR calls Flush around its copy for the same
	// reason (OvrDirectModeComponent.cpp:334, :358).
	ring_.release_to_helper(lease, frame_id);
	context_->Flush();

	out = ipc::FrameReady{};
	out.frame_id = frame_id;
	out.sample_time_qpc = frame.qpc;
	out.generation = ring_.generation();
	out.staging_index = static_cast<uint32_t>(lease.index);

	float rotation[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	float position[3] = {0.0f, 0.0f, 0.0f};
	util::decompose_matrix34(frame.pose, rotation, position);
	out.qw = rotation[0];
	out.qx = rotation[1];
	out.qy = rotation[2];
	out.qz = rotation[3];
	out.px = position[0];
	out.py = position[1];
	out.pz = position[2];
	out.predict_s = std::isfinite(frame.predict) ? frame.predict : 0.0f;

	const uint64_t sent = frames_sent_.fetch_add(1, std::memory_order_relaxed) + 1;
	if (should_log(sent))
		breadcrumb("direct: frame %llu -> slot %d (generation %u), pose q=(%.3f,%.3f,%.3f,%.3f) "
		           "p=(%.3f,%.3f,%.3f) predict %.4fs",
		           static_cast<unsigned long long>(frame_id),
		           lease.index,
		           out.generation,
		           static_cast<double>(out.qw),
		           static_cast<double>(out.qx),
		           static_cast<double>(out.qy),
		           static_cast<double>(out.qz),
		           static_cast<double>(out.px),
		           static_cast<double>(out.py),
		           static_cast<double>(out.pz),
		           static_cast<double>(out.predict_s));
	return true;
}

void DirectModeComponent::PostPresent(const Throttling_t * pThrottling)
{
	try
	{
		const uint64_t count = post_present_count_.fetch_add(1, std::memory_order_relaxed) + 1;

		if (should_log(count))
			breadcrumb("direct: PostPresent #%llu throttle=%u predict=%u",
			           static_cast<unsigned long long>(count),
			           pThrottling != nullptr ? pThrottling->nFramesToThrottle : 0u,
			           pThrottling != nullptr ? pThrottling->nAdditionalFramesToPredict : 0u);

		// ALVR's PostPresent is one line: WaitForVSync()
		// (OvrDirectModeComponent.cpp:275-279), implemented on the Rust side as
		// "block until the next vsync tick". Same shape here, against our own
		// vsync thread.
		//
		// The timeout is the safety net that makes this a stub rather than a
		// hazard: if the vsync thread never started, or the driver context went
		// away underneath it, the compositor's present thread returns a frame
		// late instead of never.
		start_vsync_thread();

		const auto interval = std::chrono::duration<double>(frame_interval_seconds());
		const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(interval * 2.0);

		std::unique_lock<std::mutex> lock(vsync_mutex_);
		const uint64_t seen = vsync_counter_;
		vsync_cv_.wait_for(lock, timeout, [this, seen] {
			return vsync_counter_ != seen || !vsync_running_.load(std::memory_order_acquire);
		});
	}
	catch (...)
	{
		breadcrumb("direct: PostPresent threw, swallowed at the ABI boundary");
	}
}

void DirectModeComponent::GetFrameTiming(vr::DriverDirectMode_FrameTiming * pFrameTiming)
{
	try
	{
		if (pFrameTiming == nullptr)
			return;

		const uint64_t count = frame_timing_count_.fetch_add(1, std::memory_order_relaxed) + 1;

		// m_nSize is the compositor's version handshake: it fills it in with the
		// size of the struct *it* knows about, and new members are only ever
		// appended. Writing past that size would scribble on the compositor's
		// stack.
		const uint32_t size = pFrameTiming->m_nSize;

		auto fits = [size](size_t end) noexcept {
			return static_cast<size_t>(size) >= end;
		};

		if (fits(offsetof(vr::DriverDirectMode_FrameTiming, m_nNumFramePresents) + sizeof(uint32_t)))
			pFrameTiming->m_nNumFramePresents = 1;
		if (fits(offsetof(vr::DriverDirectMode_FrameTiming, m_nNumMisPresented) + sizeof(uint32_t)))
			pFrameTiming->m_nNumMisPresented = 0;
		if (fits(offsetof(vr::DriverDirectMode_FrameTiming, m_nNumDroppedFrames) + sizeof(uint32_t)))
			pFrameTiming->m_nNumDroppedFrames = 0;
		// Mandatory. The header's own default body does only this, and says why:
		// the VRCompositor_ReprojectionMotion_* flags are passed *in* and
		// overlap VRCompositor_ThrottleMask, so leaving them set would be read
		// back as a throttling request (openvr_driver.h:3141-3146).
		if (fits(offsetof(vr::DriverDirectMode_FrameTiming, m_nReprojectionFlags) + sizeof(uint32_t)))
			pFrameTiming->m_nReprojectionFlags = 0;

		if (should_log(count))
			breadcrumb("direct: GetFrameTiming #%llu (m_nSize=%u, sizeof=%zu)",
			           static_cast<unsigned long long>(count),
			           size,
			           sizeof(vr::DriverDirectMode_FrameTiming));
	}
	catch (...)
	{
		breadcrumb("direct: GetFrameTiming threw, swallowed at the ABI boundary");
	}
}

// ---------------------------------------------------------------------------
// Vsync
// ---------------------------------------------------------------------------

double DirectModeComponent::frame_interval_seconds() const noexcept
{
	double hz = 90.0;
	try
	{
		if (owner_ != nullptr)
			hz = static_cast<double>(owner_->config_copy().refresh_hz);
	}
	catch (...)
	{
	}
	// Also catches NaN: every comparison against NaN is false, so the ternary
	// falls through to the clamp below with hz unchanged, which std::clamp would
	// propagate -- hence the explicit test.
	if (!(hz > 0.0))
		hz = 90.0;
	hz = std::clamp(hz, kMinRefreshHz, kMaxRefreshHz);
	return 1.0 / hz;
}

void DirectModeComponent::start_vsync_thread() noexcept
{
	if (shut_down_.load(std::memory_order_acquire))
		return;
	if (vsync_started_.exchange(true, std::memory_order_acq_rel))
		return;

	vsync_running_.store(true, std::memory_order_release);
	vsync_exited_.store(false, std::memory_order_release);
	try
	{
		vsync_thread_ = std::thread([this] { vsync_loop(); });
		breadcrumb("direct: vsync thread started at %.2f Hz", 1.0 / frame_interval_seconds());
	}
	catch (...)
	{
		vsync_running_.store(false, std::memory_order_release);
		vsync_exited_.store(true, std::memory_order_release);
		breadcrumb("direct: could not start the vsync thread, PostPresent will fall back to "
		           "its timeout");
	}
}

void DirectModeComponent::vsync_loop() noexcept
{
	auto next = std::chrono::steady_clock::now();

	while (vsync_running_.load(std::memory_order_acquire))
	{
		const auto interval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
		        std::chrono::duration<double>(frame_interval_seconds()));
		next += interval;

		const auto now = std::chrono::steady_clock::now();
		if (next < now)
			next = now + interval; // fell behind (or the rate changed); resync

		std::this_thread::sleep_until(next);
		if (!vsync_running_.load(std::memory_order_acquire))
			break;

		// Mirrors ALVR's SendVSync (alvr_server.cpp:362), through the null-safe
		// accessors: vrctx::host() is null both before Init and after Cleanup,
		// and the raw vr::VRServerDriverHost() would dereference a null context
		// instead of saying so (vr_context.h).
		if (vrctx::live())
		{
			if (vr::IVRServerDriverHost * host = vrctx::host(); host != nullptr)
				host->VsyncEvent(0.0);
		}

		{
			std::lock_guard<std::mutex> lock(vsync_mutex_);
			++vsync_counter_;
		}
		vsync_cv_.notify_all();

		const uint64_t count = vsync_count_.fetch_add(1, std::memory_order_relaxed) + 1;
		if (should_log(count))
			breadcrumb("direct: VsyncEvent #%llu (%.2f Hz, context %s)",
			           static_cast<unsigned long long>(count),
			           1.0 / frame_interval_seconds(),
			           vrctx::live() ? "live" : "down");
	}

	vsync_exited_.store(true, std::memory_order_release);
	// Anyone parked in PostPresent has to be released, or Cleanup waits on a
	// thread that is waiting on the thread it is joining.
	vsync_cv_.notify_all();
}

// ---------------------------------------------------------------------------
// Teardown
// ---------------------------------------------------------------------------

void DirectModeComponent::shutdown() noexcept
{
	if (shut_down_.exchange(true, std::memory_order_acq_rel))
		return;

	breadcrumb("direct: shutdown (presents=%llu submits=%llu vsyncs=%llu frames sent=%llu "
	           "dropped=%llu)",
	           static_cast<unsigned long long>(present_count_.load(std::memory_order_relaxed)),
	           static_cast<unsigned long long>(submit_count_.load(std::memory_order_relaxed)),
	           static_cast<unsigned long long>(vsync_count_.load(std::memory_order_relaxed)),
	           static_cast<unsigned long long>(frames_sent_.load(std::memory_order_relaxed)),
	           static_cast<unsigned long long>(frames_dropped_.load(std::memory_order_relaxed)));

	// Nothing may reach the IPC client once we start tearing D3D down.
	ipc_.store(nullptr, std::memory_order_release);

	vsync_running_.store(false, std::memory_order_release);
	vsync_cv_.notify_all();
	if (vsync_thread_.joinable())
	{
		try
		{
			vsync_thread_.join();
		}
		catch (...)
		{
			breadcrumb("direct: joining the vsync thread threw");
		}
	}

	try
	{
		std::lock_guard<std::mutex> lock(sets_mutex_);
		while (!sets_.empty())
			destroy_set_locked(sets_.size() - 1);
	}
	catch (...)
	{
	}

	try
	{
		std::lock_guard<std::mutex> lock(device_mutex_);
		// Before release_device_locked: the staging textures were created by
		// this device and their NT handles are ours to close.
		ring_.destroy();
		release_device_locked();
	}
	catch (...)
	{
	}

	breadcrumb("direct: shutdown done");
}

} // namespace wivrnnx
