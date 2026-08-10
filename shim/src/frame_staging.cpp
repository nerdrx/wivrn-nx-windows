#include "frame_staging.h"

#include "breadcrumb.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <d3d11.h>
#include <dxgi.h>
// IDXGIResource1::CreateSharedHandle: the NT-handle export. IDXGIResource's
// GetSharedHandle (which the swap textures use) cannot produce one, and an
// NT handle is what the helper needs -- a legacy shared handle is not
// duplicable into another process.
#include <dxgi1_2.h>

namespace wivrnnx
{
namespace
{

// Zero, deliberately: a slot the helper still owns must fail *immediately*.
// Present runs on the compositor's present thread and anything that blocks there
// costs a frame everywhere, so "busy" has to mean "skip it", never "wait".
constexpr DWORD kSlotAcquireTimeoutMs = 0;

// AcquireSync does not report contention as a failed HRESULT: WAIT_TIMEOUT is
// 0x00000102 and WAIT_ABANDONED is 0x00000080, both of which pass SUCCEEDED().
// Only S_OK means "acquired and the contents are what the last owner left";
// WAIT_ABANDONED means "acquired, but the previous owner died mid-flight", which
// is fine for us because we overwrite the whole texture anyway.
bool acquired_ok(HRESULT hr) noexcept
{
	return hr == S_OK || hr == static_cast<HRESULT>(WAIT_ABANDONED);
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

FrameStagingRing::~FrameStagingRing()
{
	destroy();
}

bool FrameStagingRing::usable() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return count_ != 0 && !stale_;
}

bool FrameStagingRing::matches(uint32_t width, uint32_t height, uint32_t dxgi_format) const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return count_ != 0 && !stale_ && width_ == width && height_ == height && format_ == dxgi_format;
}

uint32_t FrameStagingRing::generation() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return generation_;
}

uint32_t FrameStagingRing::width() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return width_;
}

uint32_t FrameStagingRing::height() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return height_;
}

uint32_t FrameStagingRing::format() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return format_;
}

uint32_t FrameStagingRing::count() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return count_;
}

uint32_t FrameStagingRing::busy_slots() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	uint32_t busy = 0;
	for (uint32_t i = 0; i < count_; ++i)
	{
		if (slots_[i].helper_owned)
			++busy;
	}
	return busy;
}

bool FrameStagingRing::create(ID3D11Device * device,
                              uint32_t width,
                              uint32_t height,
                              uint32_t dxgi_format,
                              uint32_t count) noexcept
{
	if (device == nullptr || width == 0 || height == 0)
		return false;
	if (count == 0)
		count = 1;
	if (count > kMaxSlots)
		count = kMaxSlots;

	std::lock_guard<std::mutex> lock(mutex_);
	destroy_locked();

	D3D11_TEXTURE2D_DESC desc{};
	desc.Width = width;
	desc.Height = height;
	desc.MipLevels = 1;
	desc.ArraySize = 1;
	desc.Format = static_cast<DXGI_FORMAT>(dxgi_format);
	// Never multisampled: the helper encodes from this, and a shared MSAA
	// surface cannot be sampled without a resolve. The submitted eye textures
	// are checked for this separately (direct_mode.cpp).
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;
	desc.Usage = D3D11_USAGE_DEFAULT;
	// SHADER_RESOURCE is the one the helper needs (encode input / SRV);
	// RENDER_TARGET costs nothing and keeps a fullscreen-quad fallback open if
	// a format pair ever turns out not to be copyable.
	desc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET;
	desc.CPUAccessFlags = 0;
	// The whole point of this ring, and the difference from the swap textures:
	// a keyed mutex (cross-process GPU ordering) on an NT handle (duplicable
	// into the helper). GetSharedHandle cannot express either.
	desc.MiscFlags = D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX | D3D11_RESOURCE_MISC_SHARED_NTHANDLE;

	for (uint32_t i = 0; i < count; ++i)
	{
		Slot & slot = slots_[i];

		HRESULT hr = device->CreateTexture2D(&desc, nullptr, &slot.texture);
		if (FAILED(hr) || slot.texture == nullptr)
		{
			breadcrumb("staging: CreateTexture2D[%u] %ux%u fmt=%u failed hr=0x%08lX",
			           i,
			           width,
			           height,
			           dxgi_format,
			           static_cast<unsigned long>(hr));
			destroy_locked();
			return false;
		}

		IDXGIResource1 * resource = nullptr;
		hr = slot.texture->QueryInterface(__uuidof(IDXGIResource1), reinterpret_cast<void **>(&resource));
		if (FAILED(hr) || resource == nullptr)
		{
			breadcrumb("staging: QueryInterface(IDXGIResource1)[%u] failed hr=0x%08lX - this "
			           "runtime cannot export NT handles, no video transport",
			           i,
			           static_cast<unsigned long>(hr));
			destroy_locked();
			return false;
		}

		HANDLE shared = nullptr;
		// Unnamed (nullptr name): the helper gets the value through the pipe and
		// duplicates it, so there is nothing to look up by name -- and a name
		// would be a squattable object in the session namespace.
		hr = resource->CreateSharedHandle(nullptr, GENERIC_ALL, nullptr, &shared);
		safe_release(resource);
		if (FAILED(hr) || shared == nullptr)
		{
			breadcrumb("staging: CreateSharedHandle[%u] failed hr=0x%08lX",
			           i,
			           static_cast<unsigned long>(hr));
			destroy_locked();
			return false;
		}
		slot.handle = shared;

		hr = slot.texture->QueryInterface(__uuidof(IDXGIKeyedMutex), reinterpret_cast<void **>(&slot.mutex));
		if (FAILED(hr) || slot.mutex == nullptr)
		{
			breadcrumb("staging: QueryInterface(IDXGIKeyedMutex)[%u] failed hr=0x%08lX",
			           i,
			           static_cast<unsigned long>(hr));
			destroy_locked();
			return false;
		}

		slot.helper_owned = false;
		slot.leased = false;
		slot.frame_id = 0;
	}

	count_ = count;
	width_ = width;
	height_ = height;
	format_ = dxgi_format;
	stale_ = false;
	next_ = 0;
	++generation_;

	breadcrumb("staging: ring generation %u ready: %u slot(s) of %ux%u dxgi_format=%u, "
	           "handles %p %p %p",
	           generation_,
	           count_,
	           width_,
	           height_,
	           format_,
	           slots_[0].handle,
	           count_ > 1 ? slots_[1].handle : nullptr,
	           count_ > 2 ? slots_[2].handle : nullptr);
	return true;
}

void FrameStagingRing::destroy() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	destroy_locked();
}

void FrameStagingRing::destroy_locked() noexcept
{
	const bool had_ring = count_ != 0;

	for (Slot & slot: slots_)
	{
		// Releasing a keyed mutex we still hold keeps the object in a sane state
		// for anyone (the helper) still holding a duplicated handle to it.
		if (slot.leased && slot.mutex != nullptr)
			slot.mutex->ReleaseSync(0);
		slot.leased = false;
		slot.helper_owned = false;
		slot.frame_id = 0;

		safe_release(slot.mutex);
		safe_release(slot.texture);
		if (slot.handle != nullptr)
		{
			::CloseHandle(static_cast<HANDLE>(slot.handle));
			slot.handle = nullptr;
		}
	}

	count_ = 0;
	width_ = 0;
	height_ = 0;
	format_ = 0;
	stale_ = false;
	next_ = 0;

	if (had_ring)
		breadcrumb("staging: ring generation %u destroyed", generation_);
}

bool FrameStagingRing::describe(ipc::StagingConfig & out) const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (count_ == 0)
		return false;

	out = ipc::StagingConfig{};
	out.generation = generation_;
	out.width = width_;
	out.height = height_;
	out.dxgi_format = format_;
	out.count = count_;
	for (uint32_t i = 0; i < count_; ++i)
		out.handles[i] = static_cast<uint64_t>(reinterpret_cast<uintptr_t>(slots_[i].handle));
	return true;
}

FrameStagingRing::Lease FrameStagingRing::acquire_slot() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (count_ == 0 || stale_)
		return Lease{};

	for (uint32_t attempt = 0; attempt < count_; ++attempt)
	{
		const uint32_t index = (next_ + attempt) % count_;
		Slot & slot = slots_[index];
		if (slot.helper_owned || slot.leased || slot.mutex == nullptr || slot.texture == nullptr)
			continue;

		HRESULT hr = slot.mutex->AcquireSync(0, kSlotAcquireTimeoutMs);
		if (!acquired_ok(hr))
		{
			// A helper that FrameDone'd without ever acquiring (no client
			// connected, stale generation) leaves the mutex released with
			// key 1 — our own release, never consumed. Reclaim it.
			hr = slot.mutex->AcquireSync(1, 0);
		}
		if (!acquired_ok(hr))
		{
			// The helper is still inside its encode even though its FrameDone
			// said otherwise, or the mutex is wedged. Either way: next slot.
			continue;
		}
		if (hr != S_OK)
			breadcrumb("staging: slot %u was abandoned by the helper, taking it anyway", index);

		slot.leased = true;
		next_ = (index + 1) % count_;
		return Lease{static_cast<int>(index), slot.texture};
	}

	return Lease{};
}

void FrameStagingRing::release_to_helper(const Lease & lease, uint64_t frame_id) noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (lease.index < 0 || static_cast<uint32_t>(lease.index) >= count_)
		return;

	Slot & slot = slots_[lease.index];
	if (!slot.leased || slot.mutex == nullptr)
		return;

	// Key 1 is the helper's. This is also the GPU-side ordering point: the
	// helper's AcquireSync(1) cannot complete until the work we submitted before
	// this release has finished on the GPU.
	slot.mutex->ReleaseSync(1);
	slot.leased = false;
	slot.helper_owned = true;
	slot.frame_id = frame_id;
}

void FrameStagingRing::return_slot(const Lease & lease) noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (lease.index < 0 || static_cast<uint32_t>(lease.index) >= count_)
		return;

	Slot & slot = slots_[lease.index];
	if (!slot.leased || slot.mutex == nullptr)
		return;

	// Key 0: we are giving it back to ourselves, the helper never sees it.
	slot.mutex->ReleaseSync(0);
	slot.leased = false;
	slot.helper_owned = false;
	slot.frame_id = 0;
}

bool FrameStagingRing::on_frame_done(const ipc::FrameDone & done) noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (count_ == 0 || done.staging_index >= count_)
		return false;

	Slot & slot = slots_[done.staging_index];
	if (!slot.helper_owned)
		return false;
	// frame_id is monotonic across ring generations, so an id match is also a
	// generation match: a FrameDone for a ring we have already thrown away can
	// never collide with a lease of the current one.
	if (slot.frame_id != done.frame_id)
		return false;

	slot.helper_owned = false;
	slot.frame_id = 0;
	return true;
}

void FrameStagingRing::mark_stale() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (count_ != 0 && !stale_)
	{
		stale_ = true;
		breadcrumb("staging: ring generation %u marked stale (helper gone), it will be rebuilt "
		           "on the next Present",
		           generation_);
	}
}

} // namespace wivrnnx
