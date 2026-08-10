// The helper's own D3D11 device and the ring of staging textures it opens out of
// vrserver.
//
// The keyed-mutex discipline is the frozen contract's, not a choice made here
// (ipc/wivrnnx_ipc.h:197-215): the shim releases with key 1 when a frame is in a
// slot, the helper acquires key 1, encodes, and releases with key 0, which is
// what the shim acquires before it reuses the slot. A slot the helper never
// released stays owned by the helper, the shim skips it, and frames drop in the
// shim rather than queueing anywhere — which is the backpressure.
#pragma once

#include <windows.h>

// d3d11_1.h for ID3D11Device1::OpenSharedResource1, d3d11_4.h for
// ID3D11Multithread (mingw-w64 declares it there, not in d3d11.h).
#include <d3d11_1.h>
#include <d3d11_4.h>

#include <cstdint>

#include "wivrnnx_ipc.h"

namespace wivrnnx::helper
{

class D3D11Stage
{
public:
	D3D11Stage() = default;
	~D3D11Stage();

	D3D11Stage(const D3D11Stage &) = delete;
	D3D11Stage & operator=(const D3D11Stage &) = delete;

	// The default adapter, or the one named by WIVRNNX_ADAPTER (an index into
	// IDXGIFactory1::EnumAdapters). Logs the adapter it picked: on a machine with
	// more than one GPU, opening the shared texture on the wrong one is the
	// failure this line exists to explain.
	bool create_device();
	void destroy_device();

	bool have_device() const
	{
		return device_ != nullptr;
	}

	ID3D11Device * device() const
	{
		return device_;
	}

	// Duplicates the NT handles out of vrserver and opens each on our device.
	// Replaces whatever ring was open before.
	bool open_ring(const ipc::StagingConfig & config, uint32_t vrserver_pid);
	void close_ring();

	uint32_t generation() const
	{
		return generation_;
	}

	uint32_t slot_count() const
	{
		return slot_count_;
	}

	uint32_t width() const
	{
		return width_;
	}

	uint32_t height() const
	{
		return height_;
	}

	uint32_t dxgi_format() const
	{
		return dxgi_format_;
	}

	// Acquire key 1 on a slot. Returns the texture, or null on timeout / bad
	// index. A successful acquire must be matched by exactly one release().
	ID3D11Texture2D * acquire(uint32_t index, uint32_t timeout_ms);

	// Release with key 0, handing the slot back to the shim.
	void release(uint32_t index);

	// Copies a rectangle of a staging texture into a destination texture of
	// exactly that size. One GPU copy, no shader, no format conversion beyond
	// what CopySubresourceRegion does inside a typeless family.
	bool copy_region(ID3D11Texture2D * src, ID3D11Texture2D * dst, uint32_t x, uint32_t width, uint32_t height);

	// Blocks until the copies queued so far have been consumed by the GPU. AMF
	// submits on its own queue; without this the encoder can read a surface the
	// copy has not landed in yet.
	void flush();

	uint64_t acquire_timeouts() const
	{
		return acquire_timeouts_;
	}

private:
	struct Slot
	{
		ID3D11Texture2D * texture = nullptr;
		IDXGIKeyedMutex * mutex = nullptr;
		bool held = false;
	};

	ID3D11Device * device_ = nullptr;
	ID3D11DeviceContext * context_ = nullptr;

	static constexpr uint32_t kMaxSlots = 3;
	Slot slots_[kMaxSlots]{};
	uint32_t slot_count_ = 0;
	uint32_t generation_ = 0;
	uint32_t width_ = 0;
	uint32_t height_ = 0;
	uint32_t dxgi_format_ = 0;
	uint64_t acquire_timeouts_ = 0;
};

} // namespace wivrnnx::helper
