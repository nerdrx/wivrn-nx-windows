#include "d3d11_stage.h"

#include <dxgi1_2.h>

#include <cstdlib>
#include <cstring>
#include <string>

#include "../log.h"

namespace wivrnnx::helper
{

namespace
{

template <typename T>
void safe_release(T *& p)
{
	if (p != nullptr)
	{
		p->Release();
		p = nullptr;
	}
}

std::string narrow_ascii(const wchar_t * s)
{
	std::string out;
	for (; s != nullptr && *s != L'\0'; ++s)
		out.push_back(*s < 128 ? static_cast<char>(*s) : '?');
	return out;
}

} // namespace

D3D11Stage::~D3D11Stage()
{
	close_ring();
	destroy_device();
}

bool D3D11Stage::create_device()
{
	if (device_ != nullptr)
		return true;

	IDXGIAdapter * adapter = nullptr;
	IDXGIFactory1 * factory = nullptr;

	// A hybrid or multi-GPU box has more than one candidate and only one of them
	// is the device vrserver rendered into; OpenSharedResource1 on the wrong one
	// simply fails. There is no way to ask a bare NT handle which adapter it
	// belongs to, so the default (adapter 0, which is what D3D11CreateDevice with
	// a null adapter picks) is the guess, and WIVRNNX_ADAPTER is the override.
	const char * adapter_env = std::getenv("WIVRNNX_ADAPTER");
	if (adapter_env != nullptr && adapter_env[0] != '\0' &&
	    SUCCEEDED(CreateDXGIFactory1(__uuidof(IDXGIFactory1), reinterpret_cast<void **>(&factory))))
	{
		const UINT index = static_cast<UINT>(std::atoi(adapter_env));
		if (FAILED(factory->EnumAdapters(index, &adapter)))
		{
			log_line("d3d11: WIVRNNX_ADAPTER=%s does not name an adapter, using the default", adapter_env);
			adapter = nullptr;
		}
	}

	const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0};
	D3D_FEATURE_LEVEL level{};

	const HRESULT hr = D3D11CreateDevice(adapter,
	                                     adapter != nullptr ? D3D_DRIVER_TYPE_UNKNOWN : D3D_DRIVER_TYPE_HARDWARE,
	                                     nullptr,
	                                     0,
	                                     levels,
	                                     static_cast<UINT>(sizeof(levels) / sizeof(levels[0])),
	                                     D3D11_SDK_VERSION,
	                                     &device_,
	                                     &level,
	                                     &context_);
	safe_release(adapter);
	safe_release(factory);

	if (FAILED(hr))
	{
		log_line("d3d11: D3D11CreateDevice failed (hr 0x%08lX)", static_cast<unsigned long>(hr));
		device_ = nullptr;
		context_ = nullptr;
		return false;
	}

	// AMF drives this device from its own threads once InitDX11 has been called.
	// ALVR does the same thing for the same reason
	// (reference/alvr/alvr/server_openvr/cpp/platform/win32/shared/d3drender.cpp:74-79).
	ID3D11Multithread * multithread = nullptr;
	if (SUCCEEDED(context_->QueryInterface(__uuidof(ID3D11Multithread), reinterpret_cast<void **>(&multithread))))
	{
		multithread->SetMultithreadProtected(TRUE);
		multithread->Release();
	}
	else
	{
		log_line("d3d11: no ID3D11Multithread on the immediate context, continuing unprotected");
	}

	std::string description = "unknown adapter";
	IDXGIDevice * dxgi_device = nullptr;
	if (SUCCEEDED(device_->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void **>(&dxgi_device))))
	{
		IDXGIAdapter * used = nullptr;
		if (SUCCEEDED(dxgi_device->GetAdapter(&used)) && used != nullptr)
		{
			DXGI_ADAPTER_DESC desc{};
			if (SUCCEEDED(used->GetDesc(&desc)))
				description = narrow_ascii(desc.Description);
			used->Release();
		}
		dxgi_device->Release();
	}

	log_line("d3d11: device created on \"%s\" (feature level %u.%u)",
	         description.c_str(),
	         static_cast<unsigned>(level >> 12) & 0xF,
	         static_cast<unsigned>(level >> 8) & 0xF);
	return true;
}

void D3D11Stage::destroy_device()
{
	safe_release(context_);
	safe_release(device_);
}

bool D3D11Stage::open_ring(const ipc::StagingConfig & config, uint32_t vrserver_pid)
{
	close_ring();

	if (device_ == nullptr)
		return false;

	if (config.count == 0 || config.count > kMaxSlots)
	{
		log_line("staging: ring of %u slots is out of range (1..%u)", config.count, kMaxSlots);
		return false;
	}
	if (config.width < 2 || config.height < 1 || (config.width & 1u) != 0)
	{
		log_line("staging: implausible ring geometry %ux%u", config.width, config.height);
		return false;
	}

	ID3D11Device1 * device1 = nullptr;
	if (FAILED(device_->QueryInterface(__uuidof(ID3D11Device1), reinterpret_cast<void **>(&device1))))
	{
		log_line("staging: no ID3D11Device1, cannot open NT-handle shared textures");
		return false;
	}

	// Same user, same session: PROCESS_DUP_HANDLE is the only right needed and
	// SteamVR's vrserver runs as the same user as the helper. A helper started
	// elevated (or vrserver elevated and the helper not) is the case that fails
	// here, which is worth the explicit error text.
	HANDLE vrserver = OpenProcess(PROCESS_DUP_HANDLE, FALSE, vrserver_pid);
	if (vrserver == nullptr)
	{
		log_win32(GetLastError(),
		          "staging: OpenProcess(PROCESS_DUP_HANDLE, pid %u) failed - is the helper running as "
		          "the same user as vrserver?",
		          vrserver_pid);
		device1->Release();
		return false;
	}

	bool ok = true;
	for (uint32_t i = 0; i < config.count && ok; ++i)
	{
		HANDLE local = nullptr;
		if (!DuplicateHandle(vrserver,
		                     reinterpret_cast<HANDLE>(static_cast<uintptr_t>(config.handles[i])),
		                     GetCurrentProcess(),
		                     &local,
		                     0,
		                     FALSE,
		                     DUPLICATE_SAME_ACCESS))
		{
			log_win32(GetLastError(), "staging: DuplicateHandle for slot %u failed", i);
			ok = false;
			break;
		}

		ID3D11Texture2D * texture = nullptr;
		const HRESULT hr = device1->OpenSharedResource1(local,
		                                               __uuidof(ID3D11Texture2D),
		                                               reinterpret_cast<void **>(&texture));
		// D3D takes its own reference; ours is done either way.
		CloseHandle(local);

		if (FAILED(hr) || texture == nullptr)
		{
			log_line("staging: OpenSharedResource1 for slot %u failed (hr 0x%08lX)",
			         i,
			         static_cast<unsigned long>(hr));
			ok = false;
			break;
		}

		// What the shim says the texture is and what it actually is have to
		// agree, or every copy after this is silently wrong.
		D3D11_TEXTURE2D_DESC desc{};
		texture->GetDesc(&desc);
		if (desc.Width != config.width || desc.Height != config.height ||
		    static_cast<uint32_t>(desc.Format) != config.dxgi_format)
		{
			log_line("staging: slot %u is %ux%u format %u, StagingConfig says %ux%u format %u",
			         i,
			         desc.Width,
			         desc.Height,
			         static_cast<unsigned>(desc.Format),
			         config.width,
			         config.height,
			         config.dxgi_format);
			texture->Release();
			ok = false;
			break;
		}

		IDXGIKeyedMutex * mutex = nullptr;
		if (FAILED(texture->QueryInterface(__uuidof(IDXGIKeyedMutex), reinterpret_cast<void **>(&mutex))))
		{
			log_line("staging: slot %u has no keyed mutex - the shim did not create it with "
			         "D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX",
			         i);
			texture->Release();
			ok = false;
			break;
		}

		slots_[i].texture = texture;
		slots_[i].mutex = mutex;
		slots_[i].held = false;
		++slot_count_;
	}

	CloseHandle(vrserver);
	device1->Release();

	if (!ok)
	{
		close_ring();
		return false;
	}

	generation_ = config.generation;
	width_ = config.width;
	height_ = config.height;
	dxgi_format_ = config.dxgi_format;

	log_line("staging: generation %u, %u slots of %ux%u (DXGI format %u), %ux%u per eye",
	         generation_,
	         slot_count_,
	         width_,
	         height_,
	         dxgi_format_,
	         width_ / 2,
	         height_);
	return true;
}

void D3D11Stage::close_ring()
{
	for (uint32_t i = 0; i < kMaxSlots; ++i)
	{
		// A slot still held at teardown must be handed back, or the shim's next
		// AcquireSync(0) on it waits forever. It cannot be encoded now anyway.
		if (slots_[i].held && slots_[i].mutex != nullptr)
			slots_[i].mutex->ReleaseSync(0);
		slots_[i].held = false;
		safe_release(slots_[i].mutex);
		safe_release(slots_[i].texture);
	}
	slot_count_ = 0;
	generation_ = 0;
	width_ = 0;
	height_ = 0;
	dxgi_format_ = 0;
}

ID3D11Texture2D * D3D11Stage::acquire(uint32_t index, uint32_t timeout_ms)
{
	if (index >= slot_count_ || slots_[index].mutex == nullptr)
		return nullptr;
	if (slots_[index].held)
	{
		log_line("staging: slot %u acquired twice without a release", index);
		return nullptr;
	}

	// Key 1 is what the shim releases with once the frame is in the slot.
	const HRESULT hr = slots_[index].mutex->AcquireSync(1, timeout_ms);
	if (hr == static_cast<HRESULT>(WAIT_TIMEOUT))
	{
		++acquire_timeouts_;
		return nullptr;
	}
	if (FAILED(hr))
	{
		++acquire_timeouts_;
		log_line("staging: AcquireSync(1) on slot %u failed (hr 0x%08lX)",
		         index,
		         static_cast<unsigned long>(hr));
		return nullptr;
	}

	slots_[index].held = true;
	return slots_[index].texture;
}

void D3D11Stage::release(uint32_t index)
{
	if (index >= slot_count_ || !slots_[index].held)
		return;
	// Key 0 hands the slot back to the shim.
	slots_[index].mutex->ReleaseSync(0);
	slots_[index].held = false;
}

bool D3D11Stage::copy_region(ID3D11Texture2D * src, ID3D11Texture2D * dst, uint32_t x, uint32_t width, uint32_t height)
{
	if (context_ == nullptr || src == nullptr || dst == nullptr)
		return false;

	const D3D11_BOX box{
	        .left = x,
	        .top = 0,
	        .front = 0,
	        .right = x + width,
	        .bottom = height,
	        .back = 1,
	};
	context_->CopySubresourceRegion(dst, 0, 0, 0, 0, src, 0, &box);
	return true;
}

void D3D11Stage::flush()
{
	if (context_ != nullptr)
		context_->Flush();
}

} // namespace wivrnnx::helper
