// RAII wrappers for Win32 kernel handles used by the shim.
#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <utility>

namespace wivrnnx
{

// Owns a HANDLE. Treats both nullptr and INVALID_HANDLE_VALUE as "empty" so the
// same type works for pipes (CreateFileW -> INVALID_HANDLE_VALUE on failure) and
// events (CreateEventW -> nullptr on failure).
class UniqueHandle
{
public:
	UniqueHandle() noexcept = default;
	explicit UniqueHandle(HANDLE h) noexcept :
	        handle_(h) {}

	UniqueHandle(const UniqueHandle &) = delete;
	UniqueHandle & operator=(const UniqueHandle &) = delete;

	UniqueHandle(UniqueHandle && other) noexcept :
	        handle_(std::exchange(other.handle_, nullptr)) {}

	UniqueHandle & operator=(UniqueHandle && other) noexcept
	{
		if (this != &other)
			reset(std::exchange(other.handle_, nullptr));
		return *this;
	}

	~UniqueHandle()
	{
		reset();
	}

	static bool is_valid(HANDLE h) noexcept
	{
		return h != nullptr && h != INVALID_HANDLE_VALUE;
	}

	bool valid() const noexcept
	{
		return is_valid(handle_);
	}

	HANDLE get() const noexcept
	{
		return handle_;
	}

	HANDLE release() noexcept
	{
		return std::exchange(handle_, nullptr);
	}

	void reset(HANDLE h = nullptr) noexcept
	{
		if (is_valid(handle_))
			::CloseHandle(handle_);
		handle_ = h;
	}

private:
	HANDLE handle_ = nullptr;
};

} // namespace wivrnnx
