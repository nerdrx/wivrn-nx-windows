// Minimal RAII wrapper for Win32 HANDLEs.
#pragma once

#include <windows.h>

#include <utility>

namespace wivrnnx::helper
{

// Owns a HANDLE and closes it exactly once. Treats both nullptr and
// INVALID_HANDLE_VALUE as "empty", so it can hold the results of CreateEventW
// (nullptr on failure) and CreateNamedPipeW (INVALID_HANDLE_VALUE on failure)
// without special-casing at every call site.
class UniqueHandle
{
public:
	UniqueHandle() = default;

	explicit UniqueHandle(HANDLE h) :
	        handle_(normalize(h))
	{
	}

	UniqueHandle(const UniqueHandle &) = delete;
	UniqueHandle & operator=(const UniqueHandle &) = delete;

	UniqueHandle(UniqueHandle && other) noexcept :
	        handle_(other.release())
	{
	}

	UniqueHandle & operator=(UniqueHandle && other) noexcept
	{
		if (this != &other)
			reset(other.release());
		return *this;
	}

	~UniqueHandle()
	{
		reset();
	}

	HANDLE get() const noexcept
	{
		return handle_;
	}

	explicit operator bool() const noexcept
	{
		return handle_ != nullptr;
	}

	[[nodiscard]] HANDLE release() noexcept
	{
		return std::exchange(handle_, nullptr);
	}

	void reset(HANDLE h = nullptr) noexcept
	{
		HANDLE old = handle_;
		handle_ = normalize(h);
		if (old != nullptr)
			CloseHandle(old);
	}

private:
	static HANDLE normalize(HANDLE h) noexcept
	{
		return h == INVALID_HANDLE_VALUE ? nullptr : h;
	}

	HANDLE handle_ = nullptr;
};

} // namespace wivrnnx::helper
