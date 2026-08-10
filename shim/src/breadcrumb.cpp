#include "breadcrumb.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace wivrnnx
{
namespace
{

// Everything here is deliberately free of the C++ runtime: no std::string, no
// std::mutex, no static objects with constructors. A breadcrumb must work even
// if the reason we are logging is that the runtime itself is unhappy.
char g_path[MAX_PATH * 2] = {};
bool g_path_resolved = false;
bool g_session_started = false;
CRITICAL_SECTION g_lock;
LONG g_lock_state = 0; // 0 = untouched, 1 = initialising, 2 = ready

void lock_acquire() noexcept
{
	for (;;)
	{
		const LONG state = ::InterlockedCompareExchange(&g_lock_state, 1, 0);
		if (state == 2)
			break;
		if (state == 0)
		{
			::InitializeCriticalSection(&g_lock);
			::InterlockedExchange(&g_lock_state, 2);
			break;
		}
		::Sleep(0); // another thread is mid-initialisation
	}
	::EnterCriticalSection(&g_lock);
}

void lock_release() noexcept
{
	::LeaveCriticalSection(&g_lock);
}

// Tries %PROGRAMDATA%\wivrnnx\driver.log, then %TEMP%\wivrnnx-driver.log.
// Leaves g_path empty if neither can be created.
void resolve_path() noexcept
{
	if (g_path_resolved)
		return;
	g_path_resolved = true;

	char base[MAX_PATH] = {};
	if (::GetEnvironmentVariableA("PROGRAMDATA", base, sizeof(base)) != 0)
	{
		char dir[MAX_PATH * 2];
		std::snprintf(dir, sizeof(dir), "%s\\wivrnnx", base);
		::CreateDirectoryA(dir, nullptr); // fine if it already exists

		char candidate[MAX_PATH * 2];
		std::snprintf(candidate, sizeof(candidate), "%s\\driver.log", dir);
		HANDLE probe = ::CreateFileA(candidate,
		                             FILE_APPEND_DATA,
		                             FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
		                             nullptr,
		                             OPEN_ALWAYS,
		                             FILE_ATTRIBUTE_NORMAL,
		                             nullptr);
		if (probe != INVALID_HANDLE_VALUE)
		{
			::CloseHandle(probe);
			std::snprintf(g_path, sizeof(g_path), "%s", candidate);
			return;
		}
	}

	if (::GetEnvironmentVariableA("TEMP", base, sizeof(base)) != 0)
	{
		char candidate[MAX_PATH * 2];
		std::snprintf(candidate, sizeof(candidate), "%s\\wivrnnx-driver.log", base);
		HANDLE probe = ::CreateFileA(candidate,
		                             FILE_APPEND_DATA,
		                             FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
		                             nullptr,
		                             OPEN_ALWAYS,
		                             FILE_ATTRIBUTE_NORMAL,
		                             nullptr);
		if (probe != INVALID_HANDLE_VALUE)
		{
			::CloseHandle(probe);
			std::snprintf(g_path, sizeof(g_path), "%s", candidate);
			return;
		}
	}
}

// Open, append, close. Slower than holding the handle open, but it means the
// bytes are with the filesystem before we return: if vrserver takes an access
// violation on the very next instruction the line is still there.
void write_line(const char * line) noexcept
{
	if (g_path[0] == '\0')
		return;

	HANDLE file = ::CreateFileA(g_path,
	                            FILE_APPEND_DATA,
	                            FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
	                            nullptr,
	                            OPEN_ALWAYS,
	                            FILE_ATTRIBUTE_NORMAL,
	                            nullptr);
	if (file == INVALID_HANDLE_VALUE)
		return;

	::SetFilePointer(file, 0, nullptr, FILE_END);
	DWORD written = 0;
	::WriteFile(file, line, static_cast<DWORD>(std::strlen(line)), &written, nullptr);
	::FlushFileBuffers(file);
	::CloseHandle(file);
}

} // namespace

void breadcrumb(const char * format, ...) noexcept
{
	char message[900];
	va_list args;
	va_start(args, format);
	const int n = std::vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	if (n < 0)
		return;

	SYSTEMTIME now{};
	::GetLocalTime(&now);

	// pid as well as tid: the direct mode component can be driven from
	// vrcompositor while the device callbacks come from vrserver, both of them
	// appending to this one file. Without the pid the two processes' lines are
	// indistinguishable, and the session banner that carries it may be
	// thousands of lines away (or, in the second process, absent entirely).
	char line[1024];
	const int len = std::snprintf(line,
	                              sizeof(line),
	                              "%04u-%02u-%02u %02u:%02u:%02u.%03u [pid %5lu tid %5lu] %s\r\n",
	                              now.wYear,
	                              now.wMonth,
	                              now.wDay,
	                              now.wHour,
	                              now.wMinute,
	                              now.wSecond,
	                              now.wMilliseconds,
	                              ::GetCurrentProcessId(),
	                              ::GetCurrentThreadId(),
	                              message);
	if (len < 0)
		return;

	lock_acquire();
	resolve_path();
	write_line(line);
	lock_release();
}

void breadcrumb_begin_session() noexcept
{
	bool first = false;
	lock_acquire();
	resolve_path();
	if (!g_session_started)
	{
		g_session_started = true;
		first = true;
	}
	lock_release();

	if (!first)
		return;

	char module_path[MAX_PATH * 2] = {};
	HMODULE self = nullptr;
	if (::GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
	                         reinterpret_cast<LPCSTR>(&breadcrumb_begin_session),
	                         &self))
		::GetModuleFileNameA(self, module_path, sizeof(module_path));

	char host_path[MAX_PATH * 2] = {};
	::GetModuleFileNameA(nullptr, host_path, sizeof(host_path));

	breadcrumb("================ wivrnnx driver session ================");
	breadcrumb("built %s %s, host process %s (pid %lu)",
	           __DATE__,
	           __TIME__,
	           host_path[0] != '\0' ? host_path : "?",
	           ::GetCurrentProcessId());
	breadcrumb("module %s loaded at %p", module_path[0] != '\0' ? module_path : "?", static_cast<void *>(self));
}

const char * breadcrumb_path() noexcept
{
	lock_acquire();
	resolve_path();
	lock_release();
	return g_path;
}

} // namespace wivrnnx
