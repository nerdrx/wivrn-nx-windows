#include "log.h"

#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <iterator>

namespace wivrnnx::helper
{

namespace
{

void emit(const char * text)
{
	SYSTEMTIME now{};
	GetLocalTime(&now);
	std::printf("%02u:%02u:%02u.%03u | %s\n",
	            static_cast<unsigned>(now.wHour),
	            static_cast<unsigned>(now.wMinute),
	            static_cast<unsigned>(now.wSecond),
	            static_cast<unsigned>(now.wMilliseconds),
	            text);
	std::fflush(stdout);
}

} // namespace

void log_line(const char * fmt, ...)
{
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	std::vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	emit(buf);
}

void log_win32(unsigned long err, const char * fmt, ...)
{
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	int n = std::vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (n < 0)
		n = 0;
	size_t used = static_cast<size_t>(n);
	if (used >= sizeof(buf))
		used = sizeof(buf) - 1;

	wchar_t wtext[512];
	DWORD len = FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
	                           nullptr,
	                           static_cast<DWORD>(err),
	                           MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
	                           wtext,
	                           static_cast<DWORD>(std::size(wtext)),
	                           nullptr);
	// Trim the trailing CRLF FormatMessageW likes to add.
	while (len > 0 && (wtext[len - 1] == L'\r' || wtext[len - 1] == L'\n'))
		wtext[--len] = L'\0';

	char narrow[512];
	narrow[0] = '\0';
	if (len > 0)
	{
		int converted = WideCharToMultiByte(
		        CP_UTF8, 0, wtext, -1, narrow, static_cast<int>(std::size(narrow)), nullptr, nullptr);
		if (converted <= 0)
			narrow[0] = '\0';
	}

	std::snprintf(buf + used, sizeof(buf) - used, ": %s (%lu)", narrow, err);
	emit(buf);
}

} // namespace wivrnnx::helper
