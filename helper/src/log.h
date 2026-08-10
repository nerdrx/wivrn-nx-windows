// Timestamped stdout logging for the Phase 0 helper.
#pragma once

namespace wivrnnx::helper
{

// printf-style. Emits "HH:MM:SS.mmm | <message>\n" to stdout and flushes, so
// the log stays readable when the helper is run from a console next to
// SteamVR's own output.
void log_line(const char * fmt, ...);

// As log_line, but appends ": <FormatMessage text> (N)" for the given Win32
// error code (usually GetLastError()).
void log_win32(unsigned long err, const char * fmt, ...);

} // namespace wivrnnx::helper
