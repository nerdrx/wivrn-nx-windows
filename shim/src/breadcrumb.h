// Crash-proof breadcrumb log, independent of SteamVR.
//
// vrserver can die at any instant (and when it does, IVRDriverLog output is
// usually lost with it), so every interesting step the driver takes is also
// appended to a plain file with the handle closed again immediately. If the
// process is killed mid-frame the file still contains everything up to the last
// completed line.
//
// Location: %PROGRAMDATA%\wivrnnx\driver.log, falling back to
// %TEMP%\wivrnnx-driver.log if ProgramData is not writable (non-admin installs
// of Steam still get a usable log this way).
#pragma once

namespace wivrnnx
{

// Appends one timestamped, thread-tagged line. Never throws, never allocates,
// safe from any thread and at any point in the driver's lifetime including the
// very first factory call.
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
void
breadcrumb(const char * format, ...) noexcept;

// Writes the "---- session start ----" banner (pid, module path, build stamp).
// Idempotent; called from HmdDriverFactory before anything else happens.
void breadcrumb_begin_session() noexcept;

// Absolute path of the file actually in use, or "" if none could be opened.
// Handy for telling the user where to look from inside a driver log line.
const char * breadcrumb_path() noexcept;

} // namespace wivrnnx
