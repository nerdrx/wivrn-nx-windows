#include "store.h"

#include <windows.h>

#include <shlobj.h>

#include <bcrypt.h>

#include <cstdio>
#include <cstring>

#include "../log.h"

namespace wivrnnx::helper
{

namespace
{

std::string narrow(const wchar_t * w)
{
	if (w == nullptr)
		return {};
	const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
	if (n <= 1)
		return {};
	std::string s(static_cast<size_t>(n - 1), '\0');
	WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
	return s;
}

std::wstring widen(const std::string & s)
{
	if (s.empty())
		return {};
	const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
	if (n <= 1)
		return {};
	std::wstring w(static_cast<size_t>(n - 1), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, w.data(), n);
	return w;
}

// Whole-file read. Returns false if the file does not exist; anything else is
// logged, because a key file that cannot be read means a headset that has to be
// paired again and the user deserves to know why.
bool read_file(const std::string & path, std::string & out)
{
	const std::wstring wpath = widen(path);
	HANDLE h = CreateFileW(wpath.c_str(),
	                       GENERIC_READ,
	                       FILE_SHARE_READ,
	                       nullptr,
	                       OPEN_EXISTING,
	                       FILE_ATTRIBUTE_NORMAL,
	                       nullptr);
	if (h == INVALID_HANDLE_VALUE)
	{
		const DWORD err = GetLastError();
		if (err != ERROR_FILE_NOT_FOUND && err != ERROR_PATH_NOT_FOUND)
			log_win32(err, "cannot open %s", path.c_str());
		return false;
	}

	out.clear();
	char buf[4096];
	DWORD got = 0;
	while (ReadFile(h, buf, sizeof(buf), &got, nullptr) && got > 0)
		out.append(buf, got);

	CloseHandle(h);
	return true;
}

// Write via a .new sibling and a rename, the way save_keys() does on Linux, so
// a crash mid-write cannot leave a truncated key list behind.
bool write_file_atomic(const std::string & path, const std::string & data)
{
	const std::string tmp = path + ".new";
	const std::wstring wtmp = widen(tmp);

	HANDLE h = CreateFileW(wtmp.c_str(),
	                       GENERIC_WRITE,
	                       0,
	                       nullptr,
	                       CREATE_ALWAYS,
	                       FILE_ATTRIBUTE_NORMAL,
	                       nullptr);
	if (h == INVALID_HANDLE_VALUE)
	{
		log_win32(GetLastError(), "cannot create %s", tmp.c_str());
		return false;
	}

	bool ok = true;
	size_t written_total = 0;
	while (written_total < data.size())
	{
		DWORD written = 0;
		const DWORD chunk = static_cast<DWORD>(data.size() - written_total);
		if (!WriteFile(h, data.data() + written_total, chunk, &written, nullptr) || written == 0)
		{
			log_win32(GetLastError(), "cannot write %s", tmp.c_str());
			ok = false;
			break;
		}
		written_total += written;
	}

	FlushFileBuffers(h);
	CloseHandle(h);

	if (!ok)
		return false;

	if (!MoveFileExW(wtmp.c_str(), widen(path).c_str(), MOVEFILE_REPLACE_EXISTING))
	{
		log_win32(GetLastError(), "cannot replace %s", path.c_str());
		return false;
	}
	return true;
}

std::string key_file()
{
	const std::string dir = appdata_dir();
	return dir.empty() ? std::string{} : dir + "\\known_keys.txt";
}

std::string cookie_file()
{
	const std::string dir = appdata_dir();
	return dir.empty() ? std::string{} : dir + "\\cookie";
}

// One record per line: "<name>\t<cleaned public key>". The key is base64 with
// the PEM armour and whitespace already stripped, so it can never contain a tab
// or a newline; the name is whatever the headset called itself, with any
// control character replaced when it is written.
std::string sanitize_name(std::string name)
{
	for (char & c: name)
	{
		if (c == '\t' || c == '\r' || c == '\n')
			c = ' ';
	}
	if (name.empty())
		name = "Unknown headset";
	return name;
}

} // namespace

std::string appdata_dir()
{
	static std::string cached = [] {
		wchar_t * roaming = nullptr;
		std::wstring base;
		if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_RoamingAppData, 0, nullptr, &roaming)) && roaming != nullptr)
		{
			base = roaming;
			CoTaskMemFree(roaming);
		}
		else
		{
			// Wine and stripped-down images sometimes answer the environment
			// variable when the shell API does not.
			wchar_t env[MAX_PATH]{};
			const DWORD n = GetEnvironmentVariableW(L"APPDATA", env, MAX_PATH);
			if (n == 0 || n >= MAX_PATH)
			{
				log_line("warning: neither FOLDERID_RoamingAppData nor %%APPDATA%% is available; "
				         "pairing will not be remembered");
				return std::string{};
			}
			base = env;
		}

		const std::wstring dir = base + L"\\wivrnnx";
		if (!CreateDirectoryW(dir.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS)
		{
			log_win32(GetLastError(), "cannot create %s", narrow(dir.c_str()).c_str());
			return std::string{};
		}
		return narrow(dir.c_str());
	}();

	return cached;
}

std::string server_cookie()
{
	static std::string cached = [] {
		const std::string path = cookie_file();

		std::string existing;
		if (!path.empty() && read_file(path, existing) && existing.size() >= 32)
			return existing.substr(0, 32);

		// 32 characters out of [0-9A-Za-z], the same alphabet and length as
		// server_cookie() on Linux. Drawn from the system CSPRNG rather than
		// std::random_device, which mingw's libstdc++ has historically made
		// deterministic.
		static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
		unsigned char raw[32]{};
		if (BCryptGenRandom(nullptr, raw, sizeof(raw), BCRYPT_USE_SYSTEM_PREFERRED_RNG) != 0)
		{
			log_line("warning: BCryptGenRandom failed, falling back to a time-seeded cookie");
			LARGE_INTEGER qpc{};
			QueryPerformanceCounter(&qpc);
			uint64_t x = static_cast<uint64_t>(qpc.QuadPart) ^ (static_cast<uint64_t>(GetCurrentProcessId()) << 32);
			for (unsigned char & b: raw)
			{
				x = x * 6364136223846793005ull + 1442695040888963407ull;
				b = static_cast<unsigned char>(x >> 33);
			}
		}

		std::string cookie(32, '0');
		for (size_t i = 0; i < cookie.size(); ++i)
			cookie[i] = alphabet[raw[i] % 62];

		if (!path.empty())
			write_file_atomic(path, cookie);

		return cookie;
	}();

	return cached;
}

std::string random_pin()
{
	uint32_t value = 0;
	if (BCryptGenRandom(nullptr,
	                    reinterpret_cast<unsigned char *>(&value),
	                    sizeof(value),
	                    BCRYPT_USE_SYSTEM_PREFERRED_RNG) != 0)
	{
		LARGE_INTEGER qpc{};
		QueryPerformanceCounter(&qpc);
		value = static_cast<uint32_t>(qpc.QuadPart);
	}

	char buffer[8];
	std::snprintf(buffer, sizeof(buffer), "%06u", value % 1000000u);
	return buffer;
}

std::vector<HeadsetKey> known_keys()
{
	std::vector<HeadsetKey> keys;

	const std::string path = key_file();
	std::string data;
	if (path.empty() || !read_file(path, data))
		return keys;

	size_t pos = 0;
	while (pos < data.size())
	{
		size_t eol = data.find('\n', pos);
		if (eol == std::string::npos)
			eol = data.size();

		std::string line = data.substr(pos, eol - pos);
		pos = eol + 1;

		if (!line.empty() && line.back() == '\r')
			line.pop_back();
		if (line.empty() || line[0] == '#')
			continue;

		const size_t tab = line.find('\t');
		if (tab == std::string::npos)
			continue;

		keys.push_back(HeadsetKey{line.substr(tab + 1), line.substr(0, tab)});
	}

	return keys;
}

bool is_key_known(const std::string & cleaned_public_key)
{
	if (cleaned_public_key.empty())
		return false;

	for (const HeadsetKey & k: known_keys())
	{
		if (k.public_key == cleaned_public_key)
			return true;
	}
	return false;
}

void add_known_key(const HeadsetKey & key)
{
	if (key.public_key.empty())
		return;

	std::vector<HeadsetKey> keys = known_keys();
	for (const HeadsetKey & k: keys)
	{
		if (k.public_key == key.public_key)
			return;
	}

	std::string name = sanitize_name(key.name);
	const std::string original = name;
	int n = 1;
	bool clash = true;
	while (clash)
	{
		clash = false;
		for (const HeadsetKey & k: keys)
		{
			if (k.name == name)
			{
				clash = true;
				break;
			}
		}
		if (clash)
			name = original + " (" + std::to_string(++n) + ")";
	}

	keys.push_back(HeadsetKey{key.public_key, name});

	std::string out = "# wivrnnx paired headsets: <name>\\t<public key>\n";
	for (const HeadsetKey & k: keys)
		out += k.name + "\t" + k.public_key + "\n";

	const std::string path = key_file();
	if (path.empty())
	{
		log_line("warning: no writable %%APPDATA%%\\wivrnnx, headset \"%s\" will have to pair again next time",
		         name.c_str());
		return;
	}

	if (write_file_atomic(path, out))
		log_line("paired headset \"%s\" saved to %s", name.c_str(), path.c_str());
}

} // namespace wivrnnx::helper
