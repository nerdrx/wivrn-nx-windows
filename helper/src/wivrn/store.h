// Persistent state under %APPDATA%\wivrnnx\ : the paired-headset key list and
// the DNS-SD cookie.
//
// The Linux side keeps both under $XDG_CONFIG_HOME/wivrn/
// (server/driver/configuration.cpp:52-53) as known_keys.json and cookie, and
// parses the former with nlohmann::json. Neither file crosses the wire, so
// there is nothing to stay byte-compatible with and no reason to drag a JSON
// dependency into this tree: the key list here is a line-oriented text file.
//
// Deliberately a plain Win32 translation unit — no <filesystem>, no <fstream>,
// no wivrn header. Everything in the helper that links wivrn-common-net
// inherits its POSIX shadow headers, and the shadow <unistd.h> defines close()
// as a macro; keeping this file clear of them keeps a `stream.close()` from
// being rewritten out from under the standard library.
#pragma once

#include <string>
#include <vector>

namespace wivrnnx::helper
{

// %APPDATA%\wivrnnx, created on first use. Empty on failure.
std::string appdata_dir();

// 32 alphanumeric characters, generated once and then stable for this machine.
// Same role and shape as server_cookie() in
// server/driver/configuration.cpp:402 — the client keys its saved-server list
// on it (client/configuration.cpp:258), so it must survive a helper restart.
std::string server_cookie();

// Six decimal digits from the system CSPRNG, the pairing PIN. Same shape as the
// one server/main.cpp:870-878 generates and shows in a desktop notification.
std::string random_pin();

struct HeadsetKey
{
	// The headset's X448 public key, PEM with the armour and all whitespace
	// stripped — the same normalisation clean_key() does in
	// server/driver/wivrn_connection.cpp:47.
	std::string public_key;
	std::string name;
};

std::vector<HeadsetKey> known_keys();

// True if this key has been paired before. The comparison is on the cleaned
// form, so callers must pass a cleaned key.
bool is_key_known(const std::string & cleaned_public_key);

// Appends a newly paired headset. A duplicate name is disambiguated the way
// add_known_key() does on Linux; a duplicate key is a no-op.
void add_known_key(const HeadsetKey & key);

} // namespace wivrnnx::helper
