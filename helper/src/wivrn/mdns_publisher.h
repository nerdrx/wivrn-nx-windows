// DNS-SD announcement of the _wivrn._tcp service, without avahi.
//
// The Linux server publishes through avahi (server/main.cpp start_publishing()).
// There is no avahi on Windows, so this is a minimal responder built directly on
// the single-file mdns.h library that the *client* already uses for discovery
// (${WIVRNNX_LINUX_REPO}/external/mdns.h) — the two ends therefore share one
// implementation of the wire format.
//
// The service name, the instance-name convention and the TXT record set are
// replicated exactly from server/main.cpp:610-631; the client filters on them
// (client/scenes/lobby.cpp:215-221 rejects a server whose "protocol" TXT does
// not match its own protocol_version, and keys its server list on "cookie").
#pragma once

#include <map>
#include <string>

namespace wivrnnx::helper
{

class MdnsPublisher
{
public:
	// Opaque; defined in the translation unit, where the mdns.h record types
	// exist. Public only so the file-scope helpers there can name it.
	struct Impl;

	MdnsPublisher(std::string instance_name,
	              std::string host_name,
	              int port,
	              std::map<std::string, std::string> txt);
	~MdnsPublisher();

	MdnsPublisher(const MdnsPublisher &) = delete;
	MdnsPublisher & operator=(const MdnsPublisher &) = delete;

	// Opens one socket per local interface address, bound to 5353, and sends the
	// unsolicited announcement. False if no socket could be opened at all.
	bool start();

	// Answers queries until the shutdown event is signalled, then sends the
	// goodbye records. `shutdown_event` is a Win32 HANDLE.
	void run(void * shutdown_event);

private:
	Impl * impl_;
};

// The service instance name the Linux server would use, adapted: a DNS-SD label
// may not contain a dot, and the Windows computer name is the closest thing to
// the pretty hostname `wivrn::hostname()` returns (server/hostname.cpp).
std::string default_instance_name();

} // namespace wivrnnx::helper
