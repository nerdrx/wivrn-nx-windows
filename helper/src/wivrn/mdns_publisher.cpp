#include "mdns_publisher.h"

// This translation unit inherits the POSIX shadow headers from
// wivrn-common-net's PUBLIC include directory, and mdns.h pulls <fcntl.h> in,
// which chains to them. That is a problem here in a way it is not in the rest
// of the helper: the shadows install global overloads of recvfrom/sendto that
// take `int fd` and `void *`, and mdns.h calls them with an int socket and a
// `char *` buffer, which makes neither the shadow nor the Winsock declaration
// a better match — every call comes out ambiguous.
//
// WIVRNNX_NET_IMPL is win_net.h's own switch for exactly this (win_net.cpp
// defines it so the implementation does not recurse into its own forwarders).
// With it set, this file gets the Winsock declarations mdns.h was written
// against and none of the POSIX veneer, while wivrn::win::ensure_winsock() —
// declared above the switch — is still available.
#define WIVRNNX_NET_IMPL
#include "win_net.h"

#include <iphlpapi.h>

#include <mdns.h>

#include <windows.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../log.h"

namespace wivrnnx::helper
{

namespace
{

// Avahi's own defaults, which is what a WiVRn client on the same network is
// used to seeing: a long TTL for the service records, a short one for the
// addresses.
constexpr uint32_t kServiceTtl = 4500;
constexpr uint32_t kAddressTtl = 120;

// Re-announce this often. Nothing requires it — the client re-queries every
// 5 s (wivrn_discover::discover_period) — but an unsolicited answer after a
// Wi-Fi hiccup gets the headset's list refreshed without waiting for its next
// sweep.
constexpr DWORD kAnnounceIntervalMs = 60'000;

constexpr size_t kBufferSize = 2048;

const char kDnsSdQuery[] = "_services._dns-sd._udp.local.";

mdns_string_t to_mdns(const std::string & s)
{
	return mdns_string_t{s.c_str(), s.size()};
}

// A DNS label may not contain a dot and DNS-SD instance names are not escaped
// by mdns.h's string builder, which splits on '.' to make labels. The Linux
// server hands avahi a pretty hostname and lets it deal with that; here the
// dots simply become dashes.
std::string sanitize_label(std::string s)
{
	std::string out;
	out.reserve(s.size());
	for (char c: s)
	{
		if (c == '.' || c == '\0')
			out.push_back('-');
		else
			out.push_back(c);
	}
	if (out.empty())
		out = "wivrnnx";
	return out;
}

struct Socket
{
	int fd = -1;
	bool ipv6 = false;
	mdns_record_t record_a{};
	mdns_record_t record_aaaa{};
	bool has_a = false;
	bool has_aaaa = false;
};

} // namespace

struct MdnsPublisher::Impl
{
	std::string instance_label;   // "MYPC"
	std::string host_label;       // "MYPC"
	std::string service;          // "_wivrn._tcp.local."
	std::string service_instance; // "MYPC._wivrn._tcp.local."
	std::string host_qualified;   // "MYPC.local."
	int port = 0;

	// The TXT strings have to outlive every send: mdns_record_t only holds
	// pointers into them.
	std::vector<std::string> txt_keys;
	std::vector<std::string> txt_values;

	std::vector<Socket> sockets;
	alignas(8) unsigned char buffer[kBufferSize]{};

	mdns_record_t ptr_record() const
	{
		mdns_record_t r{};
		r.name = to_mdns(service);
		r.type = MDNS_RECORDTYPE_PTR;
		r.data.ptr.name = to_mdns(service_instance);
		r.rclass = MDNS_CLASS_IN;
		r.ttl = kServiceTtl;
		return r;
	}

	mdns_record_t srv_record() const
	{
		mdns_record_t r{};
		r.name = to_mdns(service_instance);
		r.type = MDNS_RECORDTYPE_SRV;
		r.data.srv.priority = 0;
		r.data.srv.weight = 0;
		r.data.srv.port = static_cast<uint16_t>(port);
		r.data.srv.name = to_mdns(host_qualified);
		r.rclass = MDNS_CLASS_IN;
		r.ttl = kServiceTtl;
		return r;
	}

	std::vector<mdns_record_t> txt_records() const
	{
		std::vector<mdns_record_t> records;
		records.reserve(txt_keys.size());
		for (size_t i = 0; i < txt_keys.size(); ++i)
		{
			mdns_record_t r{};
			r.name = to_mdns(service_instance);
			r.type = MDNS_RECORDTYPE_TXT;
			r.data.txt.key = to_mdns(txt_keys[i]);
			r.data.txt.value = to_mdns(txt_values[i]);
			r.rclass = MDNS_CLASS_IN;
			r.ttl = kServiceTtl;
			records.push_back(r);
		}
		return records;
	}

	// Everything but the PTR, in the order a resolver wants it: SRV, then the
	// addresses of the interface the query came in on, then the TXT set.
	std::vector<mdns_record_t> additional_for(const Socket & sock) const
	{
		std::vector<mdns_record_t> extra;
		extra.push_back(srv_record());
		if (sock.has_a)
			extra.push_back(sock.record_a);
		if (sock.has_aaaa)
			extra.push_back(sock.record_aaaa);
		for (const mdns_record_t & r: txt_records())
			extra.push_back(r);
		return extra;
	}

	bool open_sockets();
	void announce(bool goodbye);
	static int callback(int sock,
	                    const struct sockaddr * from,
	                    size_t addrlen,
	                    mdns_entry_type_t entry,
	                    uint16_t query_id,
	                    uint16_t rtype,
	                    uint16_t rclass,
	                    uint32_t ttl,
	                    const void * data,
	                    size_t size,
	                    size_t name_offset,
	                    size_t name_length,
	                    size_t record_offset,
	                    size_t record_length,
	                    void * user_data);
};

namespace
{

// Name of the answer that would go out for a given question, or nullptr if the
// question is not for us. Kept out of the callback so the matching rules read
// as one list.
enum class Answer
{
	none,
	dns_sd_ptr,
	service_ptr,
	srv,
	txt,
	a,
	aaaa,
};

Answer classify(const MdnsPublisher::Impl & impl, const std::string & name, uint16_t rtype)
{
	const bool any = rtype == MDNS_RECORDTYPE_ANY;

	if (name == kDnsSdQuery)
		return (any || rtype == MDNS_RECORDTYPE_PTR) ? Answer::dns_sd_ptr : Answer::none;

	if (name == impl.service)
		return (any || rtype == MDNS_RECORDTYPE_PTR) ? Answer::service_ptr : Answer::none;

	if (name == impl.service_instance)
	{
		if (any || rtype == MDNS_RECORDTYPE_SRV)
			return Answer::srv;
		if (rtype == MDNS_RECORDTYPE_TXT)
			return Answer::txt;
		return Answer::none;
	}

	if (name == impl.host_qualified)
	{
		if (any || rtype == MDNS_RECORDTYPE_A)
			return Answer::a;
		if (rtype == MDNS_RECORDTYPE_AAAA)
			return Answer::aaaa;
	}

	return Answer::none;
}

} // namespace

int MdnsPublisher::Impl::callback(int sock,
                                  const struct sockaddr * from,
                                  size_t addrlen,
                                  mdns_entry_type_t entry,
                                  uint16_t query_id,
                                  uint16_t rtype,
                                  uint16_t rclass,
                                  uint32_t /*ttl*/,
                                  const void * data,
                                  size_t size,
                                  size_t name_offset,
                                  size_t /*name_length*/,
                                  size_t /*record_offset*/,
                                  size_t /*record_length*/,
                                  void * user_data)
{
	if (entry != MDNS_ENTRYTYPE_QUESTION)
		return 0;

	Impl & impl = *static_cast<Impl *>(user_data);

	const Socket * self = nullptr;
	for (const Socket & s: impl.sockets)
	{
		if (s.fd == sock)
		{
			self = &s;
			break;
		}
	}
	if (self == nullptr)
		return 0;

	char name_buffer[256];
	size_t offset = name_offset;
	const mdns_string_t name = mdns_string_extract(data, size, &offset, name_buffer, sizeof(name_buffer));
	const std::string question{name.str, name.length};

	const Answer answer = classify(impl, question, rtype);
	if (answer == Answer::none)
		return 0;

	// The top bit of the query class asks for a unicast reply. Honouring it is
	// what keeps a one-shot resolver (which binds an ephemeral port and would
	// never see the multicast) working.
	const bool unicast = (rclass & MDNS_UNICAST_RESPONSE) != 0;

	mdns_record_t record{};
	std::vector<mdns_record_t> additional;
	std::vector<mdns_record_t> txt = impl.txt_records();

	switch (answer)
	{
		case Answer::dns_sd_ptr:
			// The service type itself, not an instance: a browser enumerating
			// what lives on this host. No additional records.
			record.name = mdns_string_t{kDnsSdQuery, sizeof(kDnsSdQuery) - 1};
			record.type = MDNS_RECORDTYPE_PTR;
			record.data.ptr.name = to_mdns(impl.service);
			record.rclass = MDNS_CLASS_IN;
			record.ttl = kServiceTtl;
			break;

		case Answer::service_ptr:
			record = impl.ptr_record();
			additional = impl.additional_for(*self);
			break;

		case Answer::srv:
			record = impl.srv_record();
			if (self->has_a)
				additional.push_back(self->record_a);
			if (self->has_aaaa)
				additional.push_back(self->record_aaaa);
			for (const mdns_record_t & r: txt)
				additional.push_back(r);
			break;

		case Answer::txt:
			if (txt.empty())
				return 0;
			record = txt.front();
			additional.assign(txt.begin() + 1, txt.end());
			break;

		case Answer::a:
			if (!self->has_a)
				return 0;
			record = self->record_a;
			if (self->has_aaaa)
				additional.push_back(self->record_aaaa);
			break;

		case Answer::aaaa:
			if (!self->has_aaaa)
				return 0;
			record = self->record_aaaa;
			if (self->has_a)
				additional.push_back(self->record_a);
			break;

		case Answer::none:
			return 0;
	}

	alignas(8) unsigned char reply[kBufferSize];

	if (unicast)
	{
		mdns_query_answer_unicast(sock,
		                          from,
		                          addrlen,
		                          reply,
		                          sizeof(reply),
		                          query_id,
		                          static_cast<mdns_record_type_t>(rtype),
		                          name.str,
		                          name.length,
		                          record,
		                          nullptr,
		                          0,
		                          additional.data(),
		                          additional.size());
	}
	else
	{
		mdns_query_answer_multicast(sock,
		                            reply,
		                            sizeof(reply),
		                            record,
		                            nullptr,
		                            0,
		                            additional.data(),
		                            additional.size());
	}

	return 0;
}

bool MdnsPublisher::Impl::open_sockets()
{
	// One socket per interface address, each bound to that address on port 5353.
	// That is what lets the A record we answer with be the address of the
	// interface the query actually arrived on, which is the whole reason
	// upstream mdns.c does it this way too.
	ULONG size = 16 * 1024;
	std::vector<unsigned char> storage;
	IP_ADAPTER_ADDRESSES * adapters = nullptr;
	ULONG ret = ERROR_BUFFER_OVERFLOW;

	for (int attempt = 0; attempt < 4 && ret == ERROR_BUFFER_OVERFLOW; ++attempt)
	{
		storage.assign(size, 0);
		adapters = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(storage.data());
		ret = GetAdaptersAddresses(AF_UNSPEC,
		                           GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
		                                   GAA_FLAG_SKIP_DNS_SERVER,
		                           nullptr,
		                           adapters,
		                           &size);
	}

	if (ret != NO_ERROR)
	{
		log_win32(ret, "GetAdaptersAddresses failed, mDNS announce disabled");
		return false;
	}

	for (IP_ADAPTER_ADDRESSES * a = adapters; a != nullptr; a = a->Next)
	{
		if (a->OperStatus != IfOperStatusUp)
			continue;

		for (IP_ADAPTER_UNICAST_ADDRESS * u = a->FirstUnicastAddress; u != nullptr; u = u->Next)
		{
			const sockaddr * sa = u->Address.lpSockaddr;

			if (sa->sa_family == AF_INET)
			{
				sockaddr_in addr{};
				std::memcpy(&addr, sa, sizeof(addr));
				addr.sin_port = htons(MDNS_PORT);

				// Loopback cannot join a multicast group in a way any peer
				// would see; skip it rather than log a failure per start.
				if (ntohl(addr.sin_addr.s_addr) == INADDR_LOOPBACK)
					continue;

				const int fd = mdns_socket_open_ipv4(&addr);
				if (fd < 0)
					continue;

				Socket s;
				s.fd = fd;
				s.ipv6 = false;
				s.has_a = true;
				s.record_a.name = to_mdns(host_qualified);
				s.record_a.type = MDNS_RECORDTYPE_A;
				s.record_a.data.a.addr = addr;
				s.record_a.data.a.addr.sin_port = 0;
				s.record_a.rclass = MDNS_CLASS_IN;
				s.record_a.ttl = kAddressTtl;
				sockets.push_back(s);

				char text[INET_ADDRSTRLEN]{};
				inet_ntop(AF_INET, &addr.sin_addr, text, sizeof(text));
				log_line("mDNS: listening on %s:%d", text, MDNS_PORT);
			}
			else if (sa->sa_family == AF_INET6)
			{
				sockaddr_in6 addr{};
				std::memcpy(&addr, sa, sizeof(addr));
				addr.sin6_port = htons(MDNS_PORT);

				static const unsigned char loopback[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
				if (std::memcmp(&addr.sin6_addr, loopback, 16) == 0)
					continue;

				const int fd = mdns_socket_open_ipv6(&addr);
				if (fd < 0)
					continue;

				Socket s;
				s.fd = fd;
				s.ipv6 = true;
				s.has_aaaa = true;
				s.record_aaaa.name = to_mdns(host_qualified);
				s.record_aaaa.type = MDNS_RECORDTYPE_AAAA;
				s.record_aaaa.data.aaaa.addr = addr;
				s.record_aaaa.data.aaaa.addr.sin6_port = 0;
				s.record_aaaa.rclass = MDNS_CLASS_IN;
				s.record_aaaa.ttl = kAddressTtl;
				sockets.push_back(s);

				char text[INET6_ADDRSTRLEN]{};
				inet_ntop(AF_INET6, &addr.sin6_addr, text, sizeof(text));
				log_line("mDNS: listening on [%s]:%d", text, MDNS_PORT);
			}
		}
	}

	return !sockets.empty();
}

void MdnsPublisher::Impl::announce(bool goodbye)
{
	for (const Socket & s: sockets)
	{
		const mdns_record_t record = ptr_record();
		std::vector<mdns_record_t> additional = additional_for(s);

		if (goodbye)
			mdns_goodbye_multicast(s.fd, buffer, sizeof(buffer), record, nullptr, 0, additional.data(), additional.size());
		else
			mdns_announce_multicast(s.fd, buffer, sizeof(buffer), record, nullptr, 0, additional.data(), additional.size());
	}
}

MdnsPublisher::MdnsPublisher(std::string instance_name,
                             std::string host_name,
                             int port,
                             std::map<std::string, std::string> txt) :
        impl_(new Impl)
{
	impl_->instance_label = sanitize_label(std::move(instance_name));
	impl_->host_label = sanitize_label(std::move(host_name));
	impl_->service = "_wivrn._tcp.local.";
	impl_->service_instance = impl_->instance_label + "." + impl_->service;
	impl_->host_qualified = impl_->host_label + ".local.";
	impl_->port = port;

	for (const auto & [key, value]: txt)
	{
		impl_->txt_keys.push_back(key);
		impl_->txt_values.push_back(value);
	}
}

MdnsPublisher::~MdnsPublisher()
{
	for (const Socket & s: impl_->sockets)
		mdns_socket_close(s.fd);
	delete impl_;
}

bool MdnsPublisher::start()
{
	wivrn::win::ensure_winsock();

	if (!impl_->open_sockets())
	{
		log_line("mDNS: no usable interface, the headset will have to connect by address");
		return false;
	}

	impl_->announce(false);

	log_line("mDNS: announced \"%s\" on port %d", impl_->service_instance.c_str(), impl_->port);
	for (size_t i = 0; i < impl_->txt_keys.size(); ++i)
		log_line("mDNS:   TXT %s=%s", impl_->txt_keys[i].c_str(), impl_->txt_values[i].c_str());

	return true;
}

void MdnsPublisher::run(void * shutdown_event)
{
	HANDLE stop = static_cast<HANDLE>(shutdown_event);
	DWORD next_announce = kAnnounceIntervalMs;

	for (;;)
	{
		// The sockets are non-blocking (mdns_socket_setup_* sets FIONBIO), so
		// this is a readability wait, not a receive.
		fd_set readable;
		FD_ZERO(&readable);
		int max_fd = 0;
		for (const Socket & s: impl_->sockets)
		{
			FD_SET(static_cast<SOCKET>(s.fd), &readable);
			if (s.fd > max_fd)
				max_fd = s.fd;
		}

		timeval timeout{0, 100'000}; // 100 ms, so the stop event is honoured promptly
		const int ready = ::select(max_fd + 1, &readable, nullptr, nullptr, &timeout);

		if (ready > 0)
		{
			for (const Socket & s: impl_->sockets)
			{
				if (FD_ISSET(static_cast<SOCKET>(s.fd), &readable))
					mdns_socket_listen(s.fd, impl_->buffer, sizeof(impl_->buffer), &Impl::callback, impl_);
			}
		}

		if (WaitForSingleObject(stop, 0) == WAIT_OBJECT_0)
			break;

		if (next_announce <= 100)
		{
			impl_->announce(false);
			next_announce = kAnnounceIntervalMs;
		}
		else
		{
			next_announce -= 100;
		}
	}

	impl_->announce(true);
	log_line("mDNS: goodbye sent for \"%s\"", impl_->service_instance.c_str());
}

std::string default_instance_name()
{
	wchar_t name[MAX_COMPUTERNAME_LENGTH + 1]{};
	DWORD size = MAX_COMPUTERNAME_LENGTH + 1;
	if (!GetComputerNameW(name, &size))
		return "wivrnnx";

	char utf8[(MAX_COMPUTERNAME_LENGTH + 1) * 4]{};
	if (WideCharToMultiByte(CP_UTF8, 0, name, -1, utf8, sizeof(utf8), nullptr, nullptr) <= 0)
		return "wivrnnx";

	return utf8;
}

} // namespace wivrnnx::helper
