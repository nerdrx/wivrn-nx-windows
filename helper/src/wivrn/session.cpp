#include "session.h"

#include "win_net.h" // winsock2 first

#include <poll.h>

#include <openssl/rand.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <variant>

#include "../log.h"
#include "crypto.h"
#include "protocol_version.h"
#include "secrets.h"
#include "smp.h"
#include "store.h"
#include "video_out.h"

using namespace std::chrono_literals;

namespace wivrnnx::helper
{

namespace
{

template <typename... Ts>
struct overloaded : Ts...
{
	using Ts::operator()...;
};
template <typename... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// Strip the PEM armour and every space, so that two spellings of the same key
// compare equal. Same normalisation as clean_key() in
// server/driver/wivrn_connection.cpp:47, done without <regex> — it is three
// fixed rules and pulling std::regex into a static mingw binary costs about
// half a megabyte.
std::string clean_key(const std::string & key)
{
	std::string out;
	out.reserve(key.size());

	size_t pos = 0;
	while (pos < key.size())
	{
		size_t eol = key.find('\n', pos);
		if (eol == std::string::npos)
			eol = key.size();

		std::string line = key.substr(pos, eol - pos);
		pos = eol + 1;

		// "-----BEGIN ...-----" / "-----END ...-----"
		const bool armour = line.find("-----BEGIN") != std::string::npos ||
		                    line.find("-----END") != std::string::npos ||
		                    (line.size() > 1 && line[0] == '-');
		if (armour)
			continue;

		for (char c: line)
		{
			if (c != ' ' && c != '\t' && c != '\r' && c != '\n' && c != '\f' && c != '\v')
				out.push_back(c);
		}
	}

	return out;
}

std::array<uint8_t, 16> random_session_token()
{
	std::array<uint8_t, 16> token{};
	// OpenSSL's CSPRNG rather than std::random_device, whose mingw
	// implementation has historically been deterministic. The token is what a
	// secondary path would present to prove it owns this session, so a
	// predictable one would be a real hole even though nothing attaches
	// secondary paths in this phase.
	if (RAND_bytes(token.data(), static_cast<int>(token.size())) != 1)
		throw std::runtime_error("RAND_bytes failed");
	return token;
}

sockaddr_in6 local_of(int fd)
{
	sockaddr_in6 addr{};
	socklen_t len = sizeof(addr);
	if (getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len) < 0)
		throw std::system_error{errno, std::generic_category(), "getsockname"};
	return addr;
}

sockaddr_in6 peer_of(int fd)
{
	sockaddr_in6 addr{};
	socklen_t len = sizeof(addr);
	if (getpeername(fd, reinterpret_cast<sockaddr *>(&addr), &len) < 0)
		throw std::system_error{errno, std::generic_category(), "getpeername"};
	return addr;
}

// wivrn::device_id -> the three devices the IPC contract knows about. Anything
// else (aim poses, palms, hands, body, face, eye gaze) has no shim-side device
// in Phase 1 and is discarded.
//
// The grip pose is the one SteamVR calls the controller's raw pose, so that is
// what is forwarded; LEFT_AIM/RIGHT_AIM are still requested from the client
// because a later phase wants them for the pointer pose.
bool ipc_device_for(wivrn::device_id id, ipc::DeviceId & out)
{
	switch (id)
	{
		case wivrn::device_id::HEAD:
			out = ipc::DeviceId::hmd;
			return true;
		case wivrn::device_id::LEFT_GRIP:
			out = ipc::DeviceId::left_controller;
			return true;
		case wivrn::device_id::RIGHT_GRIP:
			out = ipc::DeviceId::right_controller;
			return true;
		default:
			return false;
	}
}

struct ButtonMapping
{
	wivrn::device_id id;
	ipc::DeviceId device;
	ipc::Button button;
	bool touch; // false: pressed bit, true: touched bit
};

// The Pico 4 / Pico 4 Ultra controller layout, which is the shape ipc::Button
// was frozen around: X/Y and menu on the left, A/B and system on the right.
constexpr ButtonMapping kButtons[] = {
        {wivrn::device_id::X_CLICK, ipc::DeviceId::left_controller, ipc::Button::x, false},
        {wivrn::device_id::X_TOUCH, ipc::DeviceId::left_controller, ipc::Button::x, true},
        {wivrn::device_id::Y_CLICK, ipc::DeviceId::left_controller, ipc::Button::y, false},
        {wivrn::device_id::Y_TOUCH, ipc::DeviceId::left_controller, ipc::Button::y, true},
        {wivrn::device_id::MENU_CLICK, ipc::DeviceId::left_controller, ipc::Button::menu, false},
        {wivrn::device_id::LEFT_TRIGGER_CLICK, ipc::DeviceId::left_controller, ipc::Button::trigger_click, false},
        {wivrn::device_id::LEFT_TRIGGER_TOUCH, ipc::DeviceId::left_controller, ipc::Button::trigger_touch, true},
        {wivrn::device_id::LEFT_SQUEEZE_CLICK, ipc::DeviceId::left_controller, ipc::Button::grip_click, false},
        {wivrn::device_id::LEFT_THUMBSTICK_CLICK, ipc::DeviceId::left_controller, ipc::Button::thumbstick_click, false},
        {wivrn::device_id::LEFT_THUMBSTICK_TOUCH, ipc::DeviceId::left_controller, ipc::Button::thumbstick_touch, true},

        {wivrn::device_id::A_CLICK, ipc::DeviceId::right_controller, ipc::Button::a, false},
        {wivrn::device_id::A_TOUCH, ipc::DeviceId::right_controller, ipc::Button::a, true},
        {wivrn::device_id::B_CLICK, ipc::DeviceId::right_controller, ipc::Button::b, false},
        {wivrn::device_id::B_TOUCH, ipc::DeviceId::right_controller, ipc::Button::b, true},
        {wivrn::device_id::SYSTEM_CLICK, ipc::DeviceId::right_controller, ipc::Button::system, false},
        {wivrn::device_id::RIGHT_TRIGGER_CLICK, ipc::DeviceId::right_controller, ipc::Button::trigger_click, false},
        {wivrn::device_id::RIGHT_TRIGGER_TOUCH, ipc::DeviceId::right_controller, ipc::Button::trigger_touch, true},
        {wivrn::device_id::RIGHT_SQUEEZE_CLICK, ipc::DeviceId::right_controller, ipc::Button::grip_click, false},
        {wivrn::device_id::RIGHT_THUMBSTICK_CLICK, ipc::DeviceId::right_controller, ipc::Button::thumbstick_click, false},
        {wivrn::device_id::RIGHT_THUMBSTICK_TOUCH, ipc::DeviceId::right_controller, ipc::Button::thumbstick_touch, true},
};

// The tracking pattern the client is asked for. The Linux server derives it
// from what the compositor actually queried (tracking_control::resolve); with
// no compositor here it is a fixed list, sampled at the production timestamp.
// The client always adds HEAD itself (client/scenes/stream_tracking.cpp:332),
// but naming it keeps the request explicit.
constexpr wivrn::device_id kTrackedDevices[] = {
        wivrn::device_id::HEAD,
        wivrn::device_id::LEFT_GRIP,
        wivrn::device_id::LEFT_AIM,
        wivrn::device_id::RIGHT_GRIP,
        wivrn::device_id::RIGHT_AIM,
};

} // namespace

Session::Session(wivrn::TCP && tcp,
                 const ServerOptions & options,
                 Bridge & bridge,
                 VideoBridge & video,
                 std::atomic<bool> & stop) :
        control_(std::move(tcp)),
        options_(options),
        bridge_(bridge),
        video_(video),
        stop_(stop),
        token_(random_session_token())
{
	handshake();
}

Session::~Session()
{
	if (stream_)
		::shutdown(stream_.get_fd(), SHUT_RDWR);
	if (control_)
		::shutdown(control_.get_fd(), SHUT_RDWR);

	bridge_.on_client_gone();
	// The encoder thread stops encoding on this and answers every further
	// FrameReady with FrameDone flags=1, which is what keeps SteamVR presenting
	// happily into a ring nobody reads while the headset is away.
	video_.clear_client();
}

std::pair<wivrn::from_headset::packets, int> Session::receive_one(std::chrono::seconds timeout, bool allow_stream)
{
	const auto deadline = clock::now() + timeout;

	for (;;)
	{
		if (stop_.load())
			throw std::runtime_error("Connection cancelled");

		pollfd fds[2]{};
		fds[0].fd = allow_stream && stream_ ? stream_.get_fd() : -1;
		fds[0].events = POLLIN;
		fds[1].fd = control_.get_fd();
		fds[1].events = POLLIN;

		if (::poll(fds, 2, 100) < 0)
			throw std::system_error(errno, std::system_category(), "poll");

		if (fds[0].fd >= 0 && (fds[0].revents & (POLLHUP | POLLERR)))
			throw std::runtime_error("Error on stream socket");

		if (fds[1].revents & (POLLHUP | POLLERR))
			throw std::runtime_error("Error on control socket");

		if (fds[0].fd >= 0 && (fds[0].revents & POLLIN))
		{
			auto [raw_packet, peer_addr] = stream_.receive_from_raw();
			if (not raw_packet.empty())
			{
				// A malformed datagram must not abort the handshake.
				try
				{
					return {raw_packet.deserialize<wivrn::from_headset::packets>(),
					        static_cast<int>(ntohs(peer_addr.sin6_port))};
				}
				catch (const std::exception &)
				{
					stream_.count_dropped_datagram();
				}
			}
		}

		if (fds[1].revents & POLLIN)
		{
			if (auto packet = control_.receive())
				return {std::move(*packet), -1};
		}

		if (clock::now() > deadline)
			throw std::runtime_error("No handshake packet received from client");
	}
}

void Session::handshake()
{
	// The UDP stream socket is bound to the same local address and port the
	// accepted TCP connection landed on, so the client can find it at the port
	// it already knows.
	const sockaddr_in6 server_address = local_of(control_.get_fd());
	const sockaddr_in6 client_address = peer_of(control_.get_fd());

	int stream_port = ntohs(server_address.sin6_port);
	if (options_.tcp_only)
	{
		stream_port = -1;
	}
	else
	{
		stream_ = decltype(stream_)();

		// wivrn::UDP opens an AF_INET6 socket; Winsock defaults IPV6_V6ONLY to
		// on, where Linux defaults it off. Without this the socket could
		// neither be bound to the IPv4-mapped local address an IPv4 headset's
		// TCP connection landed on, nor receive that headset's datagrams. Must
		// precede bind(), which is why it cannot live inside common/.
		int v6only = 0;
		if (setsockopt(stream_.get_fd(), IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only)) < 0)
			log_line("warning: could not clear IPV6_V6ONLY on the stream socket (errno %d)", errno);

		// Bind to the wildcard address on the stream port, NOT to the specific
		// IPv4-mapped local address the TCP control connection landed on. On
		// Windows a dual-stack UDP socket bound to a specific ::ffff:a.b.c.d
		// local address then refuses connect() to an IPv4-mapped peer with
		// WSAEADDRNOTAVAIL; binding to the wildcard lets the later connect()
		// to the client's mapped address succeed. Linux tolerates either.
		sockaddr_in6 bind_address = server_address;
		bind_address.sin6_addr = in6addr_any;
		stream_.bind(bind_address);
	}

	// --- crypto handshake ------------------------------------------------
	auto crypto_handshake = std::get<wivrn::from_headset::crypto_handshake>(receive_one(10s, false).first);

	if (crypto_handshake.protocol_version != wivrn::protocol_version)
	{
		control_.send(wivrn::to_headset::crypto_handshake{
		        .public_key = {},
		        .state = wivrn::to_headset::crypto_handshake::crypto_state::incompatible_version,
		});
		log_line("client protocol version %016llx, helper %016llx - refusing",
		         static_cast<unsigned long long>(crypto_handshake.protocol_version),
		         static_cast<unsigned long long>(wivrn::protocol_version));
		throw std::runtime_error("Incompatible protocol version");
	}

	crypto::key headset_key = crypto::key::from_public_key(crypto_handshake.public_key);
	const std::string cleaned = clean_key(crypto_handshake.public_key);
	const bool is_public_key_known = is_key_known(cleaned);

	log_line("headset \"%s\" connected (%s)",
	         crypto_handshake.name.c_str(),
	         is_public_key_known ? "already paired" : "not paired");

	switch (options_.encryption)
	{
		case EncryptionState::disabled:
			log_line("WARNING: encryption disabled, this connection is in the clear");
			control_.send(wivrn::to_headset::crypto_handshake{
			        .public_key = {},
			        .state = wivrn::to_headset::crypto_handshake::crypto_state::encryption_disabled,
			});
			break;

		case EncryptionState::enabled:
			if (not is_public_key_known)
			{
				control_.send(wivrn::to_headset::crypto_handshake{
				        .public_key = {},
				        .state = wivrn::to_headset::crypto_handshake::crypto_state::pairing_disabled,
				});
				throw std::runtime_error("Client not known and pairing is disabled");
			}
			[[fallthrough]];

		case EncryptionState::pairing: {
			// Ephemeral key pair, used only to agree the AES keys.
			crypto::key server_key = crypto::key::generate_x448_keypair();

			control_.send(wivrn::to_headset::crypto_handshake{
			        .public_key = server_key.public_key(),
			        .state = is_public_key_known
			                         ? wivrn::to_headset::crypto_handshake::crypto_state::client_already_paired
			                         : wivrn::to_headset::crypto_handshake::crypto_state::pin_needed,
			});

			if (not is_public_key_known)
			{
				log_line("================================================");
				log_line("  PAIRING: enter this PIN on the headset:  %s", options_.pin.c_str());
				log_line("================================================");

				try
				{
					// Socialist millionaires: both ends prove they know the
					// same PIN without either sending it.
					crypto::smp pin_check;

					auto msg1 = std::get<wivrn::from_headset::pin_check_1>(receive_one(2min, false).first).message;
					auto msg2 = pin_check.step2(msg1, options_.pin);
					control_.send(wivrn::to_headset::pin_check_2{msg2});

					auto msg3 = std::get<wivrn::from_headset::pin_check_3>(receive_one(10s, false).first).message;
					auto [msg4, pin_match] = pin_check.step4(msg3);
					control_.send(wivrn::to_headset::pin_check_4{msg4});

					if (not pin_match)
					{
						log_line("PIN mismatch, refusing the connection");
						throw incorrect_pin{};
					}
					log_line("handshake: PIN accepted");
				}
				catch (const crypto::smp_cheated &)
				{
					throw std::runtime_error("Unable to check PIN");
				}
			}

			secrets s{server_key, headset_key, is_public_key_known ? "000000" : options_.pin};
			control_.set_aes_key_and_ivs(s.control_key, s.control_iv_from_headset, s.control_iv_to_headset);
			if (stream_)
				stream_.set_aes_key_and_ivs(s.stream_key, s.stream_iv_header_from_headset, s.stream_iv_header_to_headset);
			break;
		}
	}

	// The client confirms it has switched its own sockets over to the agreed
	// keys; everything from here is encrypted.
	if (not std::holds_alternative<wivrn::from_headset::crypto_handshake>(receive_one(30s, false).first))
		throw std::runtime_error("No handshake received from client");
	log_line("handshake: encrypted control established");

	// --- stream socket ---------------------------------------------------
	control_.send(wivrn::to_headset::handshake{.stream_port = stream_port, .session_token = token_});

	auto [stream_handshake, client_port] = receive_one(10s, true);
	if (not std::holds_alternative<wivrn::from_headset::handshake>(stream_handshake))
		throw std::runtime_error("No handshake received from client");

	if (client_port >= 0)
	{
		char addr_str[INET6_ADDRSTRLEN] = "?";
		inet_ntop(AF_INET6, &client_address.sin6_addr, addr_str, sizeof(addr_str));
		log_line("handshake: connecting stream socket to [%s%%%u]:%d",
		         addr_str,
		         client_address.sin6_scope_id,
		         client_port);

		// wivrn::UDP::connect(in6_addr, port) drops sin6_scope_id, which a
		// link-local peer address (fe80::...) cannot survive: connect() then
		// fails with EADDRNOTAVAIL because the kernel cannot pick the
		// interface. The headset dials whichever of our mDNS addresses it
		// likes — including link-local — so connect with the full sockaddr
		// of the accepted TCP connection, scope id and all.
		sockaddr_in6 sa = client_address;
		sa.sin6_port = htons(static_cast<uint16_t>(client_port));
		if (::connect(stream_.get_fd(), reinterpret_cast<sockaddr *>(&sa), sizeof(sa)) < 0)
			throw std::system_error{errno, std::generic_category(), "stream connect"};
		stream_.set_send_buffer_size(1024 * 1024 * 5);
		log_line("stream socket connected, client UDP port %d", client_port);
	}
	else
	{
		// The client answered on TCP: it wants a control-only session.
		stream_ = decltype(stream_)(-1);
		log_line("client asked for a TCP-only session, no stream socket");
	}

	// Sent twice on purpose, exactly as the Linux server does: the second one
	// goes out after the stream socket exists, so the client can tell the
	// handshake completed on the path it will actually use.
	control_.send(wivrn::to_headset::handshake{.stream_port = stream_port, .session_token = token_});

	log_line("handshake: waiting for headset info");
	on_headset_info(std::get<wivrn::from_headset::headset_info_packet>(receive_one(10s, false).first));
	log_line("handshake: complete");

	if (options_.encryption == EncryptionState::pairing && not is_public_key_known)
		add_known_key(HeadsetKey{clean_key(headset_key.public_key()), crypto_handshake.name});

	clock_.reset();
	next_report_ = clock::now() + 5s;
}

template <typename T>
void Session::send_control(T && packet)
{
	control_.send(std::forward<T>(packet));
}

template <typename T>
void Session::send_stream(T && packet)
{
	if (stream_)
		stream_.send(std::forward<T>(packet));
	else
		control_.send(std::forward<T>(packet));
}

void Session::on_headset_info(const wivrn::from_headset::headset_info_packet & info)
{
	info_ = info;
	have_info_ = true;

	refresh_hz_ = info.settings.preferred_refresh_rate;
	if (!(refresh_hz_ > 1.f))
	{
		refresh_hz_ = 90.f;
		if (!info.available_refresh_rates.empty())
			refresh_hz_ = *std::max_element(info.available_refresh_rates.begin(), info.available_refresh_rates.end());
	}

	standby_freeze_ = info.settings.standby_freeze;

	log_line("headset info: %ux%u per eye (stream %ux%u), %.1f Hz, system \"%s\", standby freeze %s",
	         info.render_eye_width,
	         info.render_eye_height,
	         info.stream_eye_width,
	         info.stream_eye_height,
	         static_cast<double>(refresh_hz_),
	         info.system_name.c_str(),
	         standby_freeze_ ? "on" : "off");

	publish_config();
	publish_video_request();
}

void Session::on_settings(const wivrn::from_headset::settings_changed & settings)
{
	const bool freeze = settings.standby_freeze;
	if (freeze != standby_freeze_)
	{
		standby_freeze_ = freeze;
		log_line("standby freeze %s by the headset", freeze ? "enabled" : "disabled");
	}

	if (settings.preferred_refresh_rate > 1.f && settings.preferred_refresh_rate != refresh_hz_)
	{
		refresh_hz_ = settings.preferred_refresh_rate;
		publish_config();
		// A different frame rate is a different encoder: the rate control law and
		// the VBV are both sized from it.
		publish_video_request();
	}
}

void Session::publish_config()
{
	if (not have_info_)
		return;

	ipc::HmdConfig cfg{};
	cfg.eye_width = info_.render_eye_width;
	cfg.eye_height = info_.render_eye_height;
	cfg.refresh_hz = refresh_hz_;

	// The field of view and the eye-to-head transforms are taken from the
	// client's own view configuration. headset_info_packet::fov is what the
	// client measured at connect time; once tracking starts, tracking::views
	// carries both the fov and the per-eye pose *relative to VIEW space*, which
	// is exactly the eye-to-head transform SteamVR asks for, so that supersedes
	// it (and follows a runtime IPD change for free).
	std::array<XrFovf, 2> fov = info_.fov;

	if (have_views_)
	{
		for (int eye = 0; eye < 2; ++eye)
		{
			fov[eye] = views_[eye].fov;

			const XrQuaternionf & q = views_[eye].pose.orientation;
			cfg.eye_to_head_q[eye][0] = q.w;
			cfg.eye_to_head_q[eye][1] = q.x;
			cfg.eye_to_head_q[eye][2] = q.y;
			cfg.eye_to_head_q[eye][3] = q.z;

			cfg.eye_to_head_p[eye][0] = views_[eye].pose.position.x;
			cfg.eye_to_head_p[eye][1] = views_[eye].pose.position.y;
			cfg.eye_to_head_p[eye][2] = views_[eye].pose.position.z;
		}

		cfg.ipd_m = std::abs(views_[1].pose.position.x - views_[0].pose.position.x);
	}
	else
	{
		// Nothing better yet: identity rotations and a nominal 63 mm split, so
		// the shim has a usable HMD before the first tracking packet lands.
		cfg.ipd_m = 0.063f;
		for (int eye = 0; eye < 2; ++eye)
		{
			cfg.eye_to_head_q[eye][0] = 1.f;
			cfg.eye_to_head_p[eye][0] = (eye == 0 ? -0.5f : 0.5f) * cfg.ipd_m;
		}
	}

	// SteamVR's GetProjectionRaw(left, right, top, bottom) wants raw half-angle
	// tangents with the *top* edge negative. OpenXR's angleDown is the negative
	// one, so it is the one that goes into `top`. Cross-checked against ALVR:
	// reference/alvr/alvr/server_openvr/cpp/alvr_server/Utils.h:73-81 sets
	// vTopLeft.y = tanf(fov.down), vBottomRight.y = tanf(fov.up), and HMD.cpp
	// hands those to top/bottom respectively.
	for (int eye = 0; eye < 2; ++eye)
	{
		cfg.proj_left[eye] = std::tan(fov[eye].angleLeft);
		cfg.proj_right[eye] = std::tan(fov[eye].angleRight);
		cfg.proj_top[eye] = std::tan(fov[eye].angleDown);
		cfg.proj_bottom[eye] = std::tan(fov[eye].angleUp);
	}

	// Anything non-finite here would end up in a projection matrix; refuse it
	// rather than hand vrserver a NaN frustum.
	bool sane = cfg.eye_width > 0 && cfg.eye_height > 0 && is_finite(cfg.refresh_hz);
	for (int eye = 0; eye < 2 && sane; ++eye)
	{
		sane = is_finite(cfg.proj_left[eye]) && is_finite(cfg.proj_right[eye]) &&
		       is_finite(cfg.proj_top[eye]) && is_finite(cfg.proj_bottom[eye]) &&
		       cfg.proj_right[eye] > cfg.proj_left[eye] && cfg.proj_bottom[eye] > cfg.proj_top[eye];
	}

	if (not sane)
	{
		log_line("ignoring an implausible view configuration from the headset");
		return;
	}

	bridge_.set_config(cfg);
}

void Session::send_stream_setup()
{
	// The client only starts its tracking thread when a video stream
	// description arrives (client/scenes/stream_network.cpp:138-145): the
	// thread is created inside that handler and nowhere else. So one goes out
	// immediately, describing the stream the shim is *going* to produce — the
	// staging ring is 2 x render_eye_width wide by construction (see the
	// StagingConfig comment in ipc/wivrnnx_ipc.h), which is what the encoder
	// will cut in half.
	//
	// Deliberately not stream_eye_width, which is what this used to send and
	// what the Linux server encodes at: there is no scaler in this path, the
	// helper encodes the staging textures at their native size, and the
	// description has to name what the decoder will actually be fed. The moment
	// the encoder says otherwise, update_stream_description() corrects it.
	description_ = wivrn::to_headset::video_stream_description{};
	description_.width = static_cast<uint16_t>(info_.render_eye_width);
	description_.height = static_cast<uint16_t>(info_.render_eye_height);

	wivrn::video_codec codec = wivrn::video_codec::h264;
	if (not info_.supported_codecs.empty())
		codec = info_.supported_codecs.front();
	description_.codec = {codec, codec, codec, codec};

	description_.frame_rate = refresh_hz_;
	description_.refresh_rate = refresh_hz_;
	description_.quad_width = 0;
	description_.quad_height = 0;

	send_control(wivrn::to_headset::video_stream_description{description_});
	description_sent_ = true;
	log_line("sent a provisional video stream description (%ux%u per eye, %s) to start the "
	         "client's tracking thread",
	         description_.width,
	         description_.height,
	         codec == wivrn::video_codec::h265 ? "HEVC" : "H.264");
}

void Session::publish_video_request()
{
	// What the encoder may produce is whatever this client can decode. The list
	// arrives preferred-first; HEVC is asked for whenever it is in there at all,
	// because it is worth about 20% of the bitrate and every Pico decodes it.
	bool allow_h265 = false;
	bool allow_h264 = false;
	for (wivrn::video_codec c: info_.supported_codecs)
	{
		if (c == wivrn::video_codec::h265)
			allow_h265 = true;
		else if (c == wivrn::video_codec::h264)
			allow_h264 = true;
	}
	if (not allow_h265 and not allow_h264)
	{
		// A client that named only codecs this phase cannot produce still gets
		// H.264 tried at it: every decoder in the field has it, and refusing to
		// encode at all would be worse than a description the client may reject.
		log_line("headset advertises no codec this build can produce, trying H.264 anyway");
		allow_h264 = true;
	}

	video_.set_client(refresh_hz_, allow_h265, allow_h264);
}

void Session::update_stream_description()
{
	const VideoBridge::StreamState state = video_.stream_state();
	if (state.generation == stream_generation_)
		return;
	stream_generation_ = state.generation;

	if (not state.valid)
		return;

	wivrn::to_headset::video_stream_description desc = description_;
	desc.width = static_cast<uint16_t>(state.info.width);
	desc.height = static_cast<uint16_t>(state.info.height);
	const wivrn::video_codec codec = state.info.codec == VideoCodec::h265
	                                         ? wivrn::video_codec::h265
	                                         : wivrn::video_codec::h264;
	desc.codec = {codec, codec, codec, codec};
	desc.frame_rate = refresh_hz_;
	desc.refresh_rate = refresh_hz_;

	if (description_sent_ and desc == description_)
		return;

	description_ = desc;
	send_control(wivrn::to_headset::video_stream_description{description_});
	description_sent_ = true;

	// New decoders on the client means no parameter sets on the client.
	video_.request_idr("new stream description");
	++idr_requests_;
	idr_.reset();

	log_line("video stream description updated: %ux%u per eye, %s, %.1f Hz",
	         description_.width,
	         description_.height,
	         codec == wivrn::video_codec::h265 ? "HEVC" : "H.264",
	         static_cast<double>(refresh_hz_));
}

void Session::send_video_frame(EncodedFrame & frame)
{
	const ClockOffset offset = clock_.get_offset();
	const int64_t present_ns = qpc_to_ns(frame.sample_time_qpc);
	const int64_t photon_ns = present_ns + static_cast<int64_t>(static_cast<double>(frame.predict_s) * 1e9);
	const int64_t now_ns = monotonic_ns();

	VideoPacketizer::Frame out{};
	out.stream_index = frame.stream_index;
	out.frame_index = frame.frame_id;
	out.idr = frame.idr;
	out.has_stream_socket = static_cast<bool>(stream_);

	// server/compositor/compositor.cpp:635 puts the *predicted display time* here,
	// converted into the headset's clock. FrameReady gives the two halves of that
	// separately: the QPC at Present and the prediction interval SteamVR used, on
	// the same performance counter the tracking path already converts with
	// clock_offset — so a pose and the frame rendered from it land on the same
	// headset timeline and the reprojection has something consistent to work
	// against.
	out.view_info.display_time = offset.to_headset(photon_ns);
	out.view_info.alpha = false;
	out.view_info.quad.reset();

	XrPosef head{};
	head.orientation = XrQuaternionf{frame.pose_q[1], frame.pose_q[2], frame.pose_q[3], frame.pose_q[0]};
	head.position = XrVector3f{frame.pose_p[0], frame.pose_p[1], frame.pose_p[2]};

	for (int eye = 0; eye < 2; ++eye)
	{
		XrPosef eye_to_head{};
		eye_to_head.orientation = XrQuaternionf{0, 0, 0, 1};
		eye_to_head.position = XrVector3f{(eye == 0 ? -0.0315f : 0.0315f), 0.f, 0.f};
		XrFovf fov = info_.fov[eye];

		if (have_views_)
		{
			eye_to_head = views_[eye].pose;
			fov = views_[eye].fov;
		}

		out.view_info.pose[eye] = compose_pose(head, eye_to_head);
		out.view_info.fov[eye] = fov;
		out.view_info.foveation[eye] = identity_foveation(description_.width, description_.height);
	}

	// The headset only reads these for its latency plots. encode_begin is taken
	// as the moment the frame was presented, which is the earliest point in this
	// pipeline that means anything; the rest are now, because the encode and the
	// send happen on two different threads and the timestamps would otherwise
	// have to be carried across the bridge for a HUD.
	out.timing_info.encode_begin = offset.to_headset(present_ns);
	out.timing_info.encode_end = offset.to_headset(now_ns);
	out.timing_info.send_begin = offset.to_headset(now_ns);
	out.timing_info.send_end = offset.to_headset(now_ns);

	// Mutable, and not incidentally: video_stream_data_shard::payload is a
	// std::span<uint8_t> and the socket encrypts through it in place
	// (common/wivrn_sockets.cpp:410). Each shard covers a disjoint stretch of
	// this buffer and each goes out once, so encrypting one does not disturb the
	// next - but the buffer is consumed by this call and must not be reused.
	std::span<uint8_t> data(frame.data);

	uint16_t shards = 0;
	try
	{
		shards = VideoPacketizer::send(
		        out,
		        data,
		        [this](VideoPacketizer::Shard && shard, bool prefer_control) {
			        if (prefer_control)
				        send_control(std::move(shard));
			        else
				        send_stream(std::move(shard));
		        });
	}
	catch (const std::exception &)
	{
		// A frame lost to a socket error is one frame; the session's own poll
		// loop is what decides the connection is over.
		++video_send_errors_;
		return;
	}

	++video_frames_sent_;
	video_shards_sent_ += shards;
	video_bytes_sent_ += frame.data.size();
	idr_.on_frame_sent(frame.frame_id, frame.idr);
}

void Session::pump_video()
{
	video_scratch_.clear();
	if (video_.take_frames(video_scratch_) == 0)
		return;

	for (EncodedFrame & frame: video_scratch_)
		send_video_frame(frame);
}

void Session::on_feedback(const wivrn::from_headset::feedback & feedback)
{
	// Temporary lobby-stall diagnosis: show how far the client got with the
	// first frames of each stream (received -> decoded -> displayed).
	if (feedback_logged_ < 20)
	{
		++feedback_logged_;
		log_line("feedback: stream %u frame %llu recv=%d/%d decoder=%d/%d blit=%d disp=%d",
		         feedback.stream_index,
		         static_cast<unsigned long long>(feedback.frame_index),
		         feedback.received_first_packet != 0,
		         feedback.received_last_packet != 0,
		         feedback.sent_to_decoder != 0,
		         feedback.received_from_decoder != 0,
		         feedback.blitted != 0,
		         feedback.displayed != 0);
	}

	if (idr_.on_feedback(feedback))
	{
		video_.request_idr("headset lost a frame");
		++idr_requests_;
	}
}

void Session::send_tracking_pattern()
{
	wivrn::to_headset::tracking_control control{};
	for (wivrn::device_id id: kTrackedDevices)
		control.pattern.push_back({.device = id, .prediction_ns = 0});
	control.motions_to_photons = 0;

	send_control(std::move(control));
	log_line("requested tracking for head, both grips and both aim poses");
}

void Session::on_tracking(const wivrn::from_headset::tracking & tracking)
{
	++tracking_packets_;

	// The views carry the eye-to-head transforms and the live fov.
	if (not have_views_ ||
	    std::memcmp(&views_, &tracking.views, sizeof(views_)) != 0)
	{
		bool usable = true;
		for (const auto & view: tracking.views)
		{
			if (not is_valid_orientation(view.pose.orientation) or not is_finite_vec(view.pose.position))
				usable = false;
		}

		if (usable)
		{
			views_ = tracking.views;
			have_views_ = true;
			publish_config();
		}
	}

	const ClockOffset offset = clock_.get_offset();
	const uint64_t time_qpc = offset
	                                  ? ns_to_qpc(offset.from_headset(tracking.timestamp))
	                                  : ns_to_qpc(monotonic_ns());

	for (const auto & device_pose: tracking.device_poses)
	{
		ipc::DeviceId device{};
		if (not ipc_device_for(device_pose.device, device))
			continue;

		const size_t index = device_index(device);
		auto result = sanitizers_[index].sanitize(device_pose,
		                                          tracking.production_timestamp,
		                                          tracking.timestamp,
		                                          standby_freeze_);

		ipc::PoseUpdate out{};
		out.time_qpc = time_qpc;
		out.device = static_cast<uint8_t>(device);
		out.connected = result.usable ? 1 : 0;
		out.qw = result.orientation.w;
		out.qx = result.orientation.x;
		out.qy = result.orientation.y;
		out.qz = result.orientation.z;
		out.px = result.position.x;
		out.py = result.position.y;
		out.pz = result.position.z;
		out.vx = result.linear_velocity.x;
		out.vy = result.linear_velocity.y;
		out.vz = result.linear_velocity.z;
		out.wx = result.angular_velocity.x;
		out.wy = result.angular_velocity.y;
		out.wz = result.angular_velocity.z;

		bridge_.set_pose(out);

		if (result.usable)
			bridge_.set_present(device, true);
	}
}

void Session::on_inputs(const wivrn::from_headset::inputs & inputs)
{
	++input_packets_;

	const uint64_t time_qpc = ns_to_qpc(monotonic_ns());
	bool touched_device[kDeviceCount]{};

	for (const auto & value: inputs.values)
	{
		bool matched = false;

		for (const ButtonMapping & m: kButtons)
		{
			if (m.id != value.id)
				continue;

			ipc::InputUpdate & state = input_state_[device_index(m.device)];
			const uint32_t bit = static_cast<uint32_t>(m.button);
			uint32_t & field = m.touch ? state.touched : state.pressed;

			if (value.value != 0.f)
				field |= bit;
			else
				field &= ~bit;

			// A click implies a touch: some runtimes do not report the touch
			// component at all, and SteamVR input profiles expect both.
			if (not m.touch && value.value != 0.f)
				state.touched |= bit;

			touched_device[device_index(m.device)] = true;
			matched = true;
			break;
		}

		if (matched)
			continue;

		auto axis = [&](ipc::DeviceId device, float ipc::InputUpdate::* field) {
			ipc::InputUpdate & state = input_state_[device_index(device)];
			if (is_finite(value.value))
				state.*field = value.value;
			touched_device[device_index(device)] = true;
		};

		switch (value.id)
		{
			case wivrn::device_id::LEFT_TRIGGER_VALUE:
				axis(ipc::DeviceId::left_controller, &ipc::InputUpdate::trigger);
				break;
			case wivrn::device_id::RIGHT_TRIGGER_VALUE:
				axis(ipc::DeviceId::right_controller, &ipc::InputUpdate::trigger);
				break;
			case wivrn::device_id::LEFT_SQUEEZE_VALUE:
				axis(ipc::DeviceId::left_controller, &ipc::InputUpdate::grip);
				break;
			case wivrn::device_id::RIGHT_SQUEEZE_VALUE:
				axis(ipc::DeviceId::right_controller, &ipc::InputUpdate::grip);
				break;
			case wivrn::device_id::LEFT_THUMBSTICK_X:
				axis(ipc::DeviceId::left_controller, &ipc::InputUpdate::thumbstick_x);
				break;
			case wivrn::device_id::LEFT_THUMBSTICK_Y:
				axis(ipc::DeviceId::left_controller, &ipc::InputUpdate::thumbstick_y);
				break;
			case wivrn::device_id::RIGHT_THUMBSTICK_X:
				axis(ipc::DeviceId::right_controller, &ipc::InputUpdate::thumbstick_x);
				break;
			case wivrn::device_id::RIGHT_THUMBSTICK_Y:
				axis(ipc::DeviceId::right_controller, &ipc::InputUpdate::thumbstick_y);
				break;
			default:
				break;
		}
	}

	for (size_t i = 1; i < kDeviceCount; ++i)
	{
		if (not touched_device[i])
			continue;

		input_state_[i].time_qpc = time_qpc;
		input_state_[i].device = static_cast<uint8_t>(i);
		bridge_.set_input(input_state_[i]);
	}
}

void Session::pump_haptics()
{
	ipc::Haptic haptic{};
	while (bridge_.pop_haptic(haptic))
	{
		wivrn::device_id id{};
		switch (static_cast<ipc::DeviceId>(haptic.device))
		{
			case ipc::DeviceId::left_controller:
				id = wivrn::device_id::LEFT_CONTROLLER_HAPTIC;
				break;
			case ipc::DeviceId::right_controller:
				id = wivrn::device_id::RIGHT_CONTROLLER_HAPTIC;
				break;
			default:
				continue; // no haptic actuator on the HMD
		}

		if (not(is_finite(haptic.duration_s) and is_finite(haptic.frequency_hz) and is_finite(haptic.amplitude)))
			continue;

		const double duration_ns = static_cast<double>(haptic.duration_s) * 1e9;
		send_control(wivrn::to_headset::haptics{
		        .id = id,
		        .duration = std::chrono::nanoseconds(static_cast<int64_t>(std::clamp(duration_ns, 0.0, 5e9))),
		        .frequency = haptic.frequency_hz,
		        .amplitude = std::clamp(haptic.amplitude, 0.f, 1.f),
		});
	}
}

void Session::run()
{
	idr_.reset();
	send_stream_setup();
	send_tracking_pattern();

	auto visitor = overloaded{
	        [this](wivrn::from_headset::tracking && packet) { on_tracking(packet); },
	        [this](wivrn::from_headset::inputs && packet) { on_inputs(packet); },
	        [this](wivrn::from_headset::timesync_response && packet) { clock_.add_sample(packet); },
	        [this](wivrn::from_headset::settings_changed && packet) { on_settings(packet); },
	        [this](wivrn::from_headset::headset_info_packet && packet) { on_headset_info(packet); },
	        [this](wivrn::from_headset::feedback && packet) { on_feedback(packet); },
	        [this](wivrn::from_headset::refresh_rate_changed && packet) {
		        if (packet.to > 1.f)
		        {
			        refresh_hz_ = packet.to;
			        publish_config();
			        publish_video_request();
		        }
	        },
	        [this](wivrn::from_headset::path_ping && packet) {
		        // The keepalive on a secondary path. There is no secondary path
		        // here, but echoing it costs nothing and keeps a client that
		        // probed the USB tunnel from waiting on a reply.
		        send_control(wivrn::to_headset::path_pong{packet.path_id, packet.timestamp});
	        },
	        // Everything else - audio, hand/body/face tracking, battery, Wi-Fi
	        // state, the application list, HID forwarding - has no consumer in
	        // this phase and is read and dropped so the socket never backs up.
	        [this](auto &&) { ++discarded_packets_; },
	};

	for (;;)
	{
		if (stop_.load())
			return;

		// Whatever the last read already buffered, before going back to the
		// sockets. Same shape as wivrn_connection::poll().
		try
		{
			while (stream_)
			{
				auto packet = stream_.receive_pending_lossy();
				if (not packet)
					break;
				std::visit(visitor, std::move(*packet));
			}

			while (auto packet = control_.receive_pending())
				std::visit(visitor, std::move(*packet));
		}
		catch (const std::exception & e)
		{
			log_line("session ended: %s", e.what());
			return;
		}

		pollfd fds[2]{};
		fds[0].fd = stream_ ? stream_.get_fd() : -1;
		fds[0].events = POLLIN;
		fds[1].fd = control_.get_fd();
		fds[1].events = POLLIN;

		const int r = ::poll(fds, 2, 20);
		if (r < 0)
		{
			log_line("session ended: poll failed (errno %d)", errno);
			return;
		}

		if (fds[1].revents & (POLLHUP | POLLERR))
		{
			log_line("session ended: control socket closed");
			return;
		}

		try
		{
			if (fds[0].fd >= 0 && (fds[0].revents & POLLIN))
			{
				if (auto packet = stream_.receive_lossy())
					std::visit(visitor, std::move(*packet));
			}

			if (fds[1].revents & POLLIN)
			{
				if (auto packet = control_.receive())
					std::visit(visitor, std::move(*packet));
			}
		}
		catch (const std::exception & e)
		{
			log_line("session ended: %s", e.what());
			return;
		}

		// Clock sync, on the same schedule the Linux server uses: every 10 ms
		// until the 100-sample window is full, every 100 ms after that.
		const auto now = clock::now();
		try
		{
			if (clock_.should_sample(now))
				send_stream(wivrn::to_headset::timesync_query{.query = monotonic_ns()});

			pump_haptics();

			// Video last of the three: a frame is orders of magnitude more bytes
			// than a timesync query or a haptic pulse, and putting it in front of
			// them would delay the clock estimate the frame's own timestamps
			// depend on.
			update_stream_description();
			pump_video();
		}
		catch (const std::exception & e)
		{
			log_line("session ended: %s", e.what());
			return;
		}

		if (now >= next_report_)
		{
			next_report_ = now + 5s;
			const ClockOffset offset = clock_.get_offset();
			log_line("session: %llu tracking, %llu input packets, %llu other dropped; clock %s; "
			         "poses rejected %llu, frozen %llu",
			         static_cast<unsigned long long>(tracking_packets_),
			         static_cast<unsigned long long>(input_packets_),
			         static_cast<unsigned long long>(discarded_packets_),
			         offset ? "locked" : "converging",
			         static_cast<unsigned long long>(sanitizers_[0].rejected() +
			                                         sanitizers_[1].rejected() +
			                                         sanitizers_[2].rejected()),
			         static_cast<unsigned long long>(sanitizers_[0].frozen() +
			                                         sanitizers_[1].frozen() +
			                                         sanitizers_[2].frozen()));

			log_line("video: %llu eye-frames sent in %llu shards (%llu kB), %llu send errors, "
			         "%llu IDRs requested, %llu frames dropped in the queue",
			         static_cast<unsigned long long>(video_frames_sent_),
			         static_cast<unsigned long long>(video_shards_sent_),
			         static_cast<unsigned long long>(video_bytes_sent_ / 1024),
			         static_cast<unsigned long long>(video_send_errors_),
			         static_cast<unsigned long long>(idr_requests_),
			         static_cast<unsigned long long>(video_.frames_dropped_in_queue()));
		}
	}
}

} // namespace wivrnnx::helper
