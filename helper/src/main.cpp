// wivrnnx-helper - the WiVRn server endpoint for the Windows port.
//
// Three things run side by side:
//
//   * an mDNS responder, so an unmodified WiVRn NX client APK finds this
//     machine in its server list,
//   * the WiVRn server itself: TCP accept, the pairing/crypto handshake, then
//     tracking and input packets off the headset,
//   * the named pipe the SteamVR driver shim connects to (ipc/wivrnnx_ipc.h).
//
// The first two write into a Bridge; the pipe reads out of it. When no headset
// is connected the pipe simply carries no poses, which the shim handles with
// its own static-pose fallback.
//
// --fake replaces the WiVRn half with the Phase 0 synthetic tracker, which is
// the SteamVR smoke-test mode: no network, no headset, just a slow orbiting HMD.

#ifdef WIVRNNX_HAVE_WIVRN
// winsock2.h has to be parsed before anything reaches windows.h, and this tree
// inherits POSIX shadow headers from wivrn-common-net that answer <fcntl.h> and
// friends with it. Including it first, by name, makes that ordering explicit.
#include "win_net.h"
#endif

#include <windows.h>

#include <mmsystem.h>

#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <optional>
#include <string>
#include <memory>
#include <thread>
#include <vector>

#include "bridge.h"
#include "encoder/amf_video_encoder.h"
#include "fake_tracker.h"
#include "log.h"
#include "pipe_server.h"
#include "video_bridge.h"
#include "video_intake.h"
#include "win_handle.h"

#ifdef WIVRNNX_HAVE_WIVRN
#include "protocol_version.h"

#include "wivrn/mdns_publisher.h"
#include "wivrn/server.h"
#include "wivrn/session.h"
#include "wivrn/store.h"
#endif

#ifndef WIVRNNX_DISPLAY_VERSION
#define WIVRNNX_DISPLAY_VERSION "unknown"
#endif

namespace
{

// Signalled by the console control handler, which runs on its own thread.
HANDLE g_shutdown_event = nullptr;
std::atomic<bool> * g_stop_flag = nullptr;

BOOL WINAPI console_ctrl_handler(DWORD ctrl_type)
{
	switch (ctrl_type)
	{
		case CTRL_C_EVENT:
		case CTRL_BREAK_EVENT:
		case CTRL_CLOSE_EVENT:
		case CTRL_LOGOFF_EVENT:
		case CTRL_SHUTDOWN_EVENT:
			wivrnnx::helper::log_line("shutdown requested, stopping...");
			if (g_stop_flag != nullptr)
				g_stop_flag->store(true);
			if (g_shutdown_event != nullptr)
				SetEvent(g_shutdown_event);
			return TRUE;
		default:
			return FALSE;
	}
}

#ifndef WIVRNNX_HAVE_WIVRN
// Built without a wivrn-nx checkout: the WiVRn half does not exist, so the
// options that drive it are parsed and reported as unavailable rather than
// silently ignored.
namespace wivrnnx::helper
{
enum class EncryptionState
{
	disabled,
	enabled,
	pairing,
};
struct ServerOptions
{
	int port = 9757;
	EncryptionState encryption = EncryptionState::pairing;
	std::string pin;
	bool tcp_only = false;
};
} // namespace wivrnnx::helper
#endif

struct Options
{
	bool fake = false;
	bool mdns = true;
	std::string instance_name;
	uint32_t bitrate_mbps = 50;
	int codec = 0; // 0 auto, 1 h264, 2 h265
	wivrnnx::helper::ServerOptions server;
};

void usage()
{
	std::printf(
	        "wivrnnx-helper - WiVRn NX server endpoint for the SteamVR driver shim\n"
	        "\n"
	        "  --fake             Phase 0 smoke-test mode: no network, synthetic HMD poses\n"
	        "  --port N           TCP/UDP port to listen on (default %d)\n"
	        "  --pin NNNNNN       pairing PIN to require (default: a random one, printed)\n"
	        "  --no-pairing       accept only already-paired headsets\n"
	        "  --no-encryption    disable encryption and authentication entirely\n"
	        "  --tcp-only         no UDP stream socket, everything on the control connection\n"
	        "  --no-mdns          do not announce the service; the headset must connect by address\n"
	        "  --codec C          force the video codec: h264 or h265 (default: prefer h265)\n"
	        "  --bitrate N        video bitrate in Mbit/s (default 50)\n"
	        "  --name NAME        service instance name (default: this computer's name)\n"
	        "  --help             this text\n",
	        wivrnnx::helper::ServerOptions{}.port);
}

// Returns false if the command line was rejected; `ok` distinguishes --help.
bool parse_args(int argc, char ** argv, Options & options, bool & ok)
{
	ok = true;

	for (int i = 1; i < argc; ++i)
	{
		const std::string arg = argv[i];
		auto value = [&](const char * what) -> const char * {
			if (i + 1 >= argc)
			{
				std::printf("%s needs a value\n", what);
				ok = false;
				return nullptr;
			}
			return argv[++i];
		};

		if (arg == "--fake")
			options.fake = true;
		else if (arg == "--no-mdns")
			options.mdns = false;
		else if (arg == "--tcp-only")
			options.server.tcp_only = true;
		else if (arg == "--no-pairing")
			options.server.encryption = wivrnnx::helper::EncryptionState::enabled;
		else if (arg == "--no-encryption")
			options.server.encryption = wivrnnx::helper::EncryptionState::disabled;
		else if (arg == "--port")
		{
			const char * v = value("--port");
			if (v == nullptr)
				return false;
			options.server.port = std::atoi(v);
			if (options.server.port <= 0 || options.server.port > 65535)
			{
				std::printf("--port must be between 1 and 65535\n");
				ok = false;
				return false;
			}
		}
		else if (arg == "--pin")
		{
			const char * v = value("--pin");
			if (v == nullptr)
				return false;
			options.server.pin = v;
		}
		else if (arg == "--name")
		{
			const char * v = value("--name");
			if (v == nullptr)
				return false;
			options.instance_name = v;
		}
		else if (arg == "--codec")
		{
			const char * v = value("--codec");
			if (v == nullptr)
				return false;
			const std::string codec = v;
			if (codec == "h264" || codec == "avc")
				options.codec = 1;
			else if (codec == "h265" || codec == "hevc")
				options.codec = 2;
			else
			{
				std::printf("--codec must be h264 or h265\n");
				ok = false;
				return false;
			}
		}
		else if (arg == "--bitrate")
		{
			const char * v = value("--bitrate");
			if (v == nullptr)
				return false;
			const int mbps = std::atoi(v);
			if (mbps < 1 || mbps > 500)
			{
				std::printf("--bitrate must be between 1 and 500 (Mbit/s)\n");
				ok = false;
				return false;
			}
			options.bitrate_mbps = static_cast<uint32_t>(mbps);
		}
		else if (arg == "--help" || arg == "-h")
		{
			usage();
			ok = false;
			return false;
		}
		else
		{
			std::printf("unknown argument \"%s\"\n\n", arg.c_str());
			usage();
			ok = false;
			return false;
		}
	}

	return ok;
}

} // namespace

int main(int argc, char ** argv)
{
	using namespace wivrnnx::helper;

	Options options;
	bool ok = true;
	if (!parse_args(argc, argv, options, ok))
		return ok ? 0 : 1;

#ifdef WIVRNNX_HAVE_WIVRN
	if (options.server.pin.empty())
		options.server.pin = random_pin();
	if (options.instance_name.empty())
		options.instance_name = default_instance_name();
#else
	if (!options.fake)
	{
		std::printf("This build has no WiVRn server (configured without WIVRNNX_LINUX_REPO "
		            "or without OpenXR headers). Only --fake is available.\n");
		return 1;
	}
#endif

	log_line("wivrnnx-helper starting (%s mode, IPC protocol v%u)",
	         options.fake ? "fake" : "WiVRn server",
	         wivrnnx::ipc::kProtocolVersion);

	UniqueHandle shutdown_event(CreateEventW(nullptr, TRUE, FALSE, nullptr));
	if (!shutdown_event)
	{
		log_win32(GetLastError(), "CreateEventW(shutdown) failed");
		return 1;
	}

	std::atomic<bool> stop{false};
	g_shutdown_event = shutdown_event.get();
	g_stop_flag = &stop;

	if (!SetConsoleCtrlHandler(console_ctrl_handler, TRUE))
	{
		log_win32(GetLastError(), "SetConsoleCtrlHandler failed");
		return 1;
	}

	// Default timer granularity is ~15.6 ms, which would quantise both the
	// synthetic pose tick and the pipe's bridge polling down to ~64 Hz. Since
	// Windows 10 2004 this request only affects this process.
	const bool timer_raised = timeBeginPeriod(1) == TIMERR_NOERROR;
	if (!timer_raised)
		log_line("warning: timeBeginPeriod(1) failed, pose cadence may be coarse");

	// The shim must have an HmdConfig before it may register its HMD, and it
	// may connect long before any headset does. Seed the bridge with the
	// Phase 0 Pico-4-shaped config; the real one replaces it, and is re-sent,
	// as soon as a headset reports its view configuration.
	Bridge bridge(make_hmd_config());

	// The video seam and the thread behind it. The encoder is constructed but
	// not opened: nothing touches D3D11 or amfrt64.dll until a headset is
	// connected and the shim has sent a staging ring, which is what keeps
	// --fake (and a machine with no Radeon) entirely out of the GPU.
	VideoBridge video_bridge;
	video_bridge.set_prefs(options.bitrate_mbps * 1'000'000u, options.codec);
	VideoIntake video_intake(video_bridge, std::make_unique<AmfVideoEncoder>());
	video_intake.start();

	std::thread mdns_thread;
	std::thread wivrn_thread;
	std::thread fake_thread;

#ifdef WIVRNNX_HAVE_WIVRN
	// Constructed only in the default mode: --fake is the offline smoke test and
	// has no business creating %APPDATA%\\wivrnnx just to mint a DNS-SD cookie.
	std::optional<MdnsPublisher> publisher;
	if (!options.fake)
		publisher.emplace(options.instance_name,
		                  options.instance_name,
		                  options.server.port,
		                  [&] {
			                  // Replicated from server/main.cpp:620-631. The
			                  // client rejects a server whose "protocol" does
			                  // not equal its own protocol_version rendered as
			                  // 16 lowercase hex digits
			                  // (client/scenes/lobby.cpp:205-221), and keys its
			                  // saved-server list on "cookie".
			                  char protocol_string[17];
			                  std::snprintf(protocol_string,
			                                sizeof(protocol_string),
			                                "%016" PRIx64,
			                                static_cast<uint64_t>(wivrn::protocol_version));

			                  return std::map<std::string, std::string>{
			                          {"protocol", protocol_string},
			                          {"version", WIVRNNX_DISPLAY_VERSION},
			                          {"cookie", server_cookie()},
			                  };
		                  }());
#endif

	if (options.fake)
	{
		log_line("--fake: streaming synthetic HMD poses at %.0f Hz, no network", kPoseRateHz);
		fake_thread = std::thread([&] { run_fake_tracker(bridge, shutdown_event.get()); });
	}
#ifdef WIVRNNX_HAVE_WIVRN
	else
	{
		switch (options.server.encryption)
		{
			case EncryptionState::pairing:
				log_line("pairing enabled, PIN for a new headset: %s", options.server.pin.c_str());
				break;
			case EncryptionState::enabled:
				log_line("pairing disabled, only already-paired headsets are accepted");
				break;
			case EncryptionState::disabled:
				log_line("WARNING: encryption and authentication are disabled");
				break;
		}

		const std::vector<HeadsetKey> keys = known_keys();
		log_line("%zu paired headset(s) known%s", keys.size(), keys.empty() ? "" : ":");
		for (const HeadsetKey & key: keys)
			log_line("  %s", key.name.c_str());

		if (options.mdns && publisher->start())
			mdns_thread = std::thread([&] { publisher->run(shutdown_event.get()); });
		else if (options.mdns)
			log_line("mDNS announce unavailable; connect the headset by address instead");

		wivrn_thread = std::thread([&] { run_wivrn_server(options.server, bridge, video_bridge, stop); });
	}
#endif

	int exit_code = 0;
	{
		PipeServer server(shutdown_event.get(), bridge, video_intake);
		if (server.create())
			server.run();
		else
			exit_code = 1;
	}

	// The pipe server only returns on shutdown or on a fatal create() failure;
	// in the latter case the other threads have not been told yet.
	stop.store(true);
	SetEvent(shutdown_event.get());

	// Before the WiVRn thread is joined: the intake's thread may be mid-encode
	// and the session's ~Session is what tells it to stop wanting frames.
	video_intake.stop();

	if (fake_thread.joinable())
		fake_thread.join();
	if (wivrn_thread.joinable())
		wivrn_thread.join();
	if (mdns_thread.joinable())
		mdns_thread.join();

	if (timer_raised)
		timeEndPeriod(1);

	SetConsoleCtrlHandler(console_ctrl_handler, FALSE);
	g_shutdown_event = nullptr;
	g_stop_flag = nullptr;

	log_line("wivrnnx-helper stopped");
	return exit_code;
}
