#!/bin/sh
# The helper's video-path tests.
#
# Two of the three run on the build machine; the third needs Wine.
#
#   shard_test      native Linux, g++. The shard packetizer against the real
#                   wire types, the upstream slicing loop and the client's own
#                   shard_set, plus the parity shards against upstream's own
#                   transcription of the group builder and the pacer's gap
#                   distribution. No Windows, no GPU, no socket.
#   bitrate_test    native Linux, g++. The ported bitrate controller and the IDR
#                   floor, driven with synthetic feedback traces on a virtual
#                   clock: late frames pull the bitrate down, clean ones let it
#                   climb back to the ceiling, the radio trend steps in ahead of
#                   both, and a headset that loses every frame gets two key
#                   frames a second instead of ninety.
#   amf_stub.dll +  cross compiled with llvm-mingw and run under Wine. A fake
#   amf_test.exe    amfrt64.dll that implements just enough of the AMF ABI to
#                   drive the loader, the component creation, the whole property
#                   set and the submit/poll loop, and to assert on what was set.
#   mock_shim.exe   cross compiled, run under Wine against a live
#                   wivrnnx-helper.exe --fake. Speaks protocol v3 on the pipe:
#                   handshake, StagingConfig, FrameReady, and checks the
#                   FrameDone that comes back.
#   fake_client     native Linux, g++, run against a live wivrnnx-helper.exe
#                   under Wine over loopback: the real handshake, the real
#                   shards, real feedback. Twice, once over UDP and once
#                   TCP-only. This is the one that proves the *wiring* - that
#                   the bitrate really moves, that a frame really leaves in
#                   micro-bursts, that parity appears only where it repairs
#                   something and that the IDR floor really holds.
#
# Usage: sh helper/src/tests/run_tests.sh [shard|bitrate|amf|shim|loopback|all]
set -e

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
linux_repo=${WIVRNNX_LINUX_REPO:-/run/media/nerdrx/Lex/claude/wivrn-nx}
mingw=${LLVM_MINGW_ROOT:-/run/media/nerdrx/Lex/claude/tools/llvm-mingw}
out=${WIVRNNX_TEST_OUT:-$root/build/tests}

mkdir -p "$out"
what=${1:-all}
# `all` runs everything and reports at the end rather than stopping at the first
# failing suite: a red loopback run with a green shim run is worth knowing about.
suite_failures=0

build_shard() {
	echo "=== building shard_test (native)"
	g++ -std=c++23 -O1 -g -Wall -Wextra \
		-I "$linux_repo/common" \
		-I "$linux_repo/client" \
		-I "$linux_repo/external" \
		-I "$root/external/boost-pfr/include" \
		-o "$out/shard_test" \
		"$here/shard_test.cpp" \
		"$here/../wivrn/video_out.cpp" \
		"$linux_repo/common/smp.cpp" \
		-lcrypto
}

build_bitrate() {
	echo "=== building bitrate_test (native)"
	g++ -std=c++23 -O1 -g -Wall -Wextra \
		-I "$linux_repo/common" \
		-I "$linux_repo/external" \
		-I "$root/external/boost-pfr/include" \
		-o "$out/bitrate_test" \
		"$here/bitrate_test.cpp" \
		"$here/../wivrn/bitrate_controller.cpp" \
		"$linux_repo/common/smp.cpp" \
		-lcrypto
}

build_client() {
	echo "=== building fake_client (native)"
	g++ -std=c++23 -O1 -g -Wall -Wextra \
		-I "$linux_repo/common" \
		-I "$linux_repo/client" \
		-I "$linux_repo/external" \
		-I "$root/external/boost-pfr/include" \
		-o "$out/fake_client" \
		"$here/fake_client.cpp" \
		"$linux_repo/common/wivrn_sockets.cpp" \
		"$linux_repo/common/crypto.cpp" \
		"$linux_repo/common/smp.cpp" \
		-lcrypto
}

build_win() {
	echo "=== building $1 (llvm-mingw)"
	shift
	"$mingw/bin/x86_64-w64-mingw32-clang++" "$@"
}

case "$what" in
shard | all)
	build_shard
	echo "=== running shard_test"
	set +e
	"$out/shard_test" || suite_failures=$((suite_failures + 1))
	set -e
	;;
esac

case "$what" in
bitrate | all)
	build_bitrate
	echo "=== running bitrate_test"
	set +e
	"$out/bitrate_test" || suite_failures=$((suite_failures + 1))
	set -e
	;;
esac

case "$what" in
amf | all)
	build_win amf_stub.dll \
		-std=c++20 -O1 -g -Wall -Wextra -shared \
		-I "$root/external/amf/include" \
		-o "$out/amfrt64_stub.dll" "$here/amf_stub.cpp" -lole32 \
		-static -static-libgcc -static-libstdc++
	build_win amf_test.exe \
		-std=c++20 -O1 -g -Wall -Wextra \
		-I "$root/external/amf/include" -I "$root/helper/src" -I "$root/ipc" \
		-DWIN32_LEAN_AND_MEAN -DNOMINMAX \
		-o "$out/amf_test.exe" \
		"$here/amf_test.cpp" \
		"$here/../encoder/amf_loader.cpp" \
		"$here/../encoder/amf_encoder.cpp" \
		"$here/../log.cpp" \
		-lole32 -static -static-libgcc -static-libstdc++
	echo "=== running amf_test under Wine"
	set +e
	( cd "$out" && WIVRNNX_AMF_DLL="$out/amfrt64_stub.dll" wine "$out/amf_test.exe" ) ||
		suite_failures=$((suite_failures + 1))
	set -e
	;;
esac

case "$what" in
loopback | all)
	build_client
	helper_exe=$root/build/mingw-check/helper/wivrnnx-helper.exe
	if [ ! -x "$helper_exe" ]; then
		echo "no helper at $helper_exe; run tools/compile_check.sh first"
		exit 1
	fi

	# Two runs against a live helper under Wine: one over UDP, where parity is
	# worth something, and one TCP-only, where it is not. Both drive the whole
	# transport - the controller, the pacer, the parity builder and the IDR floor -
	# with a synthetic video stream, so no GPU and no SteamVR are involved.
	port=${LOOPBACK_PORT:-19757}
	seconds=${LOOPBACK_SECONDS:-24}

	for mode in udp tcp; do
		extra=""
		# --drop-permille throws 2% of the data shards away *inside the client*,
		# after they came off the socket. Loopback never loses anything, so
		# without it the helper's parity shards are received, counted and
		# discarded and fec::reconstruct - the thing a real Wi-Fi link runs
		# several times a second - is never executed at all.
		client_extra="--expect-parity --drop-permille 20"
		if [ "$mode" = tcp ]; then
			extra="--tcp-only"
			client_extra=""
		fi

		echo "=== starting the helper under Wine ($mode)"
		# --no-mdns: nothing may appear on the network for a test.
		# --no-encryption: the fake client has no pairing UI to type a PIN into.
		wine "$helper_exe" --no-mdns --no-encryption --synthetic-video \
			--port "$port" --bitrate 50 $extra \
			>"$out/helper-$mode.log" 2>&1 &
		helper_pid=$!
		sleep 3

		echo "=== running fake_client ($mode)"
		set +e
		"$out/fake_client" --port "$port" --seconds "$seconds" $extra $client_extra
		client_rc=$?
		set -e
		[ $client_rc -eq 0 ] || suite_failures=$((suite_failures + 1))

		kill "$helper_pid" 2>/dev/null || true
		pkill -f "wivrnnx-helper.exe --no-mdns" 2>/dev/null || true
		sleep 1

		echo "=== helper log ($mode): transport lines"
		grep -E "video transport|automatic bitrate|bitrate [0-9]|IDR|video:" "$out/helper-$mode.log" | head -30
		idrs=$(grep -c "IDR requested" "$out/helper-$mode.log" || true)
		echo "--- $idrs IDR requests over ${seconds}s (an undamped tracker asked for 534 in minutes)"
	done
	;;
esac

case "$what" in
shim | all)
	build_win mock_shim.exe \
		-std=c++20 -O1 -g -Wall -Wextra \
		-I "$root/ipc" -DWIN32_LEAN_AND_MEAN -DNOMINMAX \
		-o "$out/mock_shim.exe" "$here/mock_shim.cpp" \
		-static -static-libgcc -static-libstdc++
	helper_exe=$root/build/mingw-check/helper/wivrnnx-helper.exe
	if [ ! -x "$helper_exe" ]; then
		echo "no helper at $helper_exe; run tools/compile_check.sh first"
		exit 1
	fi
	echo "=== starting the helper (--fake) under Wine"
	wine "$helper_exe" --fake >"$out/helper.log" 2>&1 &
	helper_pid=$!
	# The pipe exists a moment after start; the mock retries for ten seconds
	# anyway, so this is only to keep the log readable.
	sleep 2
	echo "=== running mock_shim under Wine"
	set +e
	wine "$out/mock_shim.exe" "${MOCK_SHIM_FRAMES:-60}"
	rc=$?
	set -e
	[ $rc -eq 0 ] || suite_failures=$((suite_failures + 1))
	kill "$helper_pid" 2>/dev/null || true
	# The wine launcher is not the Windows process; kill that by name rather
	# than reaching for wineserver -k, which would take down anything else the
	# user has running in the prefix.
	pkill -f "wivrnnx-helper.exe --fake" 2>/dev/null || true
	sleep 1
	echo "=== helper log"
	sed -n "1,40p" "$out/helper.log"
	;;
esac

if [ "$suite_failures" -ne 0 ]; then
	echo
	echo "=== $suite_failures test suite(s) failed"
	exit 1
fi
