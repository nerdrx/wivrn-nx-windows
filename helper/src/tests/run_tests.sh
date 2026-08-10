#!/bin/sh
# The helper's video-path tests.
#
# Two of the three run on the build machine; the third needs Wine.
#
#   shard_test      native Linux, g++. The shard packetizer against the real
#                   wire types, the upstream slicing loop and the client's own
#                   shard_set. No Windows, no GPU, no socket.
#   amf_stub.dll +  cross compiled with llvm-mingw and run under Wine. A fake
#   amf_test.exe    amfrt64.dll that implements just enough of the AMF ABI to
#                   drive the loader, the component creation, the whole property
#                   set and the submit/poll loop, and to assert on what was set.
#   mock_shim.exe   cross compiled, run under Wine against a live
#                   wivrnnx-helper.exe --fake. Speaks protocol v3 on the pipe:
#                   handshake, StagingConfig, FrameReady, and checks the
#                   FrameDone that comes back.
#
# Usage: sh helper/src/tests/run_tests.sh [shard|amf|shim|all]
set -e

here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
linux_repo=${WIVRNNX_LINUX_REPO:-/run/media/nerdrx/Lex/claude/wivrn-nx}
mingw=${LLVM_MINGW_ROOT:-/run/media/nerdrx/Lex/claude/tools/llvm-mingw}
out=${WIVRNNX_TEST_OUT:-$root/build/tests}

mkdir -p "$out"
what=${1:-all}

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

build_win() {
	echo "=== building $1 (llvm-mingw)"
	shift
	"$mingw/bin/x86_64-w64-mingw32-clang++" "$@"
}

case "$what" in
shard | all)
	build_shard
	echo "=== running shard_test"
	"$out/shard_test"
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
	( cd "$out" && WIVRNNX_AMF_DLL="$out/amfrt64_stub.dll" wine "$out/amf_test.exe" )
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
	kill "$helper_pid" 2>/dev/null || true
	# The wine launcher is not the Windows process; kill that by name rather
	# than reaching for wineserver -k, which would take down anything else the
	# user has running in the prefix.
	pkill -f "wivrnnx-helper.exe --fake" 2>/dev/null || true
	sleep 1
	echo "=== helper log"
	sed -n "1,40p" "$out/helper.log"
	exit $rc
	;;
esac
