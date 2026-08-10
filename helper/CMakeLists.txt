# wivrnnx-helper
#
# The WiVRn server endpoint for the Windows port. It plays the pipe-server role
# of the frozen ipc/wivrnnx_ipc.h contract towards driver_wivrnnx.dll, and the
# WiVRn server role towards an unmodified WiVRn NX client APK: mDNS announce,
# pairing, handshake, then tracking in and haptics out.
#
# Without WIVRNNX_LINUX_REPO there is no network stack to link, so only the
# Phase 0 half is built: --fake works, the default mode is unavailable. That
# keeps this tree configurable on a machine with no wivrn-nx checkout.

set(WIVRNNX_HELPER_SOURCES
    src/main.cpp
    src/log.cpp
    src/bridge.cpp
    src/fake_tracker.cpp
    src/pipe_server.cpp
    src/video_bridge.cpp
    src/video_intake.cpp
    src/encoder/amf_loader.cpp
    src/encoder/amf_encoder.cpp
    src/encoder/amf_probe.cpp
    src/encoder/amf_video_encoder.cpp
    src/encoder/d3d11_stage.cpp
)

# ---------------------------------------------------------------------------
# AMD Media Framework
#
# Headers only, and never linked: amfrt64.dll ships with the Radeon driver, has
# no import library in the public SDK, and does not exist at all on a machine
# without one. src/encoder/amf_loader.cpp LoadLibrary()s it and takes the two C
# exports; everything past that is vtables out of these headers.
#
# SYSTEM, because they are vendored upstream code whose warnings are not ours to
# fix - though as of the vendored revision they are in fact -Wall -Wextra clean
# under llvm-mingw.
# ---------------------------------------------------------------------------
add_library(amf-headers INTERFACE)
target_include_directories(amf-headers SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/amf/include)

# ---------------------------------------------------------------------------
# OpenXR headers
#
# common/wivrn_packets.h is written against <openxr/openxr.h>. helper/net does
# the same search for its self test, but it is add_subdirectory()'d after this
# one, so its cache variable is not set yet and the search is repeated here
# rather than depending on directory order.
# ---------------------------------------------------------------------------
set(WIVRNNX_OPENXR_INCLUDE_DIR "" CACHE PATH "Directory containing openxr/openxr.h")

if(WIVRNNX_LINUX_REPO AND NOT WIVRNNX_OPENXR_INCLUDE_DIR)
    foreach(candidate /usr/include /usr/local/include)
        if(EXISTS ${candidate}/openxr/openxr.h)
            file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/openxr-include)
            file(CREATE_LINK ${candidate}/openxr
                 ${CMAKE_CURRENT_BINARY_DIR}/openxr-include/openxr SYMBOLIC)
            set(WIVRNNX_OPENXR_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/openxr-include)
            break()
        endif()
    endforeach()
endif()

set(WIVRNNX_HELPER_FULL OFF)
if(WIVRNNX_LINUX_REPO AND WIVRNNX_OPENXR_INCLUDE_DIR)
    set(WIVRNNX_HELPER_FULL ON)
elseif(WIVRNNX_LINUX_REPO)
    message(WARNING
        "No openxr/openxr.h found: building wivrnnx-helper without the WiVRn "
        "server (only --fake will work). Set WIVRNNX_OPENXR_INCLUDE_DIR.")
endif()

if(WIVRNNX_HELPER_FULL)
    list(APPEND WIVRNNX_HELPER_SOURCES
        src/wivrn/clock_offset.cpp
        src/wivrn/mdns_publisher.cpp
        src/wivrn/server.cpp
        src/wivrn/session.cpp
        src/wivrn/store.cpp
        src/wivrn/video_out.cpp
    )
endif()

add_executable(wivrnnx-helper ${WIVRNNX_HELPER_SOURCES})

# winmm: timeBeginPeriod(1), so the pose cadence is not quantised to the
#        default ~15.6 ms timer granularity.
# d3d11/dxgi: the helper's own device and the shared staging textures.
target_link_libraries(wivrnnx-helper PRIVATE wivrnnx-ipc amf-headers winmm d3d11 dxgi)

target_include_directories(wivrnnx-helper PRIVATE src)

if(WIVRNNX_HELPER_FULL)
    # wivrn-common-net is defined in helper/net, which the top level adds after
    # this directory; CMake resolves the target name at generate time.
    #
    # It brings ${WIVRNNX_LINUX_REPO}/common (the packets and sockets),
    # ${WIVRNNX_LINUX_REPO}/external (magic_enum.hpp and, for the announce,
    # mdns.h) and helper/net/platform (the POSIX shadows) with it, PUBLIC.
    #
    # iphlpapi: GetAdaptersAddresses, to open one mDNS socket per interface.
    # bcrypt:   BCryptGenRandom, for the pairing PIN and the DNS-SD cookie.
    target_link_libraries(wivrnnx-helper PRIVATE wivrn-common-net iphlpapi bcrypt ws2_32)

    # common/ and external/ are upstream code compiled unmodified; their
    # warnings cannot be fixed from here and would bury ours. Naming the same
    # directories as SYSTEM on this target turns them into -isystem, which does
    # not disturb the shadow headers in helper/net/platform: those stay a plain
    # -I, and the compiler always searches -I before -isystem.
    target_include_directories(wivrnnx-helper SYSTEM PRIVATE
        ${WIVRNNX_OPENXR_INCLUDE_DIR}
        ${WIVRNNX_LINUX_REPO}/common
        ${WIVRNNX_LINUX_REPO}/external)

    if(NOT MSVC)
        # mdns.h asks for <Winsock2.h> and <Ws2tcpip.h>; mingw-w64 ships them in
        # lower case. See src/mdns_compat/Winsock2.h.
        target_include_directories(wivrnnx-helper PRIVATE src/mdns_compat)
    endif()

    target_compile_definitions(wivrnnx-helper PRIVATE WIVRNNX_HAVE_WIVRN)

    # The "version" TXT record. The Linux server publishes wivrn::display_version(),
    # which is a git describe of the checkout it was built from
    # (common/version.cpp.in); do the same for the checkout this links against.
    find_package(Git QUIET)
    set(WIVRNNX_DISPLAY_VERSION "nx-windows")
    if(GIT_FOUND)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} describe --tags --always
            WORKING_DIRECTORY ${WIVRNNX_LINUX_REPO}
            OUTPUT_VARIABLE WIVRNNX_GIT_DESCRIBE
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
            RESULT_VARIABLE WIVRNNX_GIT_RESULT)
        if(WIVRNNX_GIT_RESULT EQUAL 0 AND WIVRNNX_GIT_DESCRIBE)
            set(WIVRNNX_DISPLAY_VERSION "${WIVRNNX_GIT_DESCRIBE}")
        endif()
    endif()
    message(STATUS "wivrnnx-helper: announcing version \"${WIVRNNX_DISPLAY_VERSION}\"")
    target_compile_definitions(wivrnnx-helper PRIVATE
        WIVRNNX_DISPLAY_VERSION="${WIVRNNX_DISPLAY_VERSION}")
else()
    message(STATUS "wivrnnx-helper: WiVRn server disabled, only --fake is available")
endif()

target_compile_features(wivrnnx-helper PRIVATE cxx_std_20)

target_compile_definitions(wivrnnx-helper PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    _USE_MATH_DEFINES
)

# Console subsystem: this is a foreground diagnostic tool.
set_target_properties(wivrnnx-helper PROPERTIES WIN32_EXECUTABLE OFF)

if(MSVC)
    target_compile_options(wivrnnx-helper PRIVATE /W4 /permissive-)
else()
    target_compile_options(wivrnnx-helper PRIVATE -Wall -Wextra)
    # llvm-mingw links libc++/libunwind dynamically by default; keep the exe
    # self-contained so it runs without toolchain DLLs beside it.
    target_link_options(wivrnnx-helper PRIVATE -static -static-libgcc -static-libstdc++)
endif()
