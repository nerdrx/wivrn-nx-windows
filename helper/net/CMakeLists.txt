# wivrn-common-net - the Linux tree's network/crypto core, built for Windows.
#
# The sources are compiled straight out of ${WIVRNNX_LINUX_REPO}/common with no
# copy and no patch: common/ is meant to stay genuinely shared and #ifdef-free,
# so the port is done by putting platform/ first on the include path, where a
# set of shadow headers answers common/'s POSIX includes with Winsock2. See
# platform/win_net.h.
#
# Nothing in this directory may modify the Linux checkout. Anything that cannot
# be expressed as a shadow header belongs in a report, not in common/.

set(WIVRN_COMMON ${WIVRNNX_LINUX_REPO}/common)

if(NOT EXISTS ${WIVRN_COMMON}/wivrn_sockets.cpp)
    message(FATAL_ERROR
        "WIVRNNX_LINUX_REPO=${WIVRNNX_LINUX_REPO} does not look like a wivrn-nx "
        "checkout (no common/wivrn_sockets.cpp)")
endif()

# ---------------------------------------------------------------------------
# OpenSSL
#
# Cross builds use the static libcrypto from tools/build_openssl_mingw.sh. The
# vs2022 preset overrides this with a vcpkg prefix.
# ---------------------------------------------------------------------------
set(WIVRNNX_OPENSSL_ROOT "/run/media/nerdrx/Lex/claude/tools/openssl-mingw"
    CACHE PATH "Prefix of an OpenSSL 3.x install with a static libcrypto for the target")

# NO_CMAKE_FIND_ROOT_PATH: the cross toolchain pins find_library to the
# llvm-mingw sysroot, and this prefix deliberately lives outside it.
find_library(WIVRNNX_LIBCRYPTO
    NAMES crypto libcrypto
    HINTS ${WIVRNNX_OPENSSL_ROOT}/lib ${WIVRNNX_OPENSSL_ROOT}/lib64
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

if(NOT WIVRNNX_LIBCRYPTO)
    message(FATAL_ERROR
        "No libcrypto under WIVRNNX_OPENSSL_ROOT=${WIVRNNX_OPENSSL_ROOT}.\n"
        "Cross builds: run tools/build_openssl_mingw.sh once.\n"
        "MSVC builds: point WIVRNNX_OPENSSL_ROOT at the vcpkg installed prefix.")
endif()

if(NOT EXISTS ${WIVRNNX_OPENSSL_ROOT}/include/openssl/evp.h)
    message(FATAL_ERROR "No openssl headers under ${WIVRNNX_OPENSSL_ROOT}/include")
endif()

add_library(wivrnnx-openssl INTERFACE)
target_include_directories(wivrnnx-openssl SYSTEM INTERFACE ${WIVRNNX_OPENSSL_ROOT}/include)
# libcrypto on Windows pulls the CryptoAPI entropy and cert stores in.
target_link_libraries(wivrnnx-openssl INTERFACE ${WIVRNNX_LIBCRYPTO} crypt32 bcrypt advapi32 user32 ws2_32)

# ---------------------------------------------------------------------------
# The library
# ---------------------------------------------------------------------------
add_library(wivrn-common-net STATIC
    ${WIVRN_COMMON}/wivrn_sockets.cpp
    ${WIVRN_COMMON}/crypto.cpp
    ${WIVRN_COMMON}/smp.cpp
    ${WIVRN_COMMON}/secrets.cpp
    platform/win_net.cpp
)

# Order matters: platform/ has to come before common/ and before the toolchain's
# own directories so that <sys/socket.h> and friends resolve to the shadows.
target_include_directories(wivrn-common-net PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/platform
    ${WIVRN_COMMON}
    ${WIVRNNX_LINUX_REPO}/external          # magic_enum.hpp
    ${CMAKE_SOURCE_DIR}/external/boost-pfr/include
)

target_link_libraries(wivrn-common-net PUBLIC wivrnnx-openssl ws2_32)

target_compile_features(wivrn-common-net PUBLIC cxx_std_20)

target_compile_definitions(wivrn-common-net PUBLIC
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    _WIN32_WINNT=0x0A00
    _CRT_SECURE_NO_WARNINGS      # common/ uses sprintf in its hexdump
)

if(MSVC)
    target_compile_options(wivrn-common-net PRIVATE /permissive-)
else()
    target_compile_options(wivrn-common-net PRIVATE -Wall -Wextra)

    # common/ is compiled unmodified, so its own warnings cannot be fixed from
    # here and would only bury the ones that come from platform/. Silence
    # exactly those three, on exactly those files.
    set_source_files_properties(
        ${WIVRN_COMMON}/wivrn_sockets.cpp
        ${WIVRN_COMMON}/crypto.cpp
        ${WIVRN_COMMON}/smp.cpp
        ${WIVRN_COMMON}/secrets.cpp
        TARGET_DIRECTORY wivrn-common-net
        PROPERTIES COMPILE_OPTIONS
        "-Wno-sign-compare;-Wno-unused-parameter;-Wno-missing-designated-field-initializers")
endif()

# ---------------------------------------------------------------------------
# Self test
#
# Proves three things that a compile alone does not: that the serialization
# templates instantiate for the real protocol variants outside of clang-on-Linux,
# that the shadow headers produce sockets that actually talk over loopback, and
# that the AES-CTR framing survives a round trip through Winsock.
#
# wivrn_packets.h needs the OpenXR types. Rather than putting a whole system
# include directory on a mingw include path, link just the openxr/ subtree into
# the build directory.
# ---------------------------------------------------------------------------
set(WIVRNNX_OPENXR_INCLUDE_DIR "" CACHE PATH "Directory containing openxr/openxr.h")

if(NOT WIVRNNX_OPENXR_INCLUDE_DIR)
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

if(WIVRNNX_OPENXR_INCLUDE_DIR)
    add_executable(wivrnnx-net-selftest selftest.cpp)
    target_link_libraries(wivrnnx-net-selftest PRIVATE wivrn-common-net)
    target_include_directories(wivrnnx-net-selftest SYSTEM PRIVATE ${WIVRNNX_OPENXR_INCLUDE_DIR})
    set_target_properties(wivrnnx-net-selftest PROPERTIES WIN32_EXECUTABLE OFF)

    if(NOT MSVC)
        # llvm-mingw links libc++/libunwind dynamically by default; keep the exe
        # runnable under wine and on a bare Windows box with no toolchain DLLs.
        target_link_options(wivrnnx-net-selftest PRIVATE -static -static-libgcc -static-libstdc++)
    endif()
else()
    message(WARNING
        "No openxr/openxr.h found: skipping wivrnnx-net-selftest. "
        "Set WIVRNNX_OPENXR_INCLUDE_DIR to a directory containing openxr/openxr.h.")
endif()
