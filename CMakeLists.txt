cmake_minimum_required(VERSION 3.24)

project(wivrn-nx-windows LANGUAGES CXX)

if(NOT WIN32)
    message(FATAL_ERROR "This tree builds Windows binaries only. Cross-compile with "
        "--preset mingw-check or build natively with --preset vs2022.")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared, header-only targets
add_library(wivrnnx-ipc INTERFACE)
target_include_directories(wivrnnx-ipc INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/ipc)

add_library(openvr-headers INTERFACE)
target_include_directories(openvr-headers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/external/openvr)

# Path to the Linux wivrn-nx checkout, for common/ reuse from Phase 1 on.
# Phase 0 does not use it.
set(WIVRNNX_LINUX_REPO "" CACHE PATH "Path to the wivrn-nx (Linux) repo for common/")

add_subdirectory(shim)
add_subdirectory(helper)

# Phase 1+: network stack ported from the Linux repo's common/.
if(WIVRNNX_LINUX_REPO)
    add_subdirectory(helper/net)
endif()
