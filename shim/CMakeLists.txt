# driver_wivrnnx.dll — the thin OpenVR driver shim that lives inside vrserver.exe.

add_library(driver_wivrnnx SHARED
    src/breadcrumb.cpp
    src/controller_device.cpp
    src/direct_mode.cpp
    src/display_component.cpp
    src/driver_provider.cpp
    src/driverlog.cpp
    src/frame_staging.cpp
    src/hmd_device.cpp
    src/ipc_client.cpp
    src/vr_context.cpp
)

# d3d11/dxgi: the direct mode component allocates the shared textures the
# compositor renders into (src/direct_mode.cpp). Both are OS DLLs, so this adds
# no redistributable dependency.
target_link_libraries(driver_wivrnnx PRIVATE openvr-headers wivrnnx-ipc d3d11 dxgi)

set_target_properties(driver_wivrnnx PROPERTIES
    OUTPUT_NAME driver_wivrnnx
    PREFIX ""                      # mingw would otherwise emit libdriver_wivrnnx.dll
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_compile_definitions(driver_wivrnnx PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    UNICODE
    _UNICODE
)

if(MSVC)
    target_compile_options(driver_wivrnnx PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(driver_wivrnnx PRIVATE -Wall -Wextra)
    # Keep the DLL free of libc++/libunwind runtime dependencies so it can be
    # dropped next to SteamVR without shipping the toolchain runtime.
    target_link_options(driver_wivrnnx PRIVATE -static -static-libgcc -static-libstdc++)
endif()

# ---------------------------------------------------------------------------
# Registerable SteamVR driver tree: build/<preset>/driver/wivrnnx/
#
#   driver.vrdrivermanifest
#   bin/win64/driver_wivrnnx.dll
#   resources/driver.vrresources
#   resources/settings/default.vrsettings
#   resources/input/wivrnnx_controller_profile.json
#
# Register it on Windows with:
#   vrpathreg.exe adddriver <path-to>/driver/wivrnnx
# ---------------------------------------------------------------------------

set(WIVRNNX_DRIVER_TREE "${CMAKE_BINARY_DIR}/driver/wivrnnx")

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/driver_manifest/driver.vrdrivermanifest"
    "${WIVRNNX_DRIVER_TREE}/driver.vrdrivermanifest"
    COPYONLY
)
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/driver_manifest/resources/driver.vrresources"
    "${WIVRNNX_DRIVER_TREE}/resources/driver.vrresources"
    COPYONLY
)
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/driver_manifest/resources/settings/default.vrsettings"
    "${WIVRNNX_DRIVER_TREE}/resources/settings/default.vrsettings"
    COPYONLY
)
# Referenced by Prop_InputProfilePath_String as
# "{wivrnnx}/input/wivrnnx_controller_profile.json".
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/driver_manifest/resources/input/wivrnnx_controller_profile.json"
    "${WIVRNNX_DRIVER_TREE}/resources/input/wivrnnx_controller_profile.json"
    COPYONLY
)

add_custom_command(TARGET driver_wivrnnx POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${WIVRNNX_DRIVER_TREE}/bin/win64"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "$<TARGET_FILE:driver_wivrnnx>"
            "${WIVRNNX_DRIVER_TREE}/bin/win64/driver_wivrnnx.dll"
    COMMENT "Assembling SteamVR driver tree at ${WIVRNNX_DRIVER_TREE}"
    VERBATIM
)
