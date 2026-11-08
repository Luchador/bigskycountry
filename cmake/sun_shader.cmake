# Compile the editable HLSL for SDL's Direct3D 12 and Vulkan GPU renderers.
# The resulting headers are generated in the build tree and linked into both executables.
if(NOT WIN32 AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    return()
endif()

set(BIGSKY_DXC "" CACHE FILEPATH "Optional host DXC executable with SPIR-V support")
set(dxc "${BIGSKY_DXC}")
if(NOT dxc)
    if(CMAKE_HOST_WIN32)
        set(dxc_archive "dxc_2025_07_14.zip")
        set(dxc_hash "9ad895a6b039e3a8f8c22a1009f866800b840a74b50db9218d13319e215ea8a4")
        if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "^(ARM64|arm64|aarch64)$")
            set(dxc_relative_path "bin/arm64/dxc.exe")
        else()
            set(dxc_relative_path "bin/x64/dxc.exe")
        endif()
    elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
        set(dxc_archive "linux_dxc_2025_07_14.x86_64.tar.gz")
        set(dxc_hash "f2213da1fc99dc8778c8823078e16ba97c7f80f86a1d4520ab1adf4b462bc48c")
        set(dxc_relative_path "bin/dxc")
    else()
        find_program(host_dxc NAMES dxc)
        if(NOT host_dxc)
            message(STATUS "No host DXC found; using SDL additive blending for the sun")
            return()
        endif()
        set(dxc "${host_dxc}")
    endif()

    if(dxc_archive)
        FetchContent_Declare(bigsky_dxc
            URL "https://github.com/microsoft/DirectXShaderCompiler/releases/download/v1.8.2505.1/${dxc_archive}"
            URL_HASH "SHA256=${dxc_hash}"
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
        FetchContent_MakeAvailable(bigsky_dxc)
        set(dxc "${bigsky_dxc_SOURCE_DIR}/${dxc_relative_path}")
    endif()
endif()

set(shader "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sun.frag.hlsl")
set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/shaders")
set(dxil "${output_dir}/sun_dxil.hpp")
set(spirv "${output_dir}/sun_spirv.hpp")
add_custom_command(
    OUTPUT "${dxil}" "${spirv}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${output_dir}"
    COMMAND "${dxc}" -T ps_6_0 -E main -WX -O3 -Qstrip_debug -Qstrip_reflect -Fh "${dxil}" -Vn sunDxil "${shader}"
    COMMAND "${dxc}" -spirv -T ps_6_0 -E main -WX -O3 -Fh "${spirv}" -Vn sunSpirv "${shader}"
    DEPENDS "${shader}" "${dxc}"
    COMMENT "Compiling the additive sun shader (DXIL and SPIR-V)"
    VERBATIM
)
target_sources(BigSkyApplication PRIVATE "${dxil}" "${spirv}")
target_include_directories(BigSkyApplication PRIVATE "${output_dir}")
