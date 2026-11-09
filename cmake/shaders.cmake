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
            message(STATUS "No host DXC found; using the software world depth buffer")
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

set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/shaders")
function(bigsky_shader filename profile entry stem dxil_name spirv_name)
    set(source "${CMAKE_CURRENT_SOURCE_DIR}/shaders/${filename}")
    set(dxil "${output_dir}/${stem}_dxil.hpp")
    set(spirv "${output_dir}/${stem}_spirv.hpp")
    add_custom_command(
        OUTPUT "${dxil}" "${spirv}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${output_dir}"
        COMMAND "${dxc}" -T "${profile}" -E "${entry}" -WX -O3 -Qstrip_debug -Qstrip_reflect -Fh "${dxil}" -Vn "${dxil_name}" "${source}"
        COMMAND "${dxc}" -spirv -T "${profile}" -E "${entry}" -WX -O3 -Fh "${spirv}" -Vn "${spirv_name}" "${source}"
        DEPENDS "${source}" "${dxc}"
        COMMENT "Compiling ${filename} (${entry}, DXIL and SPIR-V)"
        VERBATIM
    )
    target_sources(BigSkyApplication PRIVATE "${dxil}" "${spirv}")
endfunction()

bigsky_shader(sun.frag.hlsl ps_6_0 main sun sunDxil sunSpirv)
bigsky_shader(world.hlsl vs_6_0 VSMain world_vert world_vert_dxil world_vert_spirv)
bigsky_shader(world.hlsl ps_6_0 PSMain world_frag world_frag_dxil world_frag_spirv)
target_include_directories(BigSkyApplication PRIVATE "${output_dir}")
target_compile_definitions(BigSkyApplication PRIVATE BIGSKY_WORLD_GPU)
