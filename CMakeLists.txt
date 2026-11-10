cmake_minimum_required(VERSION 3.24)
project(BigSkyCountry LANGUAGES C CXX)

include(FetchContent)
set(SDL_SHARED OFF)
set(SDL_STATIC ON)
FetchContent_Declare(SDL3
    URL https://github.com/libsdl-org/SDL/releases/download/release-3.4.20/SDL3-3.4.20.tar.gz
    URL_HASH SHA256=f3bb06f55bed9557227a700af2ea62e49497ebdf58037732a731a5fb3ef8581f
)
FetchContent_MakeAvailable(SDL3)

# cgltf 1.15 (MIT); shared by model loading in the editor and game.
FetchContent_Declare(cgltf
    URL https://codeload.github.com/jkuhlmann/cgltf/tar.gz/360db1a95480fe102ae9c69b27c5d101167ff5ba
    URL_HASH SHA256=445d135cf793232ae6a585ca1404e4ff28d4f4dbca070689034fe780370ac84a
)
FetchContent_MakeAvailable(cgltf)

add_library(BigSkyApplication STATIC
    shared/bscobject.cpp
    shared/building.cpp
    shared/character.cpp
    shared/prop.cpp
    shared/vehicle.cpp
    shared/object_io.cpp
    shared/mesh.cpp
    shared/mesh_import.cpp
    shared/world_renderer.cpp
    shared/road.cpp
    shared/road_stripes.cpp
    shared/road_io.cpp
    shared/scene_hierarchy.cpp
    shared/world_document.cpp
    shared/road_render.cpp
    shared/world_renderer_gpu.cpp
    shared/world_renderer_software.cpp
    shared/sky_disc.cpp
    shared/moon.cpp
    shared/sky.cpp
    shared/solar_aureole.cpp
    shared/sun.cpp
    shared/sun_shader.cpp
    shared/application.cpp
    shared/free_camera.cpp
    shared/input.cpp
    shared/math.cpp
    shared/texture_loader.cpp
    shared/world_state.cpp
)
target_compile_features(BigSkyApplication PUBLIC cxx_std_20)
target_include_directories(BigSkyApplication PUBLIC shared PRIVATE "${cgltf_SOURCE_DIR}")
target_link_libraries(BigSkyApplication PUBLIC SDL3::SDL3-static)
include(cmake/shaders.cmake)

add_executable(BigSkyCountry WIN32 game/main.cpp)
target_link_libraries(BigSkyCountry PRIVATE BigSkyApplication)

# SDL_image 3.4.0 for editor thumbnails. Use its built-in decoders to avoid extra runtime DLLs.
set(BUILD_SHARED_LIBS OFF)
set(SDLIMAGE_VENDORED OFF)
set(SDLIMAGE_PNG_LIBPNG OFF)
set(SDLIMAGE_AVIF OFF)
set(SDLIMAGE_WEBP OFF)
set(SDLIMAGE_TIF OFF)
FetchContent_Declare(SDL3_image
    URL https://codeload.github.com/libsdl-org/SDL_image/tar.gz/ad58ecfc27a1bd09e510ceff8bbbdb1094806476
    URL_HASH SHA256=29fa60ea0432997c4ae61358a8e8fcb3638520d123a590fc9f42558967239cd2
)
FetchContent_MakeAvailable(SDL3_image)

add_executable(BigSkyCountryEditor WIN32
    editor/main.cpp
    editor/content_browser.cpp
    editor/texture_catalog.cpp
    editor/scene_outliner.cpp
    editor/road_editor.cpp
    editor/transform_inspector.cpp
    editor/road_overlay.cpp
    editor/transform_gizmo.cpp
    editor/transform_settings.cpp
    editor/viewport_picker.cpp
    editor/editor_menu.cpp
    editor/edit_history.cpp
)
if(WIN32)
    enable_language(RC)
    if(MSVC AND CMAKE_GENERATOR MATCHES "^Ninja" AND CMAKE_NINJA_CMCLDEPS_RC)
        get_filename_component(cmake_bin_dir "${CMAKE_COMMAND}" DIRECTORY)
        if(NOT EXISTS "${cmake_bin_dir}/cmcldeps.exe")
            # Some CMake distributions package the Ninja RC dependency scanner separately.
            # Compile the dialog directly and track its project header explicitly.
            set(CMAKE_NINJA_CMCLDEPS_RC OFF)
            set_property(SOURCE editor/panels.rc APPEND PROPERTY OBJECT_DEPENDS
                "${CMAKE_CURRENT_SOURCE_DIR}/editor/panel_ids.h")
        endif()
    endif()
    target_sources(BigSkyCountryEditor PRIVATE editor/windows_panels.cpp editor/windows_outliner.cpp editor/panels.rc)
    set_source_files_properties(editor/windows_panels.cpp editor/windows_outliner.cpp PROPERTIES COMPILE_DEFINITIONS "UNICODE;_UNICODE;NOMINMAX")
    target_include_directories(BigSkyCountryEditor PRIVATE editor)
    if(MSVC)
        target_sources(BigSkyCountryEditor PRIVATE editor/editor.manifest)
    else()
        target_compile_definitions(BigSkyCountryEditor PRIVATE BSC_EMBED_MANIFEST)
    endif()
    target_link_libraries(BigSkyCountryEditor PRIVATE user32 comctl32 gdi32)
else()
    target_sources(BigSkyCountryEditor PRIVATE editor/camera_toolbar.cpp editor/sdl_transform_panel.cpp editor/sdl_outliner.cpp)
endif()
target_link_libraries(BigSkyCountryEditor PRIVATE BigSkyApplication SDL3_image::SDL3_image-static)

# Both executables use the same runtime directory, including on multi-config builds.
add_custom_target(BigSkyAssets
    COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:BigSkyCountry>/licenses"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${sdl3_image_SOURCE_DIR}/LICENSE.txt" "$<TARGET_FILE_DIR:BigSkyCountry>/licenses/SDL_image.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${cgltf_SOURCE_DIR}/LICENSE" "$<TARGET_FILE_DIR:BigSkyCountry>/licenses/cgltf.txt"
    COMMAND "${CMAKE_COMMAND}"
        "-DassetSource=${CMAKE_CURRENT_SOURCE_DIR}/assets"
        "-DassetDestination=$<TARGET_FILE_DIR:BigSkyCountry>/assets"
        "-DmanifestDir=${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles"
        "-Dconfig=$<CONFIG>"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/copy_assets.cmake"
    VERBATIM
)
add_dependencies(BigSkyCountry BigSkyAssets)
add_dependencies(BigSkyCountryEditor BigSkyAssets)
