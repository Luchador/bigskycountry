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

# Decode the same DDS blocks on devices using the software world renderer.
FetchContent_Declare(bcdec
    URL https://codeload.github.com/iOrange/bcdec/tar.gz/80859ed3b7afb1c527a2a99d70c61457bea72d0c
    URL_HASH SHA256=c54b589fd1db768994c8bb4bbe9dbd658fb9401ce56cb7a1c544cb1ca2c0a033
    SOURCE_SUBDIR header_only)
FetchContent_MakeAvailable(bcdec)

# Constrained Delaunay triangulation for road-bounded terrain (CDT 2.0.2).
FetchContent_Declare(cdt
    GIT_REPOSITORY https://github.com/artem-ogre/CDT.git
    GIT_TAG b4413ee44affb489099ed4d6ec65e94788cb6708
    GIT_SUBMODULES ""
    SOURCE_SUBDIR header_only)
FetchContent_MakeAvailable(cdt)

# Read and update versioned .bmat JSON in both the engine and editor.
FetchContent_Declare(cjson
    URL https://codeload.github.com/DaveGamble/cJSON/tar.gz/c859b25da02955fef659d658b8f324b5cde87be3
    URL_HASH SHA256=880a86445ad5cf38f877bac31198268d8fac5d2ce996ebf55d3066a4109ddd8d
    SOURCE_SUBDIR header_only)
FetchContent_MakeAvailable(cjson)

# Triangulate TrueType glyphs for world-space marker lettering.
FetchContent_Declare(ttf2mesh
    GIT_REPOSITORY https://github.com/fetisov/ttf2mesh.git
    GIT_TAG b767f3e1026c979c6f2d6099576f206702065234
    GIT_SUBMODULES ""
    SOURCE_SUBDIR library_only)
FetchContent_MakeAvailable(ttf2mesh)
set_source_files_properties("${ttf2mesh_SOURCE_DIR}/ttf2mesh.c" PROPERTIES COMPILE_DEFINITIONS "TTF_NO_FILESYSTEM;TTF_NO_SIGNAL_H")

add_library(BigSkyApplication STATIC
    "${ttf2mesh_SOURCE_DIR}/ttf2mesh.c"
    shared/marker.cpp
    "${cjson_SOURCE_DIR}/cJSON.c"
    shared/asset_paths.cpp
    shared/material.cpp
    shared/dds_texture.cpp
    shared/world_lighting.cpp
    shared/surface_material.cpp
    shared/bscobject.cpp
    shared/building.cpp
    shared/character.cpp
    shared/prop.cpp
    shared/vehicle.cpp
    shared/object_io.cpp
    shared/mesh.cpp
    shared/mesh_import.cpp
    shared/world_renderer.cpp
    shared/intersection.cpp
    shared/highway_ramp.cpp
    shared/road.cpp
    shared/road_stripes.cpp
    shared/road_io.cpp
    shared/scene_hierarchy.cpp
    shared/world_document.cpp
    shared/terrain_fill.cpp
    shared/world_terrain.cpp
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
    shared/geographic_coordinates.cpp
    shared/math.cpp
    shared/texture_loader.cpp
    shared/world_state.cpp
)
set_source_files_properties("${cjson_SOURCE_DIR}/cJSON.c" shared/material.cpp PROPERTIES COMPILE_DEFINITIONS CJSON_HIDE_SYMBOLS)
target_compile_features(BigSkyApplication PUBLIC cxx_std_20 PRIVATE c_std_11)
target_include_directories(BigSkyApplication PUBLIC shared PRIVATE "${ttf2mesh_SOURCE_DIR}" "${cdt_SOURCE_DIR}/CDT/include" "${cjson_SOURCE_DIR}" "${cgltf_SOURCE_DIR}" "${bcdec_SOURCE_DIR}")
target_link_libraries(BigSkyApplication PUBLIC SDL3::SDL3-static)
include(cmake/shaders.cmake)

add_executable(BigSkyCountry WIN32 game/main.cpp)
target_link_libraries(BigSkyCountry PRIVATE BigSkyApplication)

# SDL_image 3.4.0 for source material maps and editor thumbnails. Use its built-in decoders to avoid extra runtime DLLs.
set(BUILD_SHARED_LIBS OFF)
set(SDLIMAGE_VENDORED OFF)
set(SDLIMAGE_PNG_LIBPNG OFF)
set(SDLIMAGE_AVIF OFF)
set(SDLIMAGE_WEBP OFF)
set(SDLIMAGE_TIF ON)
if(WIN32)
    set(SDLIMAGE_BACKEND_WIC ON)
    set(SDLIMAGE_BACKEND_STB OFF)
elseif(NOT APPLE)
    find_package(TIFF REQUIRED)
    set(SDLIMAGE_TIF_SHARED OFF)
endif()
FetchContent_Declare(SDL3_image
    URL https://codeload.github.com/libsdl-org/SDL_image/tar.gz/ad58ecfc27a1bd09e510ceff8bbbdb1094806476
    URL_HASH SHA256=29fa60ea0432997c4ae61358a8e8fcb3638520d123a590fc9f42558967239cd2
)
FetchContent_MakeAvailable(SDL3_image)

target_link_libraries(BigSkyApplication PRIVATE SDL3_image::SDL3_image-static)
if(WIN32)
    target_link_libraries(BigSkyApplication PRIVATE windowscodecs)
endif()

add_executable(BigSkyCountryEditor WIN32
    editor/main.cpp
    editor/content_browser.cpp
    editor/material_editor.cpp
    editor/material_preview.cpp
    editor/texture_catalog.cpp
    editor/scene_outliner.cpp
    editor/road_editor.cpp
    editor/highway_ramp_tool.cpp
    editor/road_intersection.cpp
    editor/properties_inspector.cpp
    editor/transform_inspector.cpp
    editor/road_overlay.cpp
    editor/transform_gizmo.cpp
    editor/transform_settings.cpp
    editor/viewport_picker.cpp
    editor/editor_menu.cpp
    editor/edit_history.cpp
    editor/road_group_transform.cpp
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
    target_sources(BigSkyCountryEditor PRIVATE editor/windows_material_editor.cpp editor/windows_panels.cpp editor/windows_outliner.cpp editor/windows_name_dialog.cpp editor/windows_theme.cpp editor/panels.rc)
    set_source_files_properties(editor/windows_material_editor.cpp editor/windows_panels.cpp editor/windows_outliner.cpp editor/windows_name_dialog.cpp editor/windows_theme.cpp editor/editor_menu.cpp PROPERTIES COMPILE_DEFINITIONS "UNICODE;_UNICODE;NOMINMAX")
    target_include_directories(BigSkyCountryEditor PRIVATE editor)
    if(MSVC)
        target_sources(BigSkyCountryEditor PRIVATE editor/editor.manifest)
    else()
        target_compile_definitions(BigSkyCountryEditor PRIVATE BSC_EMBED_MANIFEST)
    endif()
    target_link_libraries(BigSkyCountryEditor PRIVATE user32 comctl32 comdlg32 gdi32 dwmapi uxtheme)
else()
    target_sources(BigSkyCountryEditor PRIVATE editor/sdl_material_editor.cpp editor/camera_toolbar.cpp editor/sdl_transform_panel.cpp editor/sdl_outliner.cpp)
endif()
target_link_libraries(BigSkyCountryEditor PRIVATE BigSkyApplication)

# Migrate the previous layout before globbing, including assets authored only in a build.
if(CMAKE_CURRENT_BINARY_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(FATAL_ERROR "Use an out-of-source build, for example cmake -S . -B build")
endif()
include(cmake/migrate_assets.cmake)
bigsky_migrate_assets("${CMAKE_CURRENT_BINARY_DIR}/assets")
foreach(config IN LISTS CMAKE_CONFIGURATION_TYPES)
    bigsky_migrate_assets("${CMAKE_CURRENT_BINARY_DIR}/${config}/assets")
endforeach()

include(cmake/textures.cmake)
set_property(SOURCE shared/asset_paths.cpp APPEND PROPERTY COMPILE_DEFINITIONS
    "BIGSKY_PROJECT_ASSETS=u8\"${asset_root}/\""
    "BIGSKY_COOKED_RELATIVE=u8\"$<PATH:RELATIVE_PATH,${cooked_root},$<TARGET_FILE_DIR:BigSkyCountry>>/\"")
add_custom_target(BigSkyAssets
    COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:BigSkyCountry>/licenses"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${ttf2mesh_SOURCE_DIR}/LICENSE" "$<TARGET_FILE_DIR:BigSkyCountry>/licenses/ttf2mesh.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${asset_root}/fonts/OpenSans-LICENSE.txt" "$<TARGET_FILE_DIR:BigSkyCountry>/licenses/OpenSans.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${cjson_SOURCE_DIR}/LICENSE" "$<TARGET_FILE_DIR:BigSkyCountry>/licenses/cJSON.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${bcdec_SOURCE_DIR}/LICENSE" "$<TARGET_FILE_DIR:BigSkyCountry>/licenses/bcdec.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${sdl3_image_SOURCE_DIR}/LICENSE.txt" "$<TARGET_FILE_DIR:BigSkyCountry>/licenses/SDL_image.txt"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${cgltf_SOURCE_DIR}/LICENSE" "$<TARGET_FILE_DIR:BigSkyCountry>/licenses/cgltf.txt"
    VERBATIM
)
add_dependencies(BigSkyAssets BigSkyTextures)
add_dependencies(BigSkyCountry BigSkyAssets)
add_dependencies(BigSkyCountryEditor BigSkyAssets)
