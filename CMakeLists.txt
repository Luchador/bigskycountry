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

add_library(BigSkyApplication STATIC
    shared/road.cpp
    shared/road_stripes.cpp
    shared/road_io.cpp
    shared/world_document.cpp
    shared/road_render.cpp
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
target_include_directories(BigSkyApplication PUBLIC shared)
target_link_libraries(BigSkyApplication PUBLIC SDL3::SDL3-static)
include(cmake/sun_shader.cmake)

add_executable(BigSkyCountry WIN32 game/main.cpp)
target_link_libraries(BigSkyCountry PRIVATE BigSkyApplication)

add_executable(BigSkyCountryEditor WIN32
    editor/main.cpp
    editor/camera_toolbar.cpp
    editor/road_editor.cpp
    editor/editor_menu.cpp
    editor/edit_history.cpp
)
if(WIN32)
    target_link_libraries(BigSkyCountryEditor PRIVATE user32 comctl32)
endif()
target_link_libraries(BigSkyCountryEditor PRIVATE BigSkyApplication)

# Both executables use the same runtime directory, including on multi-config builds.
add_custom_target(BigSkyTextures
    COMMAND "${CMAKE_COMMAND}"
        "-DtextureSource=${CMAKE_CURRENT_SOURCE_DIR}/assets/textures"
        "-DtextureDestination=$<TARGET_FILE_DIR:BigSkyCountry>/assets/textures"
        "-DtextureManifest=${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/BigSkyTextures-$<CONFIG>.list"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/copy_textures.cmake"
    VERBATIM
)
add_dependencies(BigSkyCountry BigSkyTextures)
add_dependencies(BigSkyCountryEditor BigSkyTextures)
