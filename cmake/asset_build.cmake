# Use the configured tools/build tree even when the editor was opened from Explorer.
# Only the asset target is built: never relink the running editor or world cooker.
set(asset_build_environment "")
if(MSVC AND NOT CMAKE_GENERATOR MATCHES "Visual Studio")
    foreach(name IN ITEMS PATH INCLUDE LIB LIBPATH)
        if(DEFINED ENV{${name}})
            string(APPEND asset_build_environment "set(ENV{${name}} [==[$ENV{${name}}]==])\n")
        endif()
    endforeach()
endif()
configure_file("${CMAKE_CURRENT_LIST_DIR}/build_assets.cmake.in" generated/build_assets.cmake.in @ONLY)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/$<CONFIG>/build_assets.cmake"
     INPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/build_assets.cmake.in")
configure_file("${CMAKE_CURRENT_LIST_DIR}/asset_build_settings.hpp.in" generated/asset_build_settings.hpp.in @ONLY)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/$<CONFIG>/asset_build_settings.hpp"
     INPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/asset_build_settings.hpp.in")
set_property(SOURCE shared/asset_build.cpp APPEND PROPERTY INCLUDE_DIRECTORIES "${CMAKE_CURRENT_BINARY_DIR}/generated/$<CONFIG>")
