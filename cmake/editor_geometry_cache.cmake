# Rebuild the cache signature when a geometry algorithm, shared data layout or
# triangulator changes. Compiler changes also invalidate derived floating-point
# results. Authored boundary geometry is fingerprinted separately per entry.
set(geometry_cache_sources
    shared/editor_geometry_cache.cpp shared/editor_geometry_cache.hpp
    shared/terrain_fill.cpp shared/terrain_fill.hpp
    shared/lake.cpp shared/lake.hpp shared/river.cpp shared/river.hpp
    shared/math.cpp shared/math.hpp shared/mesh.hpp shared/terrain_chunk.hpp shared/road.hpp)
file(GLOB_RECURSE geometry_cache_cdt CONFIGURE_DEPENDS "${cdt_SOURCE_DIR}/CDT/include/*")
list(APPEND geometry_cache_sources ${geometry_cache_cdt})
set(geometry_cache_signature "${CMAKE_CXX_COMPILER_ID};${CMAKE_CXX_COMPILER_VERSION};${CMAKE_SYSTEM_NAME};${CMAKE_SYSTEM_PROCESSOR}")
foreach(source IN LISTS geometry_cache_sources)
    if(NOT IS_ABSOLUTE "${source}")
        set(source "${CMAKE_CURRENT_SOURCE_DIR}/${source}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${source}")
    file(SHA256 "${source}" source_hash)
    string(APPEND geometry_cache_signature ";${source_hash}")
endforeach()
string(SHA256 geometry_cache_signature "${geometry_cache_signature}")
configure_file(cmake/editor_geometry_cache_build.hpp.in generated/editor_geometry_cache_build.hpp @ONLY)
target_include_directories(BigSkyApplication PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
