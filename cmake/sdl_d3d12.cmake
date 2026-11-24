# Local correction for the SDL 3.4.20 D3D12 graphics descriptor allocator.
# Compile an adjusted copy in the build tree; leave the downloaded SDL intact.
function(bigsky_fix_sdl_d3d12)
    if(NOT WIN32)
        return()
    endif()

    get_target_property(sdl_sources SDL3-static SOURCES)
    foreach(source IN LISTS sdl_sources)
        if(source MATCHES "/gpu/d3d12/SDL_gpu_d3d12\\.c$")
            set(original "${source}")
            break()
        endif()
    endforeach()
    if(NOT original OR NOT EXISTS "${original}")
        message(FATAL_ERROR "The D3D12 lighting fix requires the SDL source built by this project.")
    endif()

    # Reserve room for the entire draw before writing ANY descriptor table.
    # SDL's per-table overflow check can overrun a nearly full heap, or switch
    # heaps halfway through a draw and leave other tables pointing at old heaps.
    set(before [=[    D3D12GraphicsPipeline *graphicsPipeline = commandBuffer->currentGraphicsPipeline;

    /* Acquire GPU descriptor heaps if we haven't yet */
    if (commandBuffer->gpuDescriptorHeaps[0] == NULL) {
        D3D12_INTERNAL_SetGPUDescriptorHeaps(commandBuffer);
    }
]=])
    set(after [=[    D3D12GraphicsPipeline *graphicsPipeline = commandBuffer->currentGraphicsPipeline;

    /* BigSkyCountry local fix: reserve all graphics tables before binding any.
     * Changing either descriptor heap invalidates every descriptor table,
     * including the spotlight buffer even when only material textures changed.
     * Count a full rebind, not just dirty tables, so no write can switch heaps
     * partway through this draw. Old heaps remain tracked until GPU completion.
     */
    const Uint32 samplerCount = graphicsPipeline->header.num_vertex_samplers +
                               graphicsPipeline->header.num_fragment_samplers;
    const Uint32 viewCount = samplerCount +
                            graphicsPipeline->header.num_vertex_storage_textures +
                            graphicsPipeline->header.num_vertex_storage_buffers +
                            graphicsPipeline->header.num_fragment_storage_textures +
                            graphicsPipeline->header.num_fragment_storage_buffers;
    D3D12DescriptorHeap *views = commandBuffer->gpuDescriptorHeaps[D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV];
    D3D12DescriptorHeap *samplers = commandBuffer->gpuDescriptorHeaps[D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER];
    if (views == NULL || samplers == NULL ||
        views->currentDescriptorIndex + viewCount > views->maxDescriptors ||
        samplers->currentDescriptorIndex + samplerCount > samplers->maxDescriptors) {
        D3D12_INTERNAL_SetGPUDescriptorHeaps(commandBuffer);
        commandBuffer->needVertexSamplerBind = true;
        commandBuffer->needVertexStorageTextureBind = true;
        commandBuffer->needVertexStorageBufferBind = true;
        commandBuffer->needFragmentSamplerBind = true;
        commandBuffer->needFragmentStorageTextureBind = true;
        commandBuffer->needFragmentStorageBufferBind = true;
    }
]=])

    file(READ "${original}" backend)
    string(FIND "${backend}" "${before}" location)
    if(location EQUAL -1)
        message(FATAL_ERROR "SDL's D3D12 backend changed; review the graphics descriptor fix before updating SDL.")
    endif()
    string(REPLACE "${before}" "${after}" backend "${backend}")
    set(patched "${CMAKE_CURRENT_BINARY_DIR}/sdl-fixes/SDL_gpu_d3d12.c")
    # Do not cause a full SDL relink on every configure.
    if(EXISTS "${patched}")
        file(READ "${patched}" previous)
    endif()
    if(NOT backend STREQUAL previous)
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sdl-fixes")
        file(WRITE "${patched}" "${backend}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${original}")
    list(FIND sdl_sources "${original}" index)
    list(REMOVE_AT sdl_sources ${index})
    list(INSERT sdl_sources ${index} "${patched}")
    set_property(TARGET SDL3-static PROPERTY SOURCES "${sdl_sources}")
    # Preserve SDL's quoted, relative includes from the relocated source file.
    get_filename_component(backend_directory "${original}" DIRECTORY)
    target_include_directories(SDL3-static PRIVATE "${backend_directory}")
    message(STATUS "Applied SDL D3D12 graphics descriptor fix")
endfunction()
