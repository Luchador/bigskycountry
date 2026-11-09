#include "world_renderer.hpp"

#include <cstring>
#include <limits>

#ifdef BIGSKY_WORLD_GPU
#include "world_vert_dxil.hpp"
#include "world_vert_spirv.hpp"
#include "world_frag_dxil.hpp"
#include "world_frag_spirv.hpp"

namespace
{
    SDL_GPUShader* CreateWorldShader(SDL_GPUDevice* device, SDL_GPUShaderStage stage)
    {
        const bool vertex = stage == SDL_GPU_SHADERSTAGE_VERTEX;
        SDL_GPUShaderCreateInfo info{};
        info.stage = stage;
        info.entrypoint = vertex ? "VSMain" : "PSMain";
        const auto formats = SDL_GetGPUShaderFormats(device);
        if (formats & SDL_GPU_SHADERFORMAT_SPIRV)
        {
            info.format = SDL_GPU_SHADERFORMAT_SPIRV;
            info.code = vertex ? world_vert_spirv : world_frag_spirv;
            info.code_size = vertex ? sizeof(world_vert_spirv) : sizeof(world_frag_spirv);
        }
        else if (formats & SDL_GPU_SHADERFORMAT_DXIL)
        {
            info.format = SDL_GPU_SHADERFORMAT_DXIL;
            info.code = vertex ? world_vert_dxil : world_frag_dxil;
            info.code_size = vertex ? sizeof(world_vert_dxil) : sizeof(world_frag_dxil);
        }
        else
        {
            SDL_SetError("No supported world shader format on this GPU");
            return nullptr;
        }
        return SDL_CreateGPUShader(device, &info);
    }
}
#endif

WorldRenderer::~WorldRenderer()
{
    if (colorTarget)
    {
        SDL_DestroyTexture(colorTarget);
    }
    if (depthTarget)
    {
        SDL_ReleaseGPUTexture(device, depthTarget);
    }
    if (vertexBuffer)
    {
        SDL_ReleaseGPUBuffer(device, vertexBuffer);
    }
    if (upload)
    {
        SDL_ReleaseGPUTransferBuffer(device, upload);
    }
    if (surfacePipeline)
    {
        SDL_ReleaseGPUGraphicsPipeline(device, surfacePipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, outlinePipeline);
    }
}

#ifdef BIGSKY_WORLD_GPU
bool WorldRenderer::CreatePipelines()
{
    for (auto format : {SDL_GPU_TEXTUREFORMAT_D32_FLOAT_S8_UINT, SDL_GPU_TEXTUREFORMAT_D24_UNORM_S8_UINT})
    {
        if (SDL_GPUTextureSupportsFormat(device, format, SDL_GPU_TEXTURETYPE_2D, SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET))
        {
            depthFormat = format;
            break;
        }
    }
    if (depthFormat == SDL_GPU_TEXTUREFORMAT_INVALID)
    {
        return SDL_SetError("No supported depth-stencil format for world rendering");
    }

    SDL_GPUShader* vs = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_VERTEX);
    if (!vs)
    {
        return false;
    }
    SDL_GPUShader* ps = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_FRAGMENT);
    if (!ps)
    {
        SDL_ReleaseGPUShader(device, vs);
        return false;
    }
    SDL_GPUVertexBufferDescription layout{0, sizeof(WorldDepthVertex), SDL_GPU_VERTEXINPUTRATE_VERTEX, 0};
    SDL_GPUVertexAttribute attributes[] = {
        {0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, 0},
        {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(WorldDepthVertex, color)}
    };
    SDL_GPUColorTargetDescription color{};
    color.format = SDL_GetGPUTextureFormatFromPixelFormat(SDL_PIXELFORMAT_RGBA32);
    SDL_GPUGraphicsPipelineCreateInfo pipeline{};
    pipeline.vertex_shader = vs;
    pipeline.fragment_shader = ps;
    pipeline.vertex_input_state = {&layout, 1, attributes, 2};
    pipeline.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    pipeline.target_info.color_target_descriptions = &color;
    pipeline.target_info.num_color_targets = 1;
    pipeline.target_info.has_depth_stencil_target = true;
    pipeline.target_info.depth_stencil_format = depthFormat;
    pipeline.rasterizer_state.front_face = SDL_GPU_FRONTFACE_CLOCKWISE;
    pipeline.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_BACK;
    pipeline.rasterizer_state.enable_depth_clip = true;
    auto& ds = pipeline.depth_stencil_state;
    ds.enable_depth_test = true;
    ds.enable_depth_write = true;
    ds.compare_op = SDL_GPU_COMPAREOP_GREATER_OR_EQUAL;
    // The nearest surface marks whether this pixel belongs to the selected road.
    ds.enable_stencil_test = true;
    ds.compare_mask = ds.write_mask = 1;
    ds.front_stencil_state = {SDL_GPU_STENCILOP_KEEP, SDL_GPU_STENCILOP_REPLACE,
                              SDL_GPU_STENCILOP_KEEP, SDL_GPU_COMPAREOP_ALWAYS};
    ds.back_stencil_state = ds.front_stencil_state;
    surfacePipeline = SDL_CreateGPUGraphicsPipeline(device, &pipeline);
    if (surfacePipeline)
    {
        pipeline.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
        ds.enable_depth_write = false;
        ds.write_mask = 0;
        ds.front_stencil_state.pass_op = SDL_GPU_STENCILOP_KEEP;
        ds.front_stencil_state.compare_op = SDL_GPU_COMPAREOP_EQUAL;
        ds.back_stencil_state = ds.front_stencil_state;
        outlinePipeline = SDL_CreateGPUGraphicsPipeline(device, &pipeline);
    }
    SDL_ReleaseGPUShader(device, vs);
    SDL_ReleaseGPUShader(device, ps);
    if (!outlinePipeline)
    {
        if (surfacePipeline)
        {
            SDL_ReleaseGPUGraphicsPipeline(device, surfacePipeline);
            surfacePipeline = nullptr;
        }
        return false;
    }
    return true;
}
#endif

bool WorldRenderer::PrepareTarget(SDL_Renderer* renderer, int width, int height)
{
    if (colorTarget && colorTarget->w == width && colorTarget->h == height)
    {
        return true;
    }
#ifdef BIGSKY_WORLD_GPU
    if (!colorTarget && !device && SDL_strcmp(SDL_GetRendererName(renderer), "gpu") == 0)
    {
        device = SDL_GetGPURendererDevice(renderer);
        if (device && !CreatePipelines())
        {
            SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Using software world depth buffer: %s", SDL_GetError());
            device = nullptr;
        }
    }
#endif
    SDL_Texture* color = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32,
                                           device ? SDL_TEXTUREACCESS_TARGET : SDL_TEXTUREACCESS_STREAMING, width, height);
    if (!color)
    {
        return false;
    }
    if (!SDL_SetTextureBlendMode(color, SDL_BLENDMODE_BLEND) || !SDL_SetTextureScaleMode(color, SDL_SCALEMODE_NEAREST))
    {
        SDL_DestroyTexture(color);
        return false;
    }
    if (device)
    {
        SDL_GPUTextureCreateInfo info{};
        info.type = SDL_GPU_TEXTURETYPE_2D;
        info.format = depthFormat;
        info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
        info.width = width;
        info.height = height;
        info.layer_count_or_depth = info.num_levels = 1;
        SDL_GPUTexture* depth = SDL_CreateGPUTexture(device, &info);
        if (!depth)
        {
            SDL_DestroyTexture(color);
            return false;
        }
        if (depthTarget)
        {
            SDL_ReleaseGPUTexture(device, depthTarget);
        }
        depthTarget = depth;
    }
    else
    {
        softwareDepth.resize(static_cast<std::size_t>(width) * height);
    }
    if (colorTarget)
    {
        SDL_DestroyTexture(colorTarget);
    }
    colorTarget = color;
    return true;
}

bool WorldRenderer::DrawGPU()
{
    if (vertices.size() > std::numeric_limits<Uint32>::max() / sizeof(WorldDepthVertex))
    {
        return SDL_SetError("World vertex buffer exceeds GPU buffer size limit");
    }
    const Uint32 bytes = static_cast<Uint32>(vertices.size() * sizeof(WorldDepthVertex));
    if (bytes > bufferBytes)
    {
        const SDL_GPUBufferCreateInfo bufferInfo{SDL_GPU_BUFFERUSAGE_VERTEX, bytes, 0};
        SDL_GPUBuffer* buffer = SDL_CreateGPUBuffer(device, &bufferInfo);
        if (!buffer)
        {
            return false;
        }
        const SDL_GPUTransferBufferCreateInfo transferInfo{SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, bytes, 0};
        SDL_GPUTransferBuffer* staging = SDL_CreateGPUTransferBuffer(device, &transferInfo);
        if (!staging)
        {
            SDL_ReleaseGPUBuffer(device, buffer);
            return false;
        }
        if (vertexBuffer)
        {
            SDL_ReleaseGPUBuffer(device, vertexBuffer);
            SDL_ReleaseGPUTransferBuffer(device, upload);
        }
        vertexBuffer = buffer;
        upload = staging;
        bufferBytes = bytes;
    }

    void* mapped = SDL_MapGPUTransferBuffer(device, upload, true);
    if (!mapped)
    {
        return false;
    }
    std::memcpy(mapped, vertices.data(), bytes);
    SDL_UnmapGPUTransferBuffer(device, upload);

    SDL_GPUColorTargetInfo color{};
    color.texture = static_cast<SDL_GPUTexture*>(SDL_GetPointerProperty(SDL_GetTextureProperties(colorTarget),
                                                SDL_PROP_TEXTURE_GPU_TEXTURE_POINTER, nullptr));
    color.load_op = SDL_GPU_LOADOP_CLEAR; // Transparent pixels reveal the sky.
    color.store_op = SDL_GPU_STOREOP_STORE;
    color.cycle = true;
    SDL_GPUDepthStencilTargetInfo depth{};
    depth.texture = depthTarget;
    depth.load_op = depth.stencil_load_op = SDL_GPU_LOADOP_CLEAR; // Reversed depth clears to zero.
    depth.store_op = depth.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth.cycle = true;
    if (!color.texture)
    {
        return SDL_SetError("World color target has no GPU texture");
    }
    SDL_GPUCommandBuffer* commands = SDL_AcquireGPUCommandBuffer(device);
    if (!commands)
    {
        return false;
    }
    SDL_GPUCopyPass* copy = SDL_BeginGPUCopyPass(commands);
    if (!copy)
    {
        SDL_CancelGPUCommandBuffer(commands);
        return false;
    }
    const SDL_GPUTransferBufferLocation src{upload, 0};
    const SDL_GPUBufferRegion dst{vertexBuffer, 0, bytes};
    SDL_UploadToGPUBuffer(copy, &src, &dst, true);
    SDL_EndGPUCopyPass(copy);

    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(commands, &color, 1, &depth);
    if (!pass)
    {
        SDL_CancelGPUCommandBuffer(commands);
        return false;
    }
    SDL_BindGPUGraphicsPipeline(pass, surfacePipeline);
    const SDL_GPUBufferBinding binding{vertexBuffer, 0};
    SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
    const auto draw = [&](std::size_t first, std::size_t end, Uint8 marker)
    {
        if (first < end)
        {
            SDL_SetGPUStencilReference(pass, marker);
            SDL_DrawGPUPrimitives(pass, static_cast<Uint32>(end - first), 1, static_cast<Uint32>(first), 0);
        }
    };
    draw(0, selectedStart, 0);
    draw(selectedStart, selectedEnd, 1);
    draw(selectedEnd, outlineStart, 0);
    if (outlineStart < vertices.size())
    {
        SDL_BindGPUGraphicsPipeline(pass, outlinePipeline);
        draw(outlineStart, vertices.size(), 1);
    }
    SDL_EndGPURenderPass(pass);
    return SDL_SubmitGPUCommandBuffer(commands);
}
