#include "world_renderer.hpp"
#include "free_camera.hpp"
#include "editor_colors.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <span>

#ifdef BIGSKY_WORLD_GPU
#include "world_vert_dxil.hpp"
#include "world_vert_spirv.hpp"
#include "world_frag_dxil.hpp"
#include "world_frag_spirv.hpp"
#include "wireframe_frag_dxil.hpp"
#include "wireframe_frag_spirv.hpp"
#include "material_frag_dxil.hpp"
#include "material_frag_spirv.hpp"

namespace
{
    SDL_GPUShader* CreateWorldShader(SDL_GPUDevice* device, SDL_GPUShaderStage stage, const char* entry,
                                     std::span<const Uint8> dxil, std::span<const Uint8> spirv, Uint32 samplers = 0, Uint32 uniforms = 0)
    {
        SDL_GPUShaderCreateInfo info{};
        info.stage = stage;
        info.entrypoint = entry;
        info.num_samplers = samplers;
        info.num_uniform_buffers = uniforms;
        const auto formats = SDL_GetGPUShaderFormats(device);
        if (formats & SDL_GPU_SHADERFORMAT_SPIRV)
        {
            info.format = SDL_GPU_SHADERFORMAT_SPIRV;
            info.code = spirv.data();
            info.code_size = spirv.size();
        }
        else if (formats & SDL_GPU_SHADERFORMAT_DXIL)
        {
            info.format = SDL_GPU_SHADERFORMAT_DXIL;
            info.code = dxil.data();
            info.code_size = dxil.size();
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

WorldRenderer::ResidentGeometry::~ResidentGeometry()
{
    if (buffer) SDL_ReleaseGPUBuffer(device, buffer);
}

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
        SDL_ReleaseGPUGraphicsPipeline(device, translucentPipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, texturedPipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, wireframePipeline);
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

    SDL_GPUShader* vs = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_VERTEX, "VSMain", world_vert_dxil, world_vert_spirv, 0, 1);
    if (!vs)
    {
        return false;
    }
    SDL_GPUShader* ps = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, "PSMain", world_frag_dxil, world_frag_spirv, 0, 1);
    if (!ps)
    {
        SDL_ReleaseGPUShader(device, vs);
        return false;
    }
    SDL_GPUVertexBufferDescription layout{0, sizeof(WorldDepthVertex), SDL_GPU_VERTEXINPUTRATE_VERTEX, 0};
    SDL_GPUVertexAttribute attributes[] = {
        {0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, 0},
        {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(WorldDepthVertex, color)},
        {2, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, offsetof(WorldDepthVertex, uv)},
        {3, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(WorldDepthVertex, normal)},
        {4, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(WorldDepthVertex, tangent)},
        {5, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(WorldDepthVertex, view)}
    };
    SDL_GPUColorTargetDescription color{};
    color.format = SDL_GetGPUTextureFormatFromPixelFormat(SDL_PIXELFORMAT_RGBA32);
    SDL_GPUGraphicsPipelineCreateInfo pipeline{};
    pipeline.vertex_shader = vs;
    pipeline.fragment_shader = ps;
    pipeline.vertex_input_state = {&layout, 1, attributes, SDL_arraysize(attributes)};
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
    // The nearest surface marks whether this pixel belongs to selected geometry.
    ds.enable_stencil_test = true;
    ds.compare_mask = ds.write_mask = 1;
    ds.front_stencil_state = {SDL_GPU_STENCILOP_KEEP, SDL_GPU_STENCILOP_REPLACE,
                              SDL_GPU_STENCILOP_KEEP, SDL_GPU_COMPAREOP_ALWAYS};
    ds.back_stencil_state = ds.front_stencil_state;
    surfacePipeline = SDL_CreateGPUGraphicsPipeline(device, &pipeline);

    const auto materialPipeline = [&](const char* entry, std::span<const Uint8> dxil, std::span<const Uint8> spirv, Uint32 samplers, Uint32 uniforms = 0) -> SDL_GPUGraphicsPipeline*
    {
        auto* shader = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, entry, dxil, spirv, samplers, uniforms);
        if (!shader) return nullptr;
        auto material = pipeline;
        material.fragment_shader = shader;
        auto* result = SDL_CreateGPUGraphicsPipeline(device, &material);
        SDL_ReleaseGPUShader(device, shader);
        return result;
    };

    if (surfacePipeline)
    {
        texturedPipeline = materialPipeline("MaterialPS", material_frag_dxil, material_frag_spirv, SurfaceMaterial::shaderMapCount, 2);
    }

    if (texturedPipeline)
    {
        // Accumulate premultiplied color in the initially transparent world target.
        auto& blend = color.blend_state;
        blend.enable_blend = true;
        blend.src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
        blend.dst_color_blendfactor = blend.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        blend.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        blend.color_blend_op = blend.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        ds.enable_depth_write = false;
        translucentPipeline = materialPipeline("MaterialPS", material_frag_dxil, material_frag_spirv, SurfaceMaterial::shaderMapCount, 2);
        color.blend_state = {};
        ds.enable_depth_write = true;
    }
    if (translucentPipeline)
    {
        wireframePipeline = materialPipeline("WireframePS", wireframe_frag_dxil, wireframe_frag_spirv, 0);
    }

    if (wireframePipeline)
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
        if (wireframePipeline)
        {
            SDL_ReleaseGPUGraphicsPipeline(device, wireframePipeline);
            wireframePipeline = nullptr;
        }
        if (translucentPipeline)
        {
            SDL_ReleaseGPUGraphicsPipeline(device, translucentPipeline);
            translucentPipeline = nullptr;
        }
        if (texturedPipeline)
        {
            SDL_ReleaseGPUGraphicsPipeline(device, texturedPipeline);
            texturedPipeline = nullptr;
        }
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

/**
 * This runs on first draw or when the viewport changes size and prepares one texture to hold the rendered world, and another for the depth buffer.
 */
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
    SDL_Texture* color = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, device ? SDL_TEXTUREACCESS_TARGET : SDL_TEXTUREACCESS_STREAMING, width, height);

    if (!color)
    {
        return false;
    }

    if (!SDL_SetTextureBlendMode(color, device ? SDL_BLENDMODE_BLEND_PREMULTIPLIED : SDL_BLENDMODE_BLEND) || !SDL_SetTextureScaleMode(color, SDL_SCALEMODE_NEAREST))
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

    if (colorTarget) SDL_DestroyTexture(colorTarget);
    else if (device)
        SDL_Log("World renderer: %s on %s", SDL_GetGPUDeviceDriver(device),
                SDL_GetStringProperty(SDL_GetGPUDeviceProperties(device), SDL_PROP_GPU_DEVICE_NAME_STRING, "unknown adapter"));
    else
        SDL_Log("World renderer: CPU rasterizer (SDL renderer: %s)", SDL_GetRendererName(renderer));

    colorTarget = color;

    return true;
}

bool WorldRenderer::DrawGPU()
{
    // Stage newly visible or edited assets together. Once uploaded, their buffers
    // stay on the device. Only camera uniforms and dynamic geometry change each frame.
    constexpr size_t maxVertices = std::numeric_limits<Uint32>::max() / sizeof(WorldDepthVertex);
    if (vertices.size() > maxVertices) return SDL_SetError("Dynamic world vertex buffer exceeds GPU limit");
    const auto bytes = static_cast<Uint32>(vertices.size() * sizeof(WorldDepthVertex));
    struct Upload { ResidentGeometry* geometry; Uint32 offset, size; };
    std::vector<Upload> uploads;
    Uint32 total = bytes;
    for (const auto& draw : residentDraws)
    {
        auto& mesh = *draw.geometry;
        if (mesh.pendingUpload.empty() || mesh.uploadFrame == frame) continue;
        if (mesh.pendingUpload.size() > maxVertices) return SDL_SetError("World mesh exceeds GPU buffer limit");
        const auto size = static_cast<Uint32>(mesh.pendingUpload.size() * sizeof(WorldDepthVertex));
        if (size > std::numeric_limits<Uint32>::max() - total) return SDL_SetError("World upload exceeds GPU buffer limit");
        const SDL_GPUBufferCreateInfo info{SDL_GPU_BUFFERUSAGE_VERTEX, size, 0};
        if (!mesh.buffer) mesh.buffer = SDL_CreateGPUBuffer(device, &info);
        if (!mesh.buffer) return false;
        mesh.uploadFrame = frame;
        uploads.push_back({&mesh, total, size});
        total += size;
    }
    const auto grow = [](Uint32 required, Uint32 previous) {
        return static_cast<Uint32>(std::min<Uint64>(std::numeric_limits<Uint32>::max(),
                                    std::max<Uint64>(required, previous + Uint64(previous) / 2)));
    };
    if (bytes > bufferBytes)
    {
        const auto capacity = grow(bytes, bufferBytes);
        const SDL_GPUBufferCreateInfo info{SDL_GPU_BUFFERUSAGE_VERTEX, capacity, 0};
        auto* replacement = SDL_CreateGPUBuffer(device, &info);
        if (!replacement) return false;
        if (vertexBuffer) SDL_ReleaseGPUBuffer(device, vertexBuffer);
        vertexBuffer = replacement;
        bufferBytes = capacity;
    }
    if (total > uploadBytes)
    {
        const auto capacity = grow(total, uploadBytes);
        const SDL_GPUTransferBufferCreateInfo info{SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, capacity, 0};
        auto* replacement = SDL_CreateGPUTransferBuffer(device, &info);
        if (!replacement) return false;
        if (upload) SDL_ReleaseGPUTransferBuffer(device, upload);
        upload = replacement;
        uploadBytes = capacity;
    }
    if (total)
    {
        auto* mapped = static_cast<Uint8*>(SDL_MapGPUTransferBuffer(device, upload, true));
        if (!mapped) return false;
        if (bytes) std::memcpy(mapped, vertices.data(), bytes);
        for (const auto& item : uploads)
            std::memcpy(mapped + item.offset, item.geometry->pendingUpload.data(), item.size);
        SDL_UnmapGPUTransferBuffer(device, upload);
    }

    SDL_GPUColorTargetInfo color{};
    color.texture = static_cast<SDL_GPUTexture*>(SDL_GetPointerProperty(SDL_GetTextureProperties(colorTarget), SDL_PROP_TEXTURE_GPU_TEXTURE_POINTER, nullptr));
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

    if (total)
    {
        auto* copy = SDL_BeginGPUCopyPass(commands);
        if (!copy)
        {
            SDL_CancelGPUCommandBuffer(commands);
            return false;
        }
        const auto send = [&](SDL_GPUBuffer* buffer, Uint32 offset, Uint32 size, bool cycle) {
            const SDL_GPUTransferBufferLocation src{upload, offset};
            const SDL_GPUBufferRegion dst{buffer, 0, size};
            SDL_UploadToGPUBuffer(copy, &src, &dst, cycle);
        };
        if (bytes) send(vertexBuffer, 0, bytes, true);
        for (const auto& item : uploads) send(item.geometry->buffer, item.offset, item.size, false);
        SDL_EndGPUCopyPass(copy);
    }

    SDL_GPURenderPass* pass = SDL_BeginGPURenderPass(commands, &color, 1, &depth);

    if (!pass)
    {
        SDL_CancelGPUCommandBuffer(commands);
        return false;
    }

    SDL_BindGPUGraphicsPipeline(pass, surfacePipeline);
    // Matches WorldLights in world.hlsl. Both world surface pipelines read this block.
    const auto pack = [](Vector3 v) -> SDL_FColor { return {float(v.x), float(v.y), float(v.z), 0}; };
    const SDL_FColor lighting[]{pack(lights.sun.direction), pack(lights.sun.radiance), pack(lights.moon.direction),
                                pack(lights.moon.radiance), pack(lights.ambient)};
    SDL_PushGPUFragmentUniformData(commands, 0, lighting, sizeof(lighting));
    // Form the origin-to-eye offset in double before packing the shader uniforms.
    const auto axis = [](Vector3 v, float w = 0) { return SDL_FColor{float(v.x), float(v.y), float(v.z), w}; };
    SDL_FColor cameraData[]{axis(camera->Right() * scaleX), axis(camera->Up() * scaleY),
                            axis(camera->Forward(), float(worldNearDepth)), {}, {}, EditorColors::wire};
    SDL_GPUBuffer* source = nullptr;
    const auto bindSource = [&](SDL_GPUBuffer* buffer, const ResidentGeometry* mesh) {
        if (source == buffer) return;
        source = buffer;
        const SDL_GPUBufferBinding binding{buffer, 0};
        SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
        const Vector3 offset = mesh ? mesh->origin - camera->GetPosition() : Vector3{};
        cameraData[3] = axis(offset, mesh ? (residentWire ? 2.f : 1.f) : 0.f);
        cameraData[4] = axis(offset - Vector3{cameraData[3].r, cameraData[3].g, cameraData[3].b});
        SDL_PushGPUVertexUniformData(commands, 0, cameraData, sizeof(cameraData));
    };

    const auto draw = [&](std::size_t first, std::size_t end, Uint8 marker)
    {
        if (first < end)
        {
            SDL_SetGPUStencilReference(pass, marker);
            SDL_DrawGPUPrimitives(pass, static_cast<Uint32>(end - first), 1, static_cast<Uint32>(first), 0);
        }
    };

    SDL_GPUGraphicsPipeline* bound = surfacePipeline;
    const SurfaceMaterial* boundMaterial = nullptr;
    const auto bindPipeline = [&](const SurfaceMaterial* material, bool wire) {
        auto* pipeline = surfacePipeline;
        if (wire) pipeline = wireframePipeline;
        else if (material) pipeline = material->type == MaterialType::Unlit ? translucentPipeline : texturedPipeline;
        if (pipeline != bound)
        {
            SDL_BindGPUGraphicsPipeline(pass, pipeline);
            bound = pipeline;
        }
        if (material && material != boundMaterial) materialCache.Bind(*material, commands, pass);
        boundMaterial = material;
    };
    const auto drawResident = [&](const ResidentDraw& batch) {
        bindSource(batch.geometry->buffer, batch.geometry.get());
        for (const auto& part : batch.geometry->parts)
        {
            bindPipeline(residentWire ? nullptr : part.material, residentWire);
            draw(part.first, part.end, batch.selected);
        }
    };
    auto batch = residentDraws.begin();
    for (const auto& range : draws)
    {
        size_t first = range.first;
        // Draw opaque geometry in its original order, including coplanar paint
        // and the editor overlays. Globally sorted translucent draws come last.
        const bool translucent = range.material && range.material->type == MaterialType::Unlit;
        while (batch != residentDraws.end() && (translucent || batch->before < range.end))
        {
            const size_t split = translucent ? first : std::clamp(batch->before, first, range.end);
            if (first < split)
            {
                bindSource(vertexBuffer, nullptr);
                bindPipeline(range.material, first < wireframeEnd);
                draw(first, split, range.selected);
                first = split;
            }
            drawResident(*batch++);
        }
        bindSource(vertexBuffer, nullptr);
        bindPipeline(range.material, first < wireframeEnd);
        draw(first, range.end, range.selected);
    }
    while (batch != residentDraws.end()) drawResident(*batch++);

    if (outlineStart < foregroundStart)
    {
        bindSource(vertexBuffer, nullptr);
        SDL_BindGPUGraphicsPipeline(pass, outlinePipeline);
        draw(outlineStart, foregroundStart, 1);
    }

    SDL_EndGPURenderPass(pass);
    if (foregroundStart < vertices.size())
    {
        // Keep the scene color and start a fresh depth buffer for the gizmo.
        color.load_op = SDL_GPU_LOADOP_LOAD;
        color.cycle = false;
        pass = SDL_BeginGPURenderPass(commands, &color, 1, &depth);
        if (!pass)
        {
            SDL_CancelGPUCommandBuffer(commands);
            return false;
        }
        SDL_BindGPUGraphicsPipeline(pass, surfacePipeline);
        source = nullptr; // Bindings are local to a render pass.
        bindSource(vertexBuffer, nullptr);
        draw(foregroundStart, vertices.size(), 0);
        SDL_EndGPURenderPass(pass);
    }
    if (!SDL_SubmitGPUCommandBuffer(commands)) return false;
    for (const auto& item : uploads) std::vector<WorldDepthVertex>().swap(item.geometry->pendingUpload);
    return true;
}
