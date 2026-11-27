#include "world_renderer.hpp"
#include "camera_view.hpp"
#include "editor_colors.hpp"
#include "runtime_config.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <span>
#include <cmath>
#include <numbers>

#ifdef BIGSKY_WORLD_GPU
#include "world_vert_dxil.hpp"
#include "world_vert_spirv.hpp"
#include "world_frag_dxil.hpp"
#include "world_frag_spirv.hpp"
#include "wireframe_frag_dxil.hpp"
#include "wireframe_frag_spirv.hpp"
#include "material_frag_dxil.hpp"
#include "material_frag_spirv.hpp"
#include "water_frag_dxil.hpp"
#include "water_frag_spirv.hpp"
#include "material_msaa2_dxil.hpp"
#include "material_msaa2_spirv.hpp"
#include "material_msaa4_dxil.hpp"
#include "material_msaa4_spirv.hpp"
#include "shadow_vert_dxil.hpp"
#include "shadow_vert_spirv.hpp"
#include "shadow_frag_dxil.hpp"
#include "shadow_frag_spirv.hpp"
#include "shadow_cutout_dxil.hpp"
#include "shadow_cutout_spirv.hpp"

namespace
{
    SDL_GPUShader* CreateWorldShader(SDL_GPUDevice* device, SDL_GPUShaderStage stage, const char* entry,
                                     std::span<const Uint8> dxil, std::span<const Uint8> spirv, Uint32 samplers = 0, Uint32 uniforms = 0, Uint32 storageBuffers = 0)
    {
        SDL_GPUShaderCreateInfo info{};
        info.stage = stage;
        info.entrypoint = entry;
        info.num_samplers = samplers;
        info.num_uniform_buffers = uniforms;
        info.num_storage_buffers = storageBuffers;
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
    ReleasePostProcessing();
    if (waterPipeline) SDL_ReleaseGPUGraphicsPipeline(device, waterPipeline);
    if (waterScene) SDL_ReleaseGPUTexture(device, waterScene);
    if (waterSampler) SDL_ReleaseGPUSampler(device, waterSampler);
    ReleaseSSAO();
    if (multisampleColor) SDL_ReleaseGPUTexture(device, multisampleColor);
    if (spotBuffer) SDL_ReleaseGPUBuffer(device, spotBuffer);
    if (shadowTarget) SDL_ReleaseGPUTexture(device, shadowTarget);
    if (shadowSampler) SDL_ReleaseGPUSampler(device, shadowSampler);
    if (shadowPipeline) SDL_ReleaseGPUGraphicsPipeline(device, shadowPipeline);
    if (shadowCutoutPipeline) SDL_ReleaseGPUGraphicsPipeline(device, shadowCutoutPipeline);
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
        SDL_ReleaseGPUGraphicsPipeline(device, cutoutPipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, foliagePipeline);
        SDL_ReleaseGPUGraphicsPipeline(device, wireframePipeline);
    }
}

#ifdef BIGSKY_WORLD_GPU
bool WorldRenderer::CreatePipelines()
{
    const int requested = RuntimeConfig::Get().msaa;
    const auto colorFormat = sceneFormat;
    if (!SDL_GPUTextureSupportsFormat(device, sceneFormat, SDL_GPU_TEXTURETYPE_2D,
        SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER))
        return SDL_SetError("RGBA16F HDR render targets are not supported");
    // Color and depth/stencil must agree. Prefer the requested quality, then
    // fall back to lower counts rather than dropping to the software renderer.
    for (const auto count : {SDL_GPU_SAMPLECOUNT_4, SDL_GPU_SAMPLECOUNT_2, SDL_GPU_SAMPLECOUNT_1})
    {
        const int samples = count == SDL_GPU_SAMPLECOUNT_4 ? 4 : count == SDL_GPU_SAMPLECOUNT_2 ? 2 : 0;
        if (samples > requested || !SDL_GPUTextureSupportsSampleCount(device, colorFormat, count)) continue;
        for (auto format : {SDL_GPU_TEXTUREFORMAT_D32_FLOAT_S8_UINT, SDL_GPU_TEXTUREFORMAT_D24_UNORM_S8_UINT})
        {
            if (SDL_GPUTextureSupportsFormat(device, format, SDL_GPU_TEXTURETYPE_2D, SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET) &&
                SDL_GPUTextureSupportsSampleCount(device, format, count))
            {
                depthFormat = format;
                sampleCount = count;
                break;
            }
        }
        if (depthFormat != SDL_GPU_TEXTUREFORMAT_INVALID) break;
    }
    if (depthFormat == SDL_GPU_TEXTUREFORMAT_INVALID)
    {
        return SDL_SetError("No supported depth-stencil format for world rendering");
    }
    if (MsaaSamples() != requested)
        SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Requested MSAA %d; using %d for supported color/depth formats", requested, MsaaSamples());

    SDL_GPUShader* vs = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_VERTEX, "VSMain", world_vert_dxil, world_vert_spirv, 0, 1);
    if (!vs)
    {
        return false;
    }
    SDL_GPUShader* ps = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, "PSMain", world_frag_dxil, world_frag_spirv, 2, 1, 1);
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
        {5, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(WorldDepthVertex, view)},
        {6, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, offsetof(WorldDepthVertex, outerUV)},
        {7, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(WorldDepthVertex, outerTangent)}
    };
    SDL_GPUColorTargetDescription color{};
    color.format = sceneFormat;
    SDL_GPUGraphicsPipelineCreateInfo pipeline{};
    pipeline.vertex_shader = vs;
    pipeline.fragment_shader = ps;
    pipeline.vertex_input_state = {&layout, 1, attributes, SDL_arraysize(attributes)};
    pipeline.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    pipeline.target_info.color_target_descriptions = &color;
    pipeline.target_info.num_color_targets = 1;
    pipeline.target_info.has_depth_stencil_target = true;
    pipeline.target_info.depth_stencil_format = depthFormat;
    pipeline.multisample_state.sample_count = sampleCount;
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

    const auto materialPipeline = [&](const char* entry, std::span<const Uint8> dxil, std::span<const Uint8> spirv, Uint32 samplers, Uint32 uniforms = 0, Uint32 storageBuffers = 0) -> SDL_GPUGraphicsPipeline*
    {
        auto* shader = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, entry, dxil, spirv, samplers, uniforms, storageBuffers);
        if (!shader) return nullptr;
        auto material = pipeline;
        material.fragment_shader = shader;
        auto* result = SDL_CreateGPUGraphicsPipeline(device, &material);
        SDL_ReleaseGPUShader(device, shader);
        return result;
    };

    if (surfacePipeline)
    {
        texturedPipeline = materialPipeline("MaterialPS", material_frag_dxil, material_frag_spirv, SurfaceMaterial::shaderMapCount + 2, 2, 1);
        waterPipeline = materialPipeline("WaterPS", water_frag_dxil, water_frag_spirv, 5, 2, 1);
    }

    if (texturedPipeline)
    {
        const auto dxil = sampleCount == SDL_GPU_SAMPLECOUNT_4 ? std::span<const Uint8>(material_msaa4_dxil) :
                          sampleCount == SDL_GPU_SAMPLECOUNT_2 ? std::span<const Uint8>(material_msaa2_dxil) : std::span<const Uint8>(material_frag_dxil);
        const auto spirv = sampleCount == SDL_GPU_SAMPLECOUNT_4 ? std::span<const Uint8>(material_msaa4_spirv) :
                           sampleCount == SDL_GPU_SAMPLECOUNT_2 ? std::span<const Uint8>(material_msaa2_spirv) : std::span<const Uint8>(material_frag_spirv);
        // Shader alpha-to-coverage keeps alpha=1 in covered samples, necessary
        // for compositing this initially transparent target over SDL's sky.
        cutoutPipeline = materialPipeline("MaterialPS", dxil, spirv, SurfaceMaterial::shaderMapCount + 2, 2, 1);
        pipeline.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
        if (cutoutPipeline) foliagePipeline = materialPipeline("MaterialPS", dxil, spirv, SurfaceMaterial::shaderMapCount + 2, 2, 1);
        pipeline.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_BACK;
    }

    if (foliagePipeline)
    {
        // Accumulate premultiplied color in the initially transparent world target.
        auto& blend = color.blend_state;
        blend.enable_blend = true;
        blend.src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
        blend.dst_color_blendfactor = blend.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        blend.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        blend.color_blend_op = blend.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
        ds.enable_depth_write = false;
        translucentPipeline = materialPipeline("MaterialPS", material_frag_dxil, material_frag_spirv, SurfaceMaterial::shaderMapCount + 2, 2, 1);
        color.blend_state = {};
        ds.enable_depth_write = true;
    }
    if (translucentPipeline)
    {
        wireframePipeline = materialPipeline("WireframePS", wireframe_frag_dxil, wireframe_frag_spirv, 0, 1);
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

    if (outlinePipeline)
    {
        for (auto format : {SDL_GPU_TEXTUREFORMAT_D32_FLOAT, SDL_GPU_TEXTUREFORMAT_D16_UNORM})
            if (SDL_GPUTextureSupportsFormat(device, format, SDL_GPU_TEXTURETYPE_2D_ARRAY,
                    SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER))
            {
                shadowFormat = format;
                break;
            }
        auto* shadowVS = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_VERTEX, "ShadowVS", shadow_vert_dxil, shadow_vert_spirv, 0, 1);
        auto* shadowPS = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, "ShadowPS", shadow_frag_dxil, shadow_frag_spirv);
        auto* cutoutPS = CreateWorldShader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, "ShadowCutoutPS", shadow_cutout_dxil, shadow_cutout_spirv, 2, 1);
        if (shadowVS && shadowPS && cutoutPS && shadowFormat != SDL_GPU_TEXTUREFORMAT_INVALID)
        {
            SDL_GPUGraphicsPipelineCreateInfo shadow{};
            shadow.vertex_shader = shadowVS;
            shadow.fragment_shader = shadowPS;
            shadow.vertex_input_state = pipeline.vertex_input_state;
            shadow.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
            shadow.target_info.has_depth_stencil_target = true;
            shadow.target_info.depth_stencil_format = shadowFormat;
            shadow.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
            shadow.rasterizer_state.enable_depth_clip = true;
            shadow.depth_stencil_state.enable_depth_test = true;
            shadow.depth_stencil_state.enable_depth_write = true;
            shadow.depth_stencil_state.compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL;
            shadowPipeline = SDL_CreateGPUGraphicsPipeline(device, &shadow);
            shadow.fragment_shader = cutoutPS;
            shadowCutoutPipeline = SDL_CreateGPUGraphicsPipeline(device, &shadow);
            SDL_GPUSamplerCreateInfo sampler{};
            sampler.min_filter = sampler.mag_filter = SDL_GPU_FILTER_LINEAR;
            sampler.address_mode_u = sampler.address_mode_v = sampler.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
            sampler.enable_compare = true;
            sampler.compare_op = SDL_GPU_COMPAREOP_LESS_OR_EQUAL;
            shadowSampler = SDL_CreateGPUSampler(device, &sampler);
        }
        if (shadowVS) SDL_ReleaseGPUShader(device, shadowVS);
        if (shadowPS) SDL_ReleaseGPUShader(device, shadowPS);
        if (cutoutPS) SDL_ReleaseGPUShader(device, cutoutPS);
    }

    if (!waterPipeline || !outlinePipeline || !shadowPipeline || !shadowCutoutPipeline || !shadowSampler || !PrepareShadowTarget(1) || !CreateSSAOFallback() || !CreatePostPipelines())
    {
        ReleasePostProcessing();
        ReleaseSSAO();
        if (waterPipeline) SDL_ReleaseGPUGraphicsPipeline(device, waterPipeline);
        waterPipeline = nullptr;
        if (shadowTarget) SDL_ReleaseGPUTexture(device, shadowTarget);
        if (shadowSampler) SDL_ReleaseGPUSampler(device, shadowSampler);
        if (shadowPipeline) SDL_ReleaseGPUGraphicsPipeline(device, shadowPipeline);
        if (shadowCutoutPipeline) SDL_ReleaseGPUGraphicsPipeline(device, shadowCutoutPipeline);
        if (outlinePipeline) SDL_ReleaseGPUGraphicsPipeline(device, outlinePipeline);
        shadowTarget = nullptr;
        shadowSampler = nullptr;
        shadowPipeline = shadowCutoutPipeline = outlinePipeline = nullptr;
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
        if (cutoutPipeline)
        {
            SDL_ReleaseGPUGraphicsPipeline(device, cutoutPipeline);
            cutoutPipeline = nullptr;
        }
        if (foliagePipeline)
        {
            SDL_ReleaseGPUGraphicsPipeline(device, foliagePipeline);
            foliagePipeline = nullptr;
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
            sampleCount = SDL_GPU_SAMPLECOUNT_1;
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
        info.sample_count = sampleCount;
        SDL_GPUTexture* depth = SDL_CreateGPUTexture(device, &info);

        if (!depth)
        {
            SDL_DestroyTexture(color);
            return false;
        }
        SDL_GPUTexture* multisampled = nullptr;
        if (sampleCount != SDL_GPU_SAMPLECOUNT_1)
        {
            info.format = sceneFormat;
            info.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
            multisampled = SDL_CreateGPUTexture(device, &info);
            if (!multisampled)
            {
                SDL_ReleaseGPUTexture(device, depth);
                SDL_DestroyTexture(color);
                return false;
            }
        }

        if (depthTarget)
        {
            SDL_ReleaseGPUTexture(device, depthTarget);
        }

        if (!PreparePostTargets(width, height))
        {
            SDL_ReleaseGPUTexture(device, depth);
            if (multisampled) SDL_ReleaseGPUTexture(device, multisampled);
            SDL_DestroyTexture(color);
            depthTarget = nullptr;
            return false;
        }
        depthTarget = depth;
        if (multisampleColor) SDL_ReleaseGPUTexture(device, multisampleColor);
        multisampleColor = multisampled;
    }
    else
    {
        softwareDepth.resize(static_cast<std::size_t>(width) * height);
    }

    if (colorTarget) SDL_DestroyTexture(colorTarget);
    else if (device)
        SDL_Log("World renderer: %s on %s; MSAA %d", SDL_GetGPUDeviceDriver(device),
                SDL_GetStringProperty(SDL_GetGPUDeviceProperties(device), SDL_PROP_GPU_DEVICE_NAME_STRING, "unknown adapter"), MsaaSamples());
    else
        SDL_Log("World renderer: CPU rasterizer (SDL renderer: %s)", SDL_GetRendererName(renderer));

    colorTarget = color;

    return true;
}

// A tiny valid texture serves unshadowed previews without allocating full cascades.
bool WorldRenderer::PrepareShadowTarget(int resolution)
{
    if (shadowTarget && shadowResolution >= resolution) return true;
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D_ARRAY;
    info.format = shadowFormat;
    info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    info.width = info.height = resolution;
    info.layer_count_or_depth = ShadowCascades::count;
    info.num_levels = 1;
    auto* replacement = SDL_CreateGPUTexture(device, &info);
    if (!replacement) return false;
    if (shadowTarget) SDL_ReleaseGPUTexture(device, shadowTarget);
    shadowTarget = replacement;
    shadowResolution = resolution;
    shadowInitialized = false;
    return true;
}

bool WorldRenderer::DrawShadows(SDL_GPUCommandBuffer* commands)
{
    if (!shadows.active && shadowInitialized) return true;
    const auto axis = [](Vector3 v, float w = 0) { return SDL_FColor{float(v.x), float(v.y), float(v.z), w}; };
    for (int i = 0; i < ShadowCascades::count; ++i)
    {
        SDL_GPUDepthStencilTargetInfo depth{};
        depth.texture = shadowTarget;
        depth.layer = static_cast<Uint8>(i);
        depth.clear_depth = 1;
        depth.load_op = SDL_GPU_LOADOP_CLEAR;
        depth.store_op = SDL_GPU_STOREOP_STORE;
        depth.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
        depth.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
        depth.cycle = i == 0; // Cycle the array once; retain the layers already rendered.
        auto* pass = SDL_BeginGPURenderPass(commands, nullptr, 0, &depth);
        if (!pass) return false;
        if (shadows.active)
        {
            const auto& cascade = shadows.cascades[i];
            const auto rows = cascade.Rows(camera->GetPosition());
            SDL_FColor uniforms[]{rows[0], rows[1], rows[2], {}, {}};
            const auto bindSource = [&](SDL_GPUBuffer* buffer, const ResidentGeometry* mesh) {
                const SDL_GPUBufferBinding binding{buffer, 0};
                SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
                const Vector3 offset = mesh ? mesh->origin - camera->GetPosition() : Vector3{};
                uniforms[3] = axis(offset, mesh ? 1.f : 0.f);
                uniforms[4] = axis(offset - Vector3{uniforms[3].r, uniforms[3].g, uniforms[3].b});
                SDL_PushGPUVertexUniformData(commands, 0, uniforms, sizeof(uniforms));
            };
            SDL_GPUGraphicsPipeline* bound = nullptr;
            const auto draw = [&](size_t first, size_t end, const SurfaceMaterial* material) {
                if (first >= end || (material && (material->type == MaterialType::Unlit || material->type == MaterialType::Water))) return;
                const bool cutout = material && Material::IsCutout(material->type);
                auto* pipeline = cutout ? shadowCutoutPipeline : shadowPipeline;
                if (bound != pipeline) SDL_BindGPUGraphicsPipeline(pass, bound = pipeline);
                if (cutout)
                {
                    materialCache.BindShadow(*material, pass);
                    const SDL_FColor properties{material->alphaCutoff, 0, 0, 0};
                    SDL_PushGPUFragmentUniformData(commands, 0, &properties, sizeof(properties));
                }
                SDL_DrawGPUPrimitives(pass, static_cast<Uint32>(end-first), 1, static_cast<Uint32>(first), 0);
            };
            for (const auto& batch : residentDraws)
            {
                if (batch.uploadOnly) continue;
                const auto& mesh = *batch.geometry;
                if (!mesh.buffer || (mesh.uploadedBytes < mesh.bufferBytes &&
                    !(mesh.uploadFrame == frame && mesh.uploadEnd == mesh.bufferBytes)) || !cascade.Intersects(mesh.bounds)) continue;
                bindSource(mesh.buffer, &mesh);
                for (const auto& part : mesh.parts) draw(part.first, part.end, part.material);
            }
            const size_t end = std::min(shadowEnd, outlineStart);
            if (vertexBuffer && end)
            {
                bindSource(vertexBuffer, nullptr);
                for (const auto& range : draws) draw(range.first, std::min(range.end, end), range.material);
            }
        }
        SDL_EndGPURenderPass(pass);
    }
    return true;
}

bool WorldRenderer::PrepareWater()
{
    if (!waterSampler)
    {
        SDL_GPUSamplerCreateInfo info{};
        info.min_filter = info.mag_filter = SDL_GPU_FILTER_LINEAR;
        info.address_mode_u = info.address_mode_v = info.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
        waterSampler = SDL_CreateGPUSampler(device, &info);
        if (!waterSampler) return false;
    }
    if (waterScene && waterWidth == viewport.w && waterHeight == viewport.h) return true;
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.format = sceneFormat;
    info.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;
    info.width = viewport.w;
    info.height = viewport.h;
    info.layer_count_or_depth = info.num_levels = 1;
    auto* replacement = SDL_CreateGPUTexture(device, &info);
    if (!replacement) return false;
    if (waterScene) SDL_ReleaseGPUTexture(device, waterScene);
    waterScene = replacement;
    waterWidth = viewport.w;
    waterHeight = viewport.h;
    return true;
}

void WorldRenderer::BindWater(SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, const SurfaceMaterial& material)
{
    const auto axis = [](Vector3 v, float w) { return SDL_FColor{float(v.x),float(v.y),float(v.z),w}; };
    const auto& water = material.water;
    const auto tint = Srgb::ToLinear({water.color[0],water.color[1],water.color[2],water.roughness});
    // Reduce phases in double before sending them to the GPU. A floating origin,
    // geographic coordinates and long sessions must not make ripples jump.
    constexpr std::array directions{SDL_FPoint{1,0}, SDL_FPoint{.6f,.8f}, SDL_FPoint{-.8f,.6f}, SDL_FPoint{.28f,-.96f}};
    constexpr std::array lengths{1.,.57,.23,.11}, speeds{1.,.9,1.1,.8};
    const auto eye = camera->GetPosition();
    const double time = double(SDL_GetTicksNS())/1e9;
    std::array<float,4> phase{};
    for (size_t i=0; i<4; ++i)
    {
        const double k = 2*std::numbers::pi/(water.waveScale*lengths[i]);
        phase[i] = float(std::remainder((eye.x*directions[i].x+eye.z*directions[i].y-time*water.waveSpeed*speeds[i])*k, 2*std::numbers::pi));
    }
    const SDL_FColor properties[]{tint,
        {water.waveStrength,water.waveScale,water.waveSpeed,water.clarity},
        {water.refraction,water.shoreWidth,water.foam,waterDepthReady ? 1.f : 0.f},
        axis(camera->Right(),float(scaleX)), axis(camera->Up(),float(scaleY)), axis(camera->Forward(),float(worldNearDepth)),
        {phase[0],phase[1],phase[2],phase[3]},
        {float(std::fmod(time*water.waveSpeed*.017,1)),float(std::fmod(time*water.waveSpeed*.013,1)),0,0}};
    SDL_PushGPUFragmentUniformData(commands,1,properties,sizeof(properties));
    const SDL_GPUTextureSamplerBinding inputs[]{{waterScene,waterSampler},{waterDepthReady ? ssaoDepth : ssaoWhite,ssaoSampler}};
    SDL_BindGPUFragmentSamplers(pass,3,inputs,2);
}

bool WorldRenderer::DrawGPU()
{
    // Submit SDL sky/previous viewport uses before sampling or cycling textures.
    if (!SDL_FlushRenderer(renderer)) return false;
    const auto isWater = [](const SurfaceMaterial* m) { return m && m->type == MaterialType::Water; };
    waterRequested = !residentWire && !wireframeEnd &&
        (std::ranges::any_of(draws,[&](const auto& d) { return isWater(d.material); }) ||
         std::ranges::any_of(residentDraws,[&](const auto& d) {
             return !d.uploadOnly && BoundsVisible(d.geometry->bounds) &&
                 std::ranges::any_of(d.geometry->parts,[&](const auto& p) { return isWater(p.material); });
         }));
    if (waterRequested && !PrepareWater()) return false;
    if (shadows.active && !PrepareShadowTarget(ShadowCascades::resolution))
    {
        SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Shadows disabled for this frame: %s", SDL_GetError());
        shadows.active = false;
    }
    // Stage newly visible or edited assets together. Once uploaded, their buffers
    // stay on the device. Only camera uniforms and dynamic geometry change each frame.
    constexpr size_t maxVertices = std::numeric_limits<Uint32>::max() / sizeof(WorldDepthVertex);
    if (vertices.size() > maxVertices) return SDL_SetError("Dynamic world vertex buffer exceeds GPU limit");
    const auto bytes = static_cast<Uint32>(vertices.size() * sizeof(WorldDepthVertex));
    struct Upload { ResidentGeometry* geometry; Uint32 offset, destination, size; };
    std::vector<Upload> uploads;
    Uint32 total = bytes;
    for (const auto& draw : residentDraws)
    {
        auto& mesh = *draw.geometry;
        if (!mesh.complete || mesh.pendingUpload.empty() || mesh.uploadFrame == frame) continue;
        if (mesh.pendingUpload.size() > maxVertices) return SDL_SetError("World mesh exceeds GPU buffer limit");
        const auto fullSize = static_cast<Uint32>(mesh.pendingUpload.size() * sizeof(WorldDepthVertex));
        // A large mesh uploads over multiple frames. Never bypass the burst limit,
        // and do not draw an incomplete buffer. Also cap small-buffer allocations.
        constexpr Uint32 burst = 4 * 1024 * 1024;
        if (uploads.size() >= 8 || total - bytes >= burst) continue;
        const auto size = std::min(fullSize - static_cast<Uint32>(mesh.uploadedBytes), burst - (total - bytes));
        if (size > std::numeric_limits<Uint32>::max() - total) return SDL_SetError("World upload exceeds GPU buffer limit");
        const SDL_GPUBufferCreateInfo info{SDL_GPU_BUFFERUSAGE_VERTEX, fullSize, 0};
        if (!mesh.buffer) mesh.buffer = SDL_CreateGPUBuffer(device, &info);
        if (!mesh.buffer) return false;
        mesh.bufferBytes = fullSize;
        mesh.uploadFrame = frame;
        mesh.uploadEnd = mesh.uploadedBytes + size;
        uploads.push_back({&mesh, total, static_cast<Uint32>(mesh.uploadedBytes), size});
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
    const auto spotData = BuildSpotLightData();
    const Uint32 spotBytes = static_cast<Uint32>(spotData.size() * sizeof(SDL_FColor));
    if (spotBytes > std::numeric_limits<Uint32>::max() - total) return SDL_SetError("World light upload exceeds GPU limit");
    const Uint32 spotOffset = total;
    total += spotBytes;
    if (spotBytes > spotBufferBytes)
    {
        const Uint32 capacity = grow(spotBytes, spotBufferBytes);
        const SDL_GPUBufferCreateInfo info{SDL_GPU_BUFFERUSAGE_GRAPHICS_STORAGE_READ, capacity, 0};
        auto* replacement = SDL_CreateGPUBuffer(device, &info);
        if (!replacement) return false;
        if (spotBuffer) SDL_ReleaseGPUBuffer(device, spotBuffer);
        spotBuffer = replacement;
        spotBufferBytes = capacity;
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
        std::memcpy(mapped + spotOffset, spotData.data(), spotBytes);
        for (const auto& item : uploads)
            std::memcpy(mapped + item.offset,
                reinterpret_cast<const Uint8*>(item.geometry->pendingUpload.data()) + item.destination, item.size);
        SDL_UnmapGPUTransferBuffer(device, upload);
    }

    SDL_GPUColorTargetInfo color{};
    auto* resolved = sceneColor;
    const bool foreground = foregroundStart < vertices.size();
    color.texture = multisampleColor ? multisampleColor : resolved;
    color.load_op = SDL_GPU_LOADOP_CLEAR; // Transparent pixels reveal the sky.
    color.store_op = multisampleColor ? (waterRequested ? SDL_GPU_STOREOP_RESOLVE_AND_STORE : SDL_GPU_STOREOP_RESOLVE) : SDL_GPU_STOREOP_STORE;
    color.resolve_texture = multisampleColor ? resolved : nullptr;
    color.cycle_resolve_texture = true;
    color.cycle = true;
    SDL_GPUDepthStencilTargetInfo depth{};
    depth.texture = depthTarget;
    depth.load_op = depth.stencil_load_op = SDL_GPU_LOADOP_CLEAR; // Reversed depth clears to zero.
    depth.store_op = depth.stencil_store_op = SDL_GPU_STOREOP_STORE; // Retain selection/depth for overlays after bloom.
    depth.cycle = true;

    if (!resolved)
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
        const auto send = [&](SDL_GPUBuffer* buffer, Uint32 offset, Uint32 destination, Uint32 size, bool cycle) {
            const SDL_GPUTransferBufferLocation src{upload, offset};
            const SDL_GPUBufferRegion dst{buffer, destination, size};
            SDL_UploadToGPUBuffer(copy, &src, &dst, cycle);
        };
        if (bytes) send(vertexBuffer, 0, 0, bytes, true);
        send(spotBuffer, spotOffset, 0, spotBytes, true);
        for (const auto& item : uploads) send(item.geometry->buffer, item.offset, item.destination, item.size, false);
        SDL_EndGPUCopyPass(copy);
    }

    if (!DrawShadows(commands) || !DrawSSAO(commands))
    {
        SDL_CancelGPUCommandBuffer(commands);
        return false;
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
    const SDL_FColor fogDensity = fog ? SDL_FColor{float(fog->density), float(fog->heightFalloff),
        float(camera->GetPosition().y - fog->baseHeight), float(fog->startDistance)} : SDL_FColor{};
    struct LightingData
    {
        SDL_FColor sunDirection, sunRadiance, moonDirection, moonRadiance, ambient;
        SDL_FColor fogDensity, fogAir, fogZenith, fogOpposite, fogSunward, fogSun;
        std::array<SDL_FColor, 4> shadowX, shadowY, shadowZ, shadowParams;
        SDL_FColor shadowControl, shadowForward, visibility, ssaoControl, spotControl;
    };
    static_assert(sizeof(LightingData) == 32*sizeof(SDL_FColor));
    LightingData lighting{pack(lights.sun.direction), pack(lights.sun.radiance), pack(lights.moon.direction),
                                pack(lights.moon.radiance), pack(lights.ambient), fogDensity,
                                {fog ? float(fog->airDensity) : 0.f, 0, 0, 0},
                                fog ? fog->zenith : SDL_FColor{}, fog ? fog->oppositeHorizon : SDL_FColor{},
                                fog ? fog->sunwardHorizon : SDL_FColor{}, fog ? pack(fog->horizontalSun) : SDL_FColor{}};
    if (shadows.active)
    {
        for (int i = 0; i < ShadowCascades::count; ++i)
        {
            const auto& cascade = shadows.cascades[i];
            const auto rows = cascade.Rows(camera->GetPosition());
            lighting.shadowX[i] = rows[0]; lighting.shadowY[i] = rows[1]; lighting.shadowZ[i] = rows[2];
            lighting.shadowParams[i] = {float(cascade.farDepth), float(cascade.texelMeters),
                float(std::max(.015, cascade.texelMeters*.15) / (2*cascade.depthRadius)), float(cascade.farDepth*.9)};
        }
        const double elevation = shadows.moon ? lights.moon.direction.y : lights.sun.direction.y;
        lighting.shadowControl = {4, 1.f/ShadowCascades::resolution, shadows.moon ? 1.f : 0.f,
                        float(std::clamp((elevation-.01)/.09, 0.0, 1.0))};
    }
    lighting.shadowForward = pack(camera->Forward());
    lighting.ssaoControl = {ssaoActive ? 1.f : 0.f, 1.f / std::max(1, ssaoWidth), 1.f / std::max(1, ssaoHeight), float(worldNearDepth)};
    lighting.visibility = {float(viewDistance * viewDistance), float(1 / focal), float(viewport.w) * .5f, float(viewport.h) * .5f};
    lighting.spotControl = {float((viewport.w + lightTilePixels - 1) / lightTilePixels), float(lightTilePixels),
                            float(lights.spots.size()), float((lights.spots.size() + 127) / 128)};
    const SDL_GPUTextureSamplerBinding sceneBindings[]{{shadowTarget, shadowSampler}, {ssaoActive ? ssaoResult : ssaoWhite, ssaoSampler}};
    SDL_BindGPUFragmentSamplers(pass, 0, sceneBindings, 2);
    SDL_BindGPUFragmentStorageBuffers(pass, 0, &spotBuffer, 1);
    SDL_PushGPUFragmentUniformData(commands, 0, &lighting, sizeof(lighting));
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
        else if (material) pipeline = material->type == MaterialType::Water ? waterPipeline :
                                      material->type == MaterialType::Unlit ? translucentPipeline :
                                      material->type == MaterialType::Foliage ? foliagePipeline :
                                      material->type == MaterialType::AlphaCutout ? cutoutPipeline : texturedPipeline;
        if (pipeline != bound)
        {
            SDL_BindGPUGraphicsPipeline(pass, pipeline);
            bound = pipeline;
        }
        if (material && material != boundMaterial)
        {
            materialCache.Bind(*material, commands, pass, 2);
            if (isWater(material)) BindWater(commands, pass, *material);
        }
        boundMaterial = material;
    };
    bool overlays = false;
    int phase = 0; // Opaque scene, then water, then ordinary translucency.
    const auto accepted = [&](const SurfaceMaterial* material) {
        if (overlays || !waterRequested) return true;
        const int kind = isWater(material) ? 1 : (material && material->type == MaterialType::Unlit ? 2 : 0);
        return kind == phase;
    };
    const auto drawResident = [&](const ResidentDraw& batch) {
        // Shadow-only submissions must not bind a water pass that was skipped
        // because no water is visible in the main camera.
        if (batch.uploadOnly || !BoundsVisible(batch.geometry->bounds) ||
            (std::size_t(&batch - residentDraws.data()) >= overlayResidentStart) != overlays) return;
        const auto& mesh = *batch.geometry;
        if (!mesh.buffer || (mesh.uploadedBytes < mesh.bufferBytes &&
            !(mesh.uploadFrame == frame && mesh.uploadEnd == mesh.bufferBytes))) return;
        bindSource(batch.geometry->buffer, batch.geometry.get());
        for (const auto& part : batch.geometry->parts)
        {
            if (!accepted(part.material)) continue;
            bindPipeline(residentWire ? nullptr : part.material, residentWire);
            draw(part.first, part.end, batch.selected);
        }
    };
    const auto drawRange = [&](std::size_t first, std::size_t end, Uint8 marker) {
        if (overlays) first = std::max(first, std::min(shadowEnd, outlineStart));
        else end = std::min(end, shadowEnd);
        draw(first, end, marker);
    };
    const auto drawScene = [&] {
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
                if (first < split && accepted(range.material))
                {
                    bindSource(vertexBuffer, nullptr);
                    bindPipeline(range.material, first < wireframeEnd);
                    drawRange(first, split, range.selected);
                    first = split;
                }
                drawResident(*batch++);
            }
            if (accepted(range.material))
            {
                bindSource(vertexBuffer, nullptr);
                bindPipeline(range.material, first < wireframeEnd);
                drawRange(first, range.end, range.selected);
            }
        }
        while (batch != residentDraws.end()) drawResident(*batch++);
    };
    drawScene();
    if (waterRequested)
    {
        SDL_EndGPURenderPass(pass);
        auto* copy = SDL_BeginGPUCopyPass(commands);
        if (!copy) { SDL_CancelGPUCommandBuffer(commands); return false; }
        const SDL_GPUTextureLocation src{resolved,0,0,0,0,0}, dst{waterScene,0,0,0,0,0};
        SDL_CopyGPUTextureToTexture(copy,&src,&dst,viewport.w,viewport.h,1,true);
        SDL_EndGPUCopyPass(copy);
        color.load_op = SDL_GPU_LOADOP_LOAD;
        color.cycle = color.cycle_resolve_texture = false;
        color.store_op = multisampleColor ? SDL_GPU_STOREOP_RESOLVE : SDL_GPU_STOREOP_STORE;
        color.resolve_texture = multisampleColor ? resolved : nullptr;
        depth.load_op = depth.stencil_load_op = SDL_GPU_LOADOP_LOAD;
        depth.cycle = false;
        depth.store_op = depth.stencil_store_op = SDL_GPU_STOREOP_STORE;
        pass = SDL_BeginGPURenderPass(commands,&color,1,&depth);
        if (!pass) { SDL_CancelGPUCommandBuffer(commands); return false; }
        SDL_BindGPUFragmentSamplers(pass,0,sceneBindings,2);
        SDL_BindGPUFragmentStorageBuffers(pass,0,&spotBuffer,1);
        SDL_PushGPUFragmentUniformData(commands,0,&lighting,sizeof(lighting));
        source = nullptr; bound = nullptr; boundMaterial = nullptr;
        phase = 1; drawScene();
        phase = 2; drawScene();
    }

    SDL_EndGPURenderPass(pass);
    if (!DrawPostProcessing(commands)) { SDL_CancelGPUCommandBuffer(commands); return false; }

    // The tone-mapped image is still linear. Draw overlays using the original
    // depth/stencil and sample count, then perform display encoding exactly once.
    overlays = true;
    color.load_op = SDL_GPU_LOADOP_LOAD;
    color.cycle = color.cycle_resolve_texture = false;
    color.store_op = multisampleColor ? (foreground ? SDL_GPU_STOREOP_STORE : SDL_GPU_STOREOP_RESOLVE) : SDL_GPU_STOREOP_STORE;
    color.resolve_texture = multisampleColor && !foreground ? resolved : nullptr;
    depth.load_op = depth.stencil_load_op = SDL_GPU_LOADOP_LOAD;
    depth.store_op = depth.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth.cycle = false;
    pass = SDL_BeginGPURenderPass(commands, &color, 1, &depth);
    if (!pass) { SDL_CancelGPUCommandBuffer(commands); return false; }
    SDL_BindGPUFragmentSamplers(pass, 0, sceneBindings, 2);
    SDL_BindGPUFragmentStorageBuffers(pass, 0, &spotBuffer, 1);
    lighting.spotControl = {}; lighting.ssaoControl = {}; lighting.visibility = {};
    SDL_PushGPUFragmentUniformData(commands, 0, &lighting, sizeof(lighting));
    source = nullptr; bound = nullptr; boundMaterial = nullptr;
    drawScene();

    if (outlineStart < foregroundStart)
    {
        bindSource(vertexBuffer, nullptr);
        SDL_BindGPUGraphicsPipeline(pass, outlinePipeline);
        draw(outlineStart, foregroundStart, 1);
    }

    SDL_EndGPURenderPass(pass);
    if (foreground)
    {
        // Keep the scene color and start a fresh depth buffer for the gizmo.
        color.load_op = SDL_GPU_LOADOP_LOAD;
        color.cycle = false;
        depth.load_op = depth.stencil_load_op = SDL_GPU_LOADOP_CLEAR;
        // Retain the multisample scene through the gizmo pass and resolve once.
        if (multisampleColor)
        {
            color.store_op = SDL_GPU_STOREOP_RESOLVE;
            color.resolve_texture = resolved;
        }
        pass = SDL_BeginGPURenderPass(commands, &color, 1, &depth);
        if (!pass)
        {
            SDL_CancelGPUCommandBuffer(commands);
            return false;
        }
        SDL_BindGPUGraphicsPipeline(pass, surfacePipeline);
        SDL_BindGPUFragmentSamplers(pass, 0, sceneBindings, 2);
        SDL_BindGPUFragmentStorageBuffers(pass, 0, &spotBuffer, 1);
        lighting.spotControl = {};
        lighting.ssaoControl = {};
        lighting.visibility = {}; // Editor foreground handles remain visible at any distance.
        SDL_PushGPUFragmentUniformData(commands, 0, &lighting, sizeof(lighting));
        source = nullptr; // Bindings are local to a render pass.
        bindSource(vertexBuffer, nullptr);
        draw(foregroundStart, vertices.size(), 0);
        SDL_EndGPURenderPass(pass);
    }
    if (!EncodeDisplay(commands)) { SDL_CancelGPUCommandBuffer(commands); return false; }
    if (!SDL_SubmitGPUCommandBuffer(commands)) return false;
    shadowInitialized = ssaoInitialized = true;
    for (const auto& item : uploads)
    {
        auto& mesh = *item.geometry;
        mesh.uploadedBytes = mesh.uploadEnd;
        uploadedBytes += item.size;
        if (mesh.uploadedBytes == mesh.bufferBytes) std::vector<WorldDepthVertex>().swap(mesh.pendingUpload);
    }
    return true;
}
