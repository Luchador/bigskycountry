#include "world_renderer.hpp"
#include "camera_view.hpp"
#include "runtime_config.hpp"
#include <algorithm>
#include <span>

#ifdef BIGSKY_WORLD_GPU
#include "world_vert_dxil.hpp"
#include "world_vert_spirv.hpp"
#include "ssao_depth_dxil.hpp"
#include "ssao_depth_spirv.hpp"
#include "ssao_cutout_dxil.hpp"
#include "ssao_cutout_spirv.hpp"
#include "ssao_vert_dxil.hpp"
#include "ssao_vert_spirv.hpp"
#include "ssao_frag_dxil.hpp"
#include "ssao_frag_spirv.hpp"
#include "ssao_blur_dxil.hpp"
#include "ssao_blur_spirv.hpp"

namespace
{
    SDL_GPUShader* Shader(SDL_GPUDevice* device, SDL_GPUShaderStage stage, const char* entry,
                          std::span<const Uint8> dxil, std::span<const Uint8> spirv, Uint32 samplers, Uint32 uniforms)
    {
        SDL_GPUShaderCreateInfo info{};
        const bool vulkan = (SDL_GetGPUShaderFormats(device) & SDL_GPU_SHADERFORMAT_SPIRV) != 0;
        info.format = vulkan ? SDL_GPU_SHADERFORMAT_SPIRV : SDL_GPU_SHADERFORMAT_DXIL;
        const auto bytes = vulkan ? spirv : dxil;
        info.code = bytes.data();
        info.code_size = bytes.size();
        info.stage = stage;
        info.entrypoint = entry;
        info.num_samplers = samplers;
        info.num_uniform_buffers = uniforms;
        return SDL_CreateGPUShader(device, &info);
    }
}
#endif

void WorldRenderer::ReleaseSSAO()
{
    for (auto* texture : {ssaoWhite, ssaoDepth, ssaoResult, ssaoTemp})
        if (texture) SDL_ReleaseGPUTexture(device, texture);
    for (auto* pipeline : {ssaoDepthPipeline, ssaoCutoutPipeline, ssaoFoliagePipeline, ssaoPipeline, ssaoBlurPipeline})
        if (pipeline) SDL_ReleaseGPUGraphicsPipeline(device, pipeline);
    if (ssaoSampler) SDL_ReleaseGPUSampler(device, ssaoSampler);
    ssaoWhite = ssaoDepth = ssaoResult = ssaoTemp = nullptr;
    ssaoDepthPipeline = ssaoCutoutPipeline = ssaoFoliagePipeline = ssaoPipeline = ssaoBlurPipeline = nullptr;
    ssaoSampler = nullptr;
    ssaoInitialized = false;
}

bool WorldRenderer::CreateSSAOFallback()
{
    // Every scene shader has a valid binding even when SSAO is disabled. No
    // full-sized targets or extra scene passes are needed in that case.
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
    info.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    info.width = info.height = info.layer_count_or_depth = info.num_levels = 1;
    ssaoWhite = SDL_CreateGPUTexture(device, &info);
    SDL_GPUSamplerCreateInfo sampler{};
    sampler.min_filter = sampler.mag_filter = SDL_GPU_FILTER_NEAREST;
    sampler.address_mode_u = sampler.address_mode_v = sampler.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    ssaoSampler = SDL_CreateGPUSampler(device, &sampler);
    if (ssaoWhite && ssaoSampler) return true;
    ReleaseSSAO();
    return false;
}

bool WorldRenderer::CreateSSAOPipelines()
{
#ifdef BIGSKY_WORLD_GPU
    auto* vs = Shader(device, SDL_GPU_SHADERSTAGE_VERTEX, "VSMain", world_vert_dxil, world_vert_spirv, 0, 1);
    auto* ps = Shader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, "DepthPS", ssao_depth_dxil, ssao_depth_spirv, 0, 1);
    auto* cutout = Shader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, "CutoutPS", ssao_cutout_dxil, ssao_cutout_spirv, 2, 1);
    auto* screen = Shader(device, SDL_GPU_SHADERSTAGE_VERTEX, "FullscreenVS", ssao_vert_dxil, ssao_vert_spirv, 0, 0);
    auto* occlusion = Shader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, "OcclusionPS", ssao_frag_dxil, ssao_frag_spirv, 1, 1);
    auto* blur = Shader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, "BlurPS", ssao_blur_dxil, ssao_blur_spirv, 1, 1);
    if (vs && ps && cutout && screen && occlusion && blur)
    {
        SDL_GPUVertexBufferDescription layout{0, sizeof(WorldDepthVertex), SDL_GPU_VERTEXINPUTRATE_VERTEX, 0};
        SDL_GPUVertexAttribute attributes[] = {
            {0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, 0},
            {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(WorldDepthVertex, color)},
            {2, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, offsetof(WorldDepthVertex, uv)},
            {3, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(WorldDepthVertex, normal)},
            {4, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(WorldDepthVertex, tangent)},
            {5, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, offsetof(WorldDepthVertex, view)}
        };
        SDL_GPUGraphicsPipelineCreateInfo pipeline{};
        pipeline.vertex_shader = vs;
        pipeline.fragment_shader = ps;
        pipeline.vertex_input_state = {&layout, 1, attributes, SDL_arraysize(attributes)};
        pipeline.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
        pipeline.target_info.has_depth_stencil_target = true;
        pipeline.target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
        pipeline.rasterizer_state.front_face = SDL_GPU_FRONTFACE_CLOCKWISE;
        pipeline.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_BACK;
        pipeline.rasterizer_state.enable_depth_clip = true;
        pipeline.depth_stencil_state.enable_depth_test = pipeline.depth_stencil_state.enable_depth_write = true;
        pipeline.depth_stencil_state.compare_op = SDL_GPU_COMPAREOP_GREATER_OR_EQUAL;
        ssaoDepthPipeline = SDL_CreateGPUGraphicsPipeline(device, &pipeline);
        pipeline.fragment_shader = cutout;
        ssaoCutoutPipeline = SDL_CreateGPUGraphicsPipeline(device, &pipeline);
        pipeline.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
        ssaoFoliagePipeline = SDL_CreateGPUGraphicsPipeline(device, &pipeline);

        SDL_GPUColorTargetDescription color{};
        color.format = SDL_GPU_TEXTUREFORMAT_R16G16_FLOAT;
        pipeline = {};
        pipeline.rasterizer_state.enable_depth_clip = true;
        pipeline.vertex_shader = screen;
        pipeline.fragment_shader = occlusion;
        pipeline.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
        pipeline.target_info.color_target_descriptions = &color;
        pipeline.target_info.num_color_targets = 1;
        ssaoPipeline = SDL_CreateGPUGraphicsPipeline(device, &pipeline);
        pipeline.fragment_shader = blur;
        ssaoBlurPipeline = SDL_CreateGPUGraphicsPipeline(device, &pipeline);
    }
    for (auto* shader : {vs, ps, cutout, screen, occlusion, blur})
        if (shader) SDL_ReleaseGPUShader(device, shader);
    return ssaoDepthPipeline && ssaoCutoutPipeline && ssaoFoliagePipeline && ssaoPipeline && ssaoBlurPipeline;
#else
    return false;
#endif
}

bool WorldRenderer::PrepareSSAO()
{
    if (!ssaoPipeline)
    {
        if (!SDL_GPUTextureSupportsFormat(device, SDL_GPU_TEXTUREFORMAT_D32_FLOAT, SDL_GPU_TEXTURETYPE_2D,
                SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER) ||
            !SDL_GPUTextureSupportsFormat(device, SDL_GPU_TEXTUREFORMAT_R16G16_FLOAT, SDL_GPU_TEXTURETYPE_2D,
                SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER))
            return SDL_SetError("SSAO depth/color formats are unsupported");
        if (!CreateSSAOPipelines()) return false;
    }
    // Full camera depth preserves thin foliage; the expensive AO/filter passes
    // run at half resolution. Odd window sizes round up, never to zero.
    const int width = (viewport.w+1)/2, height = (viewport.h+1)/2;
    if (ssaoDepth && ssaoFullWidth == viewport.w && ssaoFullHeight == viewport.h) return true;
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.format = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    info.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    info.width = viewport.w;
    info.height = viewport.h;
    info.layer_count_or_depth = info.num_levels = 1;
    auto* depth = SDL_CreateGPUTexture(device, &info);
    info.width = width;
    info.height = height;
    info.format = SDL_GPU_TEXTUREFORMAT_R16G16_FLOAT;
    info.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    auto* result = SDL_CreateGPUTexture(device, &info);
    auto* temp = SDL_CreateGPUTexture(device, &info);
    if (!depth || !result || !temp)
    {
        for (auto* texture : {depth,result,temp}) if (texture) SDL_ReleaseGPUTexture(device, texture);
        return false;
    }
    for (auto* texture : {ssaoDepth,ssaoResult,ssaoTemp}) if (texture) SDL_ReleaseGPUTexture(device, texture);
    ssaoDepth = depth;
    ssaoResult = result;
    ssaoTemp = temp;
    ssaoWidth = width;
    ssaoHeight = height;
    ssaoFullWidth = viewport.w;
    ssaoFullHeight = viewport.h;
    return true;
}

bool WorldRenderer::DrawSSAO(SDL_GPUCommandBuffer* commands)
{
    ssaoActive = false;
    if (!ssaoInitialized)
    {
        SDL_GPUColorTargetInfo white{};
        white.texture = ssaoWhite;
        white.clear_color = {1,0,0,1};
        white.load_op = SDL_GPU_LOADOP_CLEAR;
        white.store_op = SDL_GPU_STOREOP_STORE;
        auto* pass = SDL_BeginGPURenderPass(commands, &white, 1, nullptr);
        if (!pass) return false;
        SDL_EndGPURenderPass(pass);
        // Set initialized only after successful command-buffer submission.
    }
    if (!RuntimeConfig::Get().ssao || !ssaoRequested || ssaoFailed || residentWire || wireframeEnd) return true;
    if (!PrepareSSAO())
    {
        SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "SSAO disabled: %s", SDL_GetError());
        ssaoFailed = true;
        return true;
    }
    SDL_GPUDepthStencilTargetInfo depth{};
    depth.texture = ssaoDepth;
    depth.load_op = SDL_GPU_LOADOP_CLEAR; // Infinite reversed depth, empty pixels = 0.
    depth.store_op = SDL_GPU_STOREOP_STORE;
    depth.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
    depth.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth.cycle = true;
    auto* pass = SDL_BeginGPURenderPass(commands, nullptr, 0, &depth);
    if (!pass) return false;
    constexpr double farDistance = 152; // Fade ends at 150 m, plus 2 m sampling radius.
    const double distance = viewDistance > 0 ? std::min(viewDistance, farDistance) : farDistance;
    const SDL_FColor limits{float(distance*distance),0,0,0};
    SDL_PushGPUFragmentUniformData(commands, 0, &limits, sizeof(limits));
    const auto axis = [](Vector3 v, float w = 0) { return SDL_FColor{float(v.x),float(v.y),float(v.z),w}; };
    SDL_FColor cameraData[]{axis(camera->Right()*scaleX), axis(camera->Up()*scaleY),
                            axis(camera->Forward(),float(worldNearDepth)), {}, {}, {}};
    const auto bind = [&](SDL_GPUBuffer* buffer, const ResidentGeometry* mesh) {
        const SDL_GPUBufferBinding binding{buffer,0};
        SDL_BindGPUVertexBuffers(pass,0,&binding,1);
        const Vector3 offset = mesh ? mesh->origin-camera->GetPosition() : Vector3{};
        cameraData[3] = axis(offset,mesh ? 1.f : 0.f);
        cameraData[4] = axis(offset-Vector3{cameraData[3].r,cameraData[3].g,cameraData[3].b});
        SDL_PushGPUVertexUniformData(commands,0,cameraData,sizeof(cameraData));
    };
    const auto draw = [&](size_t first, size_t end, const SurfaceMaterial* material) {
        if (first >= end || (material && material->type == MaterialType::Unlit)) return;
        const bool cutout = material && Material::IsCutout(material->type);
        auto* pipeline = !cutout ? ssaoDepthPipeline : material->type == MaterialType::Foliage ? ssaoFoliagePipeline : ssaoCutoutPipeline;
        SDL_BindGPUGraphicsPipeline(pass,pipeline);
        if (cutout) materialCache.BindShadow(*material,pass);
        SDL_DrawGPUPrimitives(pass,static_cast<Uint32>(end-first),1,static_cast<Uint32>(first),0);
    };
    for (const auto& batch : residentDraws)
    {
        if (batch.uploadOnly) continue;
        const auto& mesh = *batch.geometry;
        if (!mesh.buffer || (mesh.uploadedBytes < mesh.bufferBytes &&
            !(mesh.uploadFrame == frame && mesh.uploadEnd == mesh.bufferBytes)) || !BoundsVisible(mesh.bounds)) continue;
        if (!mesh.bounds.Empty())
        {
            const auto eye = camera->GetPosition();
            const Vector3 nearest{std::clamp(eye.x,mesh.bounds.min.x,mesh.bounds.max.x),
                                  std::clamp(eye.y,mesh.bounds.min.y,mesh.bounds.max.y),
                                  std::clamp(eye.z,mesh.bounds.min.z,mesh.bounds.max.z)};
            if (Math::Dot(nearest-eye,nearest-eye) > distance*distance) continue;
        }
        bind(mesh.buffer,&mesh);
        for (const auto& part : mesh.parts) draw(part.first,part.end,part.material);
    }
    const size_t end = std::min({shadowEnd,outlineStart,foregroundStart});
    if (vertexBuffer && end)
    {
        bind(vertexBuffer,nullptr);
        for (const auto& range : draws) draw(range.first,std::min(range.end,end),range.material);
    }
    SDL_EndGPURenderPass(pass);

    struct Settings { SDL_FColor projection, textureSize, settings, blurStep; };
    Settings settings{{float(1/scaleX),float(1/scaleY),float(worldNearDepth),2},
                      {1.f/viewport.w,1.f/viewport.h,float(ssaoWidth),float(ssaoHeight)},
                      {float(RuntimeConfig::Get().ssaoSamples),.05f,100,150}, {}};
    const auto screenPass = [&](SDL_GPUTexture* source, SDL_GPUTexture* target, SDL_GPUGraphicsPipeline* pipeline) {
        SDL_GPUColorTargetInfo color{};
        color.texture = target;
        color.load_op = SDL_GPU_LOADOP_DONT_CARE;
        color.store_op = SDL_GPU_STOREOP_STORE;
        color.cycle = true;
        auto* screen = SDL_BeginGPURenderPass(commands,&color,1,nullptr);
        if (!screen) return false;
        SDL_BindGPUGraphicsPipeline(screen,pipeline);
        const SDL_GPUTextureSamplerBinding binding{source,ssaoSampler};
        SDL_BindGPUFragmentSamplers(screen,0,&binding,1);
        SDL_PushGPUFragmentUniformData(commands,0,&settings,sizeof(settings));
        SDL_DrawGPUPrimitives(screen,3,1,0,0);
        SDL_EndGPURenderPass(screen);
        return true;
    };
    if (!screenPass(ssaoDepth,ssaoResult,ssaoPipeline)) return false;
    settings.blurStep = {1.f/ssaoWidth,0,0,0};
    if (!screenPass(ssaoResult,ssaoTemp,ssaoBlurPipeline)) return false;
    settings.blurStep = {0,1.f/ssaoHeight,0,0};
    if (!screenPass(ssaoTemp,ssaoResult,ssaoBlurPipeline)) return false;
    ssaoActive = true;
    return true;
}
