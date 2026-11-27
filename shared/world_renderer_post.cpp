#include "world_renderer.hpp"
#include "runtime_config.hpp"
#include <algorithm>
#include <span>

#ifdef BIGSKY_WORLD_GPU
#include "post_vert_dxil.hpp"
#include "post_vert_spirv.hpp"
#include "post_compose_dxil.hpp"
#include "post_compose_spirv.hpp"
#include "bloom_down_dxil.hpp"
#include "bloom_down_spirv.hpp"
#include "bloom_up_dxil.hpp"
#include "bloom_up_spirv.hpp"
#include "post_tone_dxil.hpp"
#include "post_tone_spirv.hpp"
#include "post_encode_dxil.hpp"
#include "post_encode_spirv.hpp"

namespace
{
    SDL_GPUShader* Shader(SDL_GPUDevice* device, SDL_GPUShaderStage stage, const char* entry,
                          std::span<const Uint8> dxil, std::span<const Uint8> spirv, Uint32 samplers, Uint32 uniforms)
    {
        const bool vulkan = (SDL_GetGPUShaderFormats(device) & SDL_GPU_SHADERFORMAT_SPIRV) != 0;
        const auto code = vulkan ? spirv : dxil;
        SDL_GPUShaderCreateInfo info{};
        info.stage = stage; info.entrypoint = entry;
        info.format = vulkan ? SDL_GPU_SHADERFORMAT_SPIRV : SDL_GPU_SHADERFORMAT_DXIL;
        info.code = code.data(); info.code_size = code.size();
        info.num_samplers = samplers; info.num_uniform_buffers = uniforms;
        return SDL_CreateGPUShader(device, &info);
    }
    struct PostSettings
    {
        SDL_FColor filter{}, settings{}, background{};
    };
    bool Pass(SDL_GPUCommandBuffer* commands, SDL_GPUGraphicsPipeline* pipeline, SDL_GPUTexture* output,
              SDL_GPUSampler* sampler, SDL_GPUTexture* source, SDL_GPUTexture* second, const PostSettings& settings)
    {
        SDL_GPUColorTargetInfo target{};
        target.texture = output;
        target.load_op = SDL_GPU_LOADOP_DONT_CARE;
        target.store_op = SDL_GPU_STOREOP_STORE;
        target.cycle = true;
        auto* pass = SDL_BeginGPURenderPass(commands, &target, 1, nullptr);
        if (!pass) return false;
        SDL_BindGPUGraphicsPipeline(pass, pipeline);
        SDL_PushGPUFragmentUniformData(commands, 0, &settings, sizeof(settings));
        const SDL_GPUTextureSamplerBinding bindings[]{{source, sampler}, {second, sampler}};
        SDL_BindGPUFragmentSamplers(pass, 0, bindings, second ? 2 : 1);
        SDL_DrawGPUPrimitives(pass, 3, 1, 0, 0);
        SDL_EndGPURenderPass(pass);
        return true;
    }
}

bool WorldRenderer::CreatePostPipelines()
{
    auto* vs = Shader(device, SDL_GPU_SHADERSTAGE_VERTEX, "FullscreenVS", post_vert_dxil, post_vert_spirv, 0, 0);
    if (!vs) return false;
    const auto create = [&](const char* entry, std::span<const Uint8> dxil, std::span<const Uint8> spirv,
                            Uint32 samplers, SDL_GPUTextureFormat format, SDL_GPUSampleCount samples) {
        auto* ps = Shader(device, SDL_GPU_SHADERSTAGE_FRAGMENT, entry, dxil, spirv, samplers, 1);
        if (!ps) return static_cast<SDL_GPUGraphicsPipeline*>(nullptr);
        SDL_GPUColorTargetDescription color{}; color.format = format;
        SDL_GPUGraphicsPipelineCreateInfo info{};
        info.vertex_shader = vs; info.fragment_shader = ps;
        info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
        info.target_info.color_target_descriptions = &color; info.target_info.num_color_targets = 1;
        info.multisample_state.sample_count = samples;
        auto* result = SDL_CreateGPUGraphicsPipeline(device, &info);
        SDL_ReleaseGPUShader(device, ps);
        return result;
    };
    composePipeline = create("ComposePS", post_compose_dxil, post_compose_spirv, 2, sceneFormat, SDL_GPU_SAMPLECOUNT_1);
    downsamplePipeline = create("DownsamplePS", bloom_down_dxil, bloom_down_spirv, 1, sceneFormat, SDL_GPU_SAMPLECOUNT_1);
    upsamplePipeline = create("UpsamplePS", bloom_up_dxil, bloom_up_spirv, 2, sceneFormat, SDL_GPU_SAMPLECOUNT_1);
    tonePipeline = create("TonePS", post_tone_dxil, post_tone_spirv, 2, sceneFormat, sampleCount);
    encodePipeline = create("EncodePS", post_encode_dxil, post_encode_spirv, 1,
                            SDL_GetGPUTextureFormatFromPixelFormat(SDL_PIXELFORMAT_RGBA32), SDL_GPU_SAMPLECOUNT_1);
    SDL_ReleaseGPUShader(device, vs);
    SDL_GPUSamplerCreateInfo sampler{};
    sampler.min_filter = sampler.mag_filter = SDL_GPU_FILTER_LINEAR;
    sampler.address_mode_u = sampler.address_mode_v = sampler.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    postSampler = SDL_CreateGPUSampler(device, &sampler);
    return composePipeline && downsamplePipeline && upsamplePipeline && tonePipeline && encodePipeline && postSampler;
}

bool WorldRenderer::PreparePostTargets(int width, int height)
{
    ReleasePostTargets();
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D; info.format = sceneFormat;
    info.usage = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET | SDL_GPU_TEXTUREUSAGE_SAMPLER;
    info.width = width; info.height = height;
    info.layer_count_or_depth = info.num_levels = 1;
    sceneColor = SDL_CreateGPUTexture(device, &info);
    postScene = SDL_CreateGPUTexture(device, &info);
    bool valid = sceneColor && postScene;
    const auto& config = RuntimeConfig::Get();
    if (valid && config.bloom && config.bloomIntensity > 0)
    {
        // Ceil division retains the last row/column of odd-sized viewports.
        for (int i = 0; i < 6 && (width > 1 || height > 1); ++i)
        {
            width = std::max(1, (width + 1) / 2); height = std::max(1, (height + 1) / 2);
            info.width = width; info.height = height;
            BloomLevel level{SDL_CreateGPUTexture(device, &info), SDL_CreateGPUTexture(device, &info), width, height};
            bloomLevels.push_back(level);
            if (!level.down || !level.up) { valid = false; break; }
        }
    }
    if (!valid) ReleasePostTargets();
    return valid;
}

bool WorldRenderer::DrawPostProcessing(SDL_GPUCommandBuffer* commands)
{
    const auto& config = RuntimeConfig::Get();
    auto* sky = skyBackground ? static_cast<SDL_GPUTexture*>(SDL_GetPointerProperty(SDL_GetTextureProperties(skyBackground),
                SDL_PROP_TEXTURE_GPU_TEXTURE_POINTER, nullptr)) : nullptr;
    const bool bloom = postProcessing && config.bloom && config.bloomIntensity > 0 && !bloomLevels.empty() && !residentWire && !wireframeEnd;
    PostSettings uniforms{{}, {bloom ? config.bloomIntensity : 0, config.bloomThreshold, postProcessing ? 1.f : 0.f, sky ? 1.f : 0.f}, backgroundColor};
    if (!Pass(commands, composePipeline, postScene, postSampler, sceneColor, sky ? sky : sceneColor, uniforms)) return false;
    SDL_GPUTexture* glow = postScene; // Valid binding even when bloom is off; never sampled then.
    if (bloom)
    {
        SDL_GPUTexture* source = postScene;
        int width = viewport.w, height = viewport.h;
        for (size_t i = 0; i < bloomLevels.size(); ++i)
        {
            const auto& level = bloomLevels[i];
            uniforms.filter = {1.f/width, 1.f/height, config.bloomRadius, i == 0 ? 1.f : 0.f};
            if (!Pass(commands, downsamplePipeline, level.down, postSampler, source, nullptr, uniforms)) return false;
            source = level.down; width = level.width; height = level.height;
        }
        for (size_t i = bloomLevels.size() - 1; i > 0; --i)
        {
            const auto& coarse = bloomLevels[i];
            const auto& fine = bloomLevels[i-1];
            uniforms.filter = {1.f/coarse.width, 1.f/coarse.height, config.bloomRadius, 0};
            if (!Pass(commands, upsamplePipeline, fine.up, postSampler, source, fine.down, uniforms)) return false;
            source = fine.up;
        }
        glow = source;
    }
    // Reuse the MSAA attachment now that the complete HDR scene is captured.
    // Tone mapping precedes all labels, selection outlines and editor gizmos.
    return Pass(commands, tonePipeline, multisampleColor ? multisampleColor : sceneColor,
                postSampler, postScene, glow, uniforms);
}

bool WorldRenderer::EncodeDisplay(SDL_GPUCommandBuffer* commands)
{
    auto* output = static_cast<SDL_GPUTexture*>(SDL_GetPointerProperty(SDL_GetTextureProperties(colorTarget),
                    SDL_PROP_TEXTURE_GPU_TEXTURE_POINTER, nullptr));
    return output && Pass(commands, encodePipeline, output, postSampler, sceneColor, nullptr, {});
}
#else
bool WorldRenderer::CreatePostPipelines() { return false; }
bool WorldRenderer::PreparePostTargets(int, int) { return false; }
bool WorldRenderer::DrawPostProcessing(SDL_GPUCommandBuffer*) { return false; }
bool WorldRenderer::EncodeDisplay(SDL_GPUCommandBuffer*) { return false; }
#endif

void WorldRenderer::ReleasePostTargets()
{
    for (const auto& level : bloomLevels)
    {
        if (level.down) SDL_ReleaseGPUTexture(device, level.down);
        if (level.up) SDL_ReleaseGPUTexture(device, level.up);
    }
    bloomLevels.clear();
    if (sceneColor) SDL_ReleaseGPUTexture(device, sceneColor);
    if (postScene) SDL_ReleaseGPUTexture(device, postScene);
    sceneColor = postScene = nullptr;
}

void WorldRenderer::ReleasePostProcessing()
{
    ReleasePostTargets();
    for (auto* pipeline : {composePipeline, downsamplePipeline, upsamplePipeline, tonePipeline, encodePipeline})
        if (pipeline) SDL_ReleaseGPUGraphicsPipeline(device, pipeline);
    composePipeline = downsamplePipeline = upsamplePipeline = tonePipeline = encodePipeline = nullptr;
    if (postSampler) SDL_ReleaseGPUSampler(device, postSampler);
    postSampler = nullptr;
}
