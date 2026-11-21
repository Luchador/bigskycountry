#include "stars.hpp"
#include "camera_view.hpp"
#include "sky_location.hpp"
#include "texture_loader.hpp"
#include "dds_texture.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

#ifdef BIGSKY_WORLD_GPU
#include "stars_vert_dxil.hpp"
#include "stars_vert_spirv.hpp"
#include "stars_frag_dxil.hpp"
#include "stars_frag_spirv.hpp"
#endif

namespace
{
    constexpr StarSky::Star catalog[]{
#include "data/hyg_v41_stars.inc"
    };
    double Smooth(double value)
    {
        const double t = std::clamp(value, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }
    float SpriteScale(SDL_Rect viewport)
    {
        // 0.5f to scale to half size.
        return 0.5f * std::clamp(std::min(viewport.w, viewport.h) / 1080.0f, 0.75f, 2.0f);
    }
}

std::span<const StarSky::Star> StarSky::Catalog() { return catalog; }

Vector3 StarSky::Frame::ToWorld(Vector3 p) const
{
    return {Math::Dot(east, p), Math::Dot(up, p), Math::Dot(north, p)};
}

StarSky::Frame StarSky::At(double minutes)
{
    minutes = std::fmod(minutes, 1440.0);
    if (minutes < 0) minutes += 1440;
    // The game repeats one calendar day. Reset that date at noon, when stars
    // are invisible, so their sidereal motion remains continuous at midnight.
    if (minutes < 720) minutes += 1440;
    const double hoursUTC = minutes / 60.0 - SkyLocation::utcOffsetHours;
    // USNO approximate GMST, https://aa.usno.navy.mil/faq/GAST.
    const double days = SkyLocation::julianDate - 2451545.0;
    const double centuries = (days + hoursUTC / 24.0) / 36525.0;
    const double gmst = 6.697375 + 0.065709824279 * days + 1.0027379 * hoursUTC +
                        0.0000258 * centuries * centuries;
    const double angle = Math::DegreesToRadians(std::fmod(gmst * 15.0 + SkyLocation::longitudeDegrees, 360.0));
    const double latitude = Math::DegreesToRadians(SkyLocation::latitudeDegrees);
    const double s = std::sin(angle), c = std::cos(angle), sl = std::sin(latitude), cl = std::cos(latitude);
    // HYG: X=RA 0, Y=RA 6h, Z=north celestial pole. World: X=east, Y=up, Z=north.
    return {{-s, c, 0}, {cl * c, cl * s, sl}, {-sl * c, -sl * s, cl}};
}

float StarSky::DaylightFade(double sunHeight)
{
    // Begin appearing just before sunset; finish disappearing just after sunrise.
    // Full strength by sun elevation -6 degrees; completely hidden at +5 degrees.
    const double degrees = std::asin(std::clamp(sunHeight, -1.0, 1.0)) * 180.0 / std::numbers::pi;
    return static_cast<float>(1.0 - Smooth((degrees + 6.0) / 11.0));
}

float StarSky::HorizonFade(double starHeight)
{
    return static_cast<float>(Smooth(starHeight / std::sin(Math::DegreesToRadians(5.0))));
}

struct Stars::Impl
{
    SDL_Renderer* renderer = nullptr;
    SDL_GPUDevice* device = nullptr;
    SDL_GPUGraphicsPipeline* pipeline = nullptr;
    SDL_GPUBuffer* buffer = nullptr;
    SDL_GPUSampler* sampler = nullptr;
    SDL_GPUTexture* gpuSprite = nullptr;
    SDL_Texture* target = nullptr;
    SDL_Texture* sprite = nullptr;
    bool gpu = false, cached = false;
    double lastMinutes = -1;
    float lastFade = -1;
    SDL_Rect lastViewport{};
    Vector3 lastRight{}, lastUp{}, lastForward{};
    std::vector<SDL_Vertex> vertices;
    std::vector<int> indices;

    ~Impl()
    {
        SDL_DestroyTexture(target);
        SDL_DestroyTexture(sprite);
        if (gpuSprite) SDL_ReleaseGPUTexture(device, gpuSprite);
        if (sampler) SDL_ReleaseGPUSampler(device, sampler);
        if (buffer) SDL_ReleaseGPUBuffer(device, buffer);
        if (pipeline) SDL_ReleaseGPUGraphicsPipeline(device, pipeline);
    }
    bool PrepareGPU();
    bool DrawGPU(const CameraView& camera, const StarSky::Frame& frame, float fade, SDL_Rect viewport);
    bool PrepareSprite();
    void Project(const CameraView& camera, const StarSky::Frame& frame, float fade, SDL_Rect viewport);
};

Stars::Stars() : impl(std::make_unique<Impl>()) {}
Stars::~Stars() = default;

bool Stars::Impl::PrepareSprite()
{
    // Shared by every star and both rendering paths. Additive blending makes
    // black texels invisible, including RGB images without an alpha channel.
    if (gpu)
    {
        // Submit the upload ourselves: SDL's texture upload stays in its pending
        // command buffer, so a separate GPU star pass cannot sample it yet.
        const std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(
            DdsTexture::LoadColorSurface("world/star.tga"), SDL_DestroySurface);
        DdsTexture image;
        // Preserve the sprite's display RGB, matching SDL's additive SDR path.
        if (!surface || !image.FromImage(surface.get(), DdsTexture::Format::Packed)) return false;
        gpuSprite = image.Upload(device);
        return gpuSprite != nullptr;
    }
    sprite = TextureLoader::Load(renderer, "world/star.tga");
    return sprite && SDL_SetTextureBlendMode(sprite, SDL_BLENDMODE_ADD) &&
           SDL_SetTextureScaleMode(sprite, SDL_SCALEMODE_LINEAR);
}

void Stars::Impl::Project(const CameraView& camera, const StarSky::Frame& frame, float fade, SDL_Rect viewport)
{
    vertices.clear();
    indices.clear();
    const double focal = std::min(viewport.w, viewport.h) * 0.5;
    const float scale = SpriteScale(viewport);
    for (const auto& star : catalog)
    {
        const auto world = frame.ToWorld({star.x, star.y, star.z});
        const float strength = fade * StarSky::HorizonFade(world.y) * star.brightness;
        if (strength <= 0) continue;
        const auto view = camera.WorldDirectionToView(world);
        if (view.z <= 0.001) continue;
        const double x = viewport.w * 0.5 + focal * view.x / view.z;
        const double y = viewport.h * 0.5 - focal * view.y / view.z;
        const float radius = (1.5f + 2.0f * std::sqrt(star.brightness)) * scale;
        if (x < -radius || x > viewport.w + radius || y < -radius || y > viewport.h + radius) continue;
        const int base = static_cast<int>(vertices.size());
        const SDL_FColor color{star.r * strength, star.g * strength, star.b * strength, 1};
        const float px = static_cast<float>(x), py = static_cast<float>(y);
        vertices.insert(vertices.end(), {{{px-radius, py-radius}, color, {0, 0}},
                                        {{px+radius, py-radius}, color, {1, 0}},
                                        {{px+radius, py+radius}, color, {1, 1}},
                                        {{px-radius, py+radius}, color, {0, 1}}});
        for (int offset : {0, 1, 2, 0, 2, 3}) indices.push_back(base + offset);
    }
}

#ifdef BIGSKY_WORLD_GPU
bool Stars::Impl::PrepareGPU()
{
    const auto shader = [&](SDL_GPUShaderStage stage, const char* entry,
                            std::span<const Uint8> dxil, std::span<const Uint8> spirv)
    {
        SDL_GPUShaderCreateInfo info{};
        info.stage = stage;
        info.entrypoint = entry;
        info.num_uniform_buffers = stage == SDL_GPU_SHADERSTAGE_VERTEX ? 1 : 0;
        info.num_samplers = stage == SDL_GPU_SHADERSTAGE_FRAGMENT ? 1 : 0;
        const bool vulkan = SDL_GetGPUShaderFormats(device) & SDL_GPU_SHADERFORMAT_SPIRV;
        info.format = vulkan ? SDL_GPU_SHADERFORMAT_SPIRV : SDL_GPU_SHADERFORMAT_DXIL;
        const auto bytes = vulkan ? spirv : dxil;
        info.code = bytes.data();
        info.code_size = bytes.size();
        return SDL_CreateGPUShader(device, &info);
    };
    auto* vs = shader(SDL_GPU_SHADERSTAGE_VERTEX, "VSMain", stars_vert_dxil, stars_vert_spirv);
    if (!vs) return false;
    auto* ps = shader(SDL_GPU_SHADERSTAGE_FRAGMENT, "PSMain", stars_frag_dxil, stars_frag_spirv);
    if (!ps) { SDL_ReleaseGPUShader(device, vs); return false; }
    const SDL_GPUVertexBufferDescription layout{0, sizeof(StarSky::Star), SDL_GPU_VERTEXINPUTRATE_INSTANCE, 0};
    const SDL_GPUVertexAttribute attributes[] = {{0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, 0},
                                                {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(StarSky::Star, r)}};
    SDL_GPUColorTargetDescription color{};
    color.format = SDL_GetGPUTextureFormatFromPixelFormat(SDL_PIXELFORMAT_RGBA32);
    color.blend_state.enable_blend = true;
    color.blend_state.src_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
    color.blend_state.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
    color.blend_state.color_blend_op = SDL_GPU_BLENDOP_ADD;
    color.blend_state.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
    color.blend_state.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
    color.blend_state.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
    SDL_GPUGraphicsPipelineCreateInfo info{};
    info.vertex_shader = vs;
    info.fragment_shader = ps;
    info.vertex_input_state = {&layout, 1, attributes, SDL_arraysize(attributes)};
    info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    info.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
    info.target_info.color_target_descriptions = &color;
    info.target_info.num_color_targets = 1;
    pipeline = SDL_CreateGPUGraphicsPipeline(device, &info);
    SDL_ReleaseGPUShader(device, vs);
    SDL_ReleaseGPUShader(device, ps);
    if (!pipeline) return false;

    SDL_GPUSamplerCreateInfo samplerInfo{};
    samplerInfo.min_filter = samplerInfo.mag_filter = SDL_GPU_FILTER_LINEAR;
    samplerInfo.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
    samplerInfo.address_mode_u = samplerInfo.address_mode_v = samplerInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    sampler = SDL_CreateGPUSampler(device, &samplerInfo);
    if (!sampler) return false;

    constexpr Uint32 bytes = static_cast<Uint32>(sizeof(catalog));
    const SDL_GPUBufferCreateInfo bufferInfo{SDL_GPU_BUFFERUSAGE_VERTEX, bytes, 0};
    buffer = SDL_CreateGPUBuffer(device, &bufferInfo);
    if (!buffer) return false;
    const SDL_GPUTransferBufferCreateInfo transferInfo{SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, bytes, 0};
    auto* transfer = SDL_CreateGPUTransferBuffer(device, &transferInfo);
    if (!transfer) return false;
    auto* data = SDL_MapGPUTransferBuffer(device, transfer, false);
    if (!data) { SDL_ReleaseGPUTransferBuffer(device, transfer); return false; }
    std::memcpy(data, catalog, bytes);
    SDL_UnmapGPUTransferBuffer(device, transfer);
    auto* commands = SDL_AcquireGPUCommandBuffer(device);
    if (!commands) { SDL_ReleaseGPUTransferBuffer(device, transfer); return false; }
    auto* copy = SDL_BeginGPUCopyPass(commands);
    if (!copy)
    {
        SDL_CancelGPUCommandBuffer(commands);
        SDL_ReleaseGPUTransferBuffer(device, transfer);
        return false;
    }
    const SDL_GPUTransferBufferLocation from{transfer, 0};
    const SDL_GPUBufferRegion to{buffer, 0, bytes};
    SDL_UploadToGPUBuffer(copy, &from, &to, false);
    SDL_EndGPUCopyPass(copy);
    const bool submitted = SDL_SubmitGPUCommandBuffer(commands);
    SDL_ReleaseGPUTransferBuffer(device, transfer);
    return submitted;
}

bool Stars::Impl::DrawGPU(const CameraView& camera, const StarSky::Frame& frame, float fade, SDL_Rect viewport)
{
    // Submit SDL's previous target use before cycling/resizing. Never wait for
    // the GPU or read pixels back; the static catalog is uploaded only once.
    if (!SDL_FlushRenderer(renderer)) return false;
    if (!target || target->w != viewport.w || target->h != viewport.h)
    {
        auto* next = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_TARGET, viewport.w, viewport.h);
        if (!next) return false;
        if (!SDL_SetTextureBlendMode(next, SDL_BLENDMODE_ADD) || !SDL_SetTextureScaleMode(next, SDL_SCALEMODE_NEAREST))
        {
            SDL_DestroyTexture(next);
            return false;
        }
        SDL_DestroyTexture(target);
        target = next;
    }
    SDL_GPUColorTargetInfo color{};
    color.texture = static_cast<SDL_GPUTexture*>(SDL_GetPointerProperty(SDL_GetTextureProperties(target), SDL_PROP_TEXTURE_GPU_TEXTURE_POINTER, nullptr));
    color.clear_color = {0, 0, 0, 1};
    color.load_op = SDL_GPU_LOADOP_CLEAR;
    color.store_op = SDL_GPU_STOREOP_STORE;
    color.cycle = true;
    if (!color.texture) return SDL_SetError("Star target has no GPU texture");
    auto* commands = SDL_AcquireGPUCommandBuffer(device);
    if (!commands) return false;
    auto* pass = SDL_BeginGPURenderPass(commands, &color, 1, nullptr);
    if (!pass) { SDL_CancelGPUCommandBuffer(commands); return false; }
    const auto equatorial = [&](Vector3 world) { return frame.east * world.x + frame.up * world.y + frame.north * world.z; };
    const auto right = equatorial(camera.Right()), up = equatorial(camera.Up()), forward = equatorial(camera.Forward());
    const float focal2 = static_cast<float>(std::min(viewport.w, viewport.h));
    const float uniforms[20]{float(right.x), float(right.y), float(right.z), focal2 / viewport.w,
                             float(up.x), float(up.y), float(up.z), focal2 / viewport.h,
                             float(forward.x), float(forward.y), float(forward.z), fade,
                             float(frame.up.x), float(frame.up.y), float(frame.up.z), SpriteScale(viewport),
                             2.0f / viewport.w, 2.0f / viewport.h, 0, 0};
    SDL_PushGPUVertexUniformData(commands, 0, uniforms, sizeof(uniforms));
    SDL_BindGPUGraphicsPipeline(pass, pipeline);
    const SDL_GPUBufferBinding binding{buffer, 0};
    SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
    const SDL_GPUTextureSamplerBinding spriteBinding{gpuSprite, sampler};
    SDL_BindGPUFragmentSamplers(pass, 0, &spriteBinding, 1);
    SDL_DrawGPUPrimitives(pass, 6, static_cast<Uint32>(std::size(catalog)), 0, 0);
    SDL_EndGPURenderPass(pass);
    return SDL_SubmitGPUCommandBuffer(commands);
}
#else
bool Stars::Impl::PrepareGPU() { return false; }
bool Stars::Impl::DrawGPU(const CameraView&, const StarSky::Frame&, float, SDL_Rect) { return false; }
#endif

bool Stars::Draw(SDL_Renderer* renderer, const CameraView& camera, double minutes, double sunHeight, SDL_Rect viewport)
{
    const float fade = StarSky::DaylightFade(sunHeight);
    if (fade <= 0 || viewport.w <= 0 || viewport.h <= 0) return true;
    auto& c = *impl;
    if (!c.renderer)
    {
        c.renderer = renderer;
#ifdef BIGSKY_WORLD_GPU
        if (SDL_strcmp(SDL_GetRendererName(renderer), "gpu") == 0)
        {
            c.device = SDL_GetGPURendererDevice(renderer);
            c.gpu = c.device && c.PrepareGPU();
            if (!c.gpu) SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Using SDL star rendering: %s", SDL_GetError());
        }
#endif
    }
    if (c.renderer != renderer) return SDL_SetError("Star field cannot be shared between renderers");
    if (!c.sprite && !c.gpuSprite && !c.PrepareSprite()) return false;
    const bool changed = !c.cached || minutes != c.lastMinutes || fade != c.lastFade ||
        camera.Right() != c.lastRight || camera.Up() != c.lastUp || camera.Forward() != c.lastForward ||
        viewport.w != c.lastViewport.w || viewport.h != c.lastViewport.h;
    if (changed)
    {
        const auto frame = StarSky::At(minutes);
        if (c.gpu)
        {
            if (!c.DrawGPU(camera, frame, fade, viewport)) return false;
        }
        else c.Project(camera, frame, fade, viewport);
        c.lastMinutes = minutes;
        c.lastFade = fade;
        c.lastRight = camera.Right();
        c.lastUp = camera.Up();
        c.lastForward = camera.Forward();
        c.lastViewport = viewport;
        c.cached = true;
    }
    if (c.gpu)
    {
        const SDL_FRect dest{0, 0, float(viewport.w), float(viewport.h)};
        return SDL_RenderTexture(renderer, c.target, nullptr, &dest);
    }
    return c.vertices.empty() || SDL_RenderGeometry(renderer, c.sprite, c.vertices.data(), static_cast<int>(c.vertices.size()),
                                                    c.indices.data(), static_cast<int>(c.indices.size()));
}
