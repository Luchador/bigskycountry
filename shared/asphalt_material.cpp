#include "asphalt_material.hpp"
#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>

AsphaltMaterial::~AsphaltMaterial()
{
    Release();
}

void AsphaltMaterial::Release()
{
    for (auto*& texture : gpu)
    {
        if (texture)
        {
            SDL_ReleaseGPUTexture(device, texture);
            texture = nullptr;
        }
    }
    if (sampler)
    {
        SDL_ReleaseGPUSampler(device, sampler);
        sampler = nullptr;
    }
    cpu = {};
}

bool AsphaltMaterial::Prepare(SDL_GPUDevice* target)
{
    if (attempted) return ready;
    attempted = true;
    device = target;
    ready = Load();
    if (!ready) Release();
    return ready;
}

bool AsphaltMaterial::Load()
{
    const char* base = SDL_GetBasePath();
    if (!base) return false;
    const std::string prefix = std::string(base) + "assets/cooked/textures/roads/asphalt01_";
    using Format = DdsTexture::Format;
    constexpr std::array maps = {Format::BaseColor, Format::Normal, Format::Scalar};
    constexpr const char* names[] = {"basecolor.dds", "normal.dds", "roughness.dds"};
    for (size_t i = 0; i < maps.size(); ++i)
    {
        DdsTexture texture;
        const std::string path = prefix + names[i];
        if (!texture.Load(path, maps[i]))
        {
            SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Using plain pavement: %s: %s", path.c_str(), SDL_GetError());
            return false;
        }
        if (device)
        {
            gpu[i] = texture.Upload(device);
            if (!gpu[i])
            {
                SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Could not upload %s: %s", path.c_str(), SDL_GetError());
                return false;
            }
        }
        else
        {
            texture.Decode();
            cpu[i] = std::move(texture);
        }
    }
    if (device)
    {
        SDL_GPUSamplerCreateInfo info{};
        info.min_filter = info.mag_filter = SDL_GPU_FILTER_LINEAR;
        info.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
        info.address_mode_u = info.address_mode_v = info.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
        info.max_lod = 15.0f;
        sampler = SDL_CreateGPUSampler(device, &info);
        if (!sampler)
        {
            SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Could not create asphalt sampler: %s", SDL_GetError());
            return false;
        }
    }
    return true;
}

void AsphaltMaterial::Bind(SDL_GPURenderPass* pass) const
{
    const SDL_GPUTextureSamplerBinding bindings[] = {{gpu[0], sampler}, {gpu[1], sampler}, {gpu[2], sampler}};
    SDL_BindGPUFragmentSamplers(pass, 0, bindings, 3);
}

SDL_FColor AsphaltMaterial::Shade(float u, float v, float footprint, const PavementLighting& basis) const
{
    const auto color = cpu[0].Sample(u, v, footprint);
    const auto xy = cpu[1].Sample(u, v, footprint);
    const float roughness = cpu[2].Sample(u, v, footprint)[0];
    const float x = xy[0] * 2 - 1, y = xy[1] * 2 - 1;
    const float z = std::sqrt(std::max(0.0f, 1 - x*x - y*y));
    const float invLength = 1 / std::sqrt(x*x + y*y + z*z);
    auto cosine = [&](const std::array<float, 3>& dir)
    {
        return std::clamp((x*dir[0] + y*dir[1] + z*dir[2]) * invLength, 0.0f, 1.0f);
    };
    const float diffuse = cosine(basis.light);
    const float gloss = 1 - roughness;
    auto normalize = [](std::array<float, 3> v)
    {
        const float scale = 1 / std::sqrt(std::max(1.0e-12f, v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
        for (auto& component : v) component *= scale;
        return v;
    };
    auto half = normalize(basis.view);
    for (size_t i = 0; i < half.size(); ++i) half[i] += basis.light[i];
    const float specular = 0.08f * gloss * diffuse * std::pow(cosine(normalize(half)), 4 + 124 * gloss * gloss);
    auto shade = [&](float albedo)
    {
        const float c = std::min(1.0f, albedo * (0.28f + 0.72f * diffuse) + specular);
        return c <= 0.0031308f ? 12.92f * c : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
    };
    return {shade(color[0]), shade(color[1]), shade(color[2]), 1};
}
