#include "surface_material.hpp"
#include "texture_loader.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
    using Format = DdsTexture::Format;
    constexpr std::array formats{Format::BaseColor, Format::Normal, Format::Packed, Format::BaseColor, Format::Scalar, Format::Scalar};
    constexpr std::array<std::array<Uint8, 4>, materialSlotCount> defaults{{
        {184,184,184,255}, {128,128,255,255}, {255,128,0,255}, {0,0,0,255}, {255,255,255,255}, {255,255,255,255}
    }};
}

SurfaceMaterialCache::~SurfaceMaterialCache()
{
    ClearTextures();
    if (sampler) SDL_ReleaseGPUSampler(device, sampler);
}

void SurfaceMaterialCache::ClearTextures()
{
    for (auto& [key, texture] : textures)
        if (texture && texture->gpu) SDL_ReleaseGPUTexture(device, texture->gpu);
    textures.clear();
}

const SurfaceMaterialTexture* SurfaceMaterialCache::Map(const std::string& path, MaterialSlot slot, bool whiteEmission)
{
    const auto index = static_cast<size_t>(slot);
    const auto format = formats[index];
    // ':' cannot occur in an asset path. Give each neutral map its own cache key.
    const auto key = path.empty() ? (whiteEmission ? ":white" : ":" + std::to_string(index)) : path;
    auto [it, inserted] = textures.try_emplace(TextureKey{key, format});
    if (inserted)
    {
        DdsTexture image;
        bool loaded = true;
        if (path.empty())
            image.Solid(format, whiteEmission ? std::array<Uint8,4>{255,255,255,255} : defaults[index]);
        else
        {
            const bool editor = location == AssetRoot::Project;
            const std::string source = root + path;
            const std::string cooked = DdsTexture::CookedPath(path, format);
            SDL_PathInfo src{}, dds{};
            loaded = (!editor || (SDL_GetPathInfo(source.c_str(), &src) && SDL_GetPathInfo(cooked.c_str(), &dds) &&
                                  dds.modify_time >= src.modify_time)) && image.Load(cooked, format);
            if (editor && !loaded)
            {
                // Preview newly added or edited source maps before the next cook.
                std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> pixels(
                    TextureLoader::LoadImageFile(source), SDL_DestroySurface);
                loaded = pixels && image.FromImage(pixels.get(), format);
            }
        }
        if (loaded)
        {
            if (device)
            {
                if (auto* gpu = image.Upload(device)) it->second.emplace().gpu = gpu;
            }
            else
            {
                image.Decode();
                it->second.emplace().cpu = std::move(image);
            }
        }
        if (!it->second) SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Surface material texture '%s': %s", path.c_str(), SDL_GetError());
    }
    if (it->second) return &*it->second;
    return path.empty() ? nullptr : Map({}, slot, whiteEmission);
}

const SurfaceMaterial* SurfaceMaterialCache::Get(const std::string& path, SDL_GPUDevice* target)
{
    device = target;
    if (revision != Material::Revision())
    {
        materials.clear();
        revision = Material::Revision();
        if (imageRevision != Material::ImageRevision())
        {
            ClearTextures();
            imageRevision = Material::ImageRevision();
        }
    }
    auto [it, inserted] = materials.try_emplace(path);
    if (inserted)
    {
        Material definition;
        if (!Material::ValidAssetPath(path) || !definition.Load(root + path))
        {
            SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Surface material '%s': %s", path.c_str(), SDL_GetError());
            return nullptr;
        }
        if (device && !sampler)
        {
            SDL_GPUSamplerCreateInfo info{};
            info.min_filter = info.mag_filter = SDL_GPU_FILTER_LINEAR;
            info.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;
            info.address_mode_u = info.address_mode_v = info.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
            info.max_lod = 15;
            sampler = SDL_CreateGPUSampler(device, &info);
            if (!sampler) return nullptr;
        }
        SurfaceMaterial material;
        material.type = definition.type;
        const auto& tint = definition.emissiveTint;
        material.tint = Srgb::ToLinear({tint[0], tint[1], tint[2], definition.translucency});
        for (size_t i = 0; i < materialSlotCount; ++i)
        {
            const auto slot = static_cast<MaterialSlot>(i);
            if (!definition.Uses(slot)) continue;
            material.maps[i] = Map(definition.textures[i], slot, material.type == MaterialType::Unlit && slot == MaterialSlot::Emissive);
            if (!material.maps[i]) return nullptr;
        }
        it->second = material;
    }
    return it->second ? &*it->second : nullptr;
}

void SurfaceMaterialCache::Bind(const SurfaceMaterial& material, SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass) const
{
    std::array<SDL_GPUTextureSamplerBinding, SurfaceMaterial::shaderMapCount> bindings{};
    const bool unlit = material.type == MaterialType::Unlit;
    for (size_t i = 0; i < bindings.size(); ++i)
    {
        auto slot = static_cast<MaterialSlot>(i);
        // All declared bindings must be valid. Reuse emission for the unused lit maps.
        if (unlit) slot = slot == MaterialSlot::Opacity ? MaterialSlot::Translucency : MaterialSlot::Emissive;
        bindings[i] = {material.maps[static_cast<size_t>(slot)]->gpu, sampler};
    }
    SDL_BindGPUFragmentSamplers(pass, 0, bindings.data(), bindings.size());
    struct Properties { SDL_FColor tint; Uint32 cutout, unlit, padding[2]; };
    const Properties params{material.tint, material.type == MaterialType::AlphaCutout, unlit, {}};
    SDL_PushGPUFragmentUniformData(commands, 1, &params, sizeof(params));
}

SDL_FColor SurfaceMaterial::Shade(float u, float v, float footprint, const WorldLighting& lights, Vector3 normal,
                                  Vector3 tangent, float handedness, Vector3 view) const
{
    const auto sample = [&](MaterialSlot slot) {
        return maps[static_cast<size_t>(slot)]->cpu.Sample(u, v, footprint);
    };
    if (type == MaterialType::Unlit)
    {
        const auto glow = sample(MaterialSlot::Emissive);
        return {Srgb::ToDisplay(glow[0] * tint.r), Srgb::ToDisplay(glow[1] * tint.g),
                Srgb::ToDisplay(glow[2] * tint.b), sample(MaterialSlot::Translucency)[0] * tint.a};
    }
    const auto color = sample(MaterialSlot::BaseColor);
    if (type == MaterialType::AlphaCutout && color[3] * sample(MaterialSlot::Opacity)[0] < 0.5f)
        return {};
    const auto xy = sample(MaterialSlot::Normal);
    const auto orm = sample(MaterialSlot::ORM);
    const auto glow = sample(MaterialSlot::Emissive);
    const float x = xy[0] * 2 - 1, y = xy[1] * 2 - 1;
    const float z = std::sqrt(std::max(0.0f, 1 - x*x - y*y));
    // Re-orthogonalize the projected U axis against the interpolated shading normal.
    tangent = Math::Normalize(tangent - normal * Math::Dot(normal, tangent));
    const Vector3 bitangent = Math::Cross(normal, tangent) * handedness;
    const Vector3 mapped = Math::Normalize(tangent * x + bitangent * y + normal * z);
    return lights.Shade(mapped, view, {color[0], color[1], color[2], 1}, {orm[0], orm[1], orm[2]},
                        {glow[0], glow[1], glow[2]});
}
