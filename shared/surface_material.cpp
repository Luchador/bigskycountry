#include "surface_material.hpp"
#include "texture_loader.hpp"
#include "material_blend.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
    using Format = DdsTexture::Format;
    constexpr std::array formats{Format::BaseColor, Format::Normal, Format::Packed, Format::BaseColor, Format::Scalar, Format::Scalar};
    constexpr std::array<std::array<Uint8, 4>, formats.size()> defaults{{
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
    // Both layers share the same texture formats, cache entries and neutral maps.
    if (slot == MaterialSlot::OuterBaseColor) slot = MaterialSlot::BaseColor;
    if (slot == MaterialSlot::OuterNormal) slot = MaterialSlot::Normal;
    if (slot == MaterialSlot::OuterORM) slot = MaterialSlot::ORM;
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
    const bool transition = material.type == MaterialType::Transition;
    using Slot = MaterialSlot;
    // The second triplet carries either outer-layer maps or the emission and alpha maps.
    // Bind a valid texture even for shader inputs unused by the current type.
    constexpr std::array standardSlots{Slot::BaseColor, Slot::Normal, Slot::ORM, Slot::Emissive, Slot::Opacity, Slot::ORM};
    constexpr std::array transitionSlots{Slot::BaseColor, Slot::Normal, Slot::ORM, Slot::OuterBaseColor, Slot::OuterNormal, Slot::OuterORM};
    constexpr std::array unlitSlots{Slot::Emissive, Slot::Emissive, Slot::Emissive, Slot::Emissive, Slot::Translucency, Slot::Emissive};
    const auto& slots = transition ? transitionSlots : unlit ? unlitSlots : standardSlots;
    for (size_t i = 0; i < slots.size(); ++i)
        bindings[i] = {material.maps[static_cast<size_t>(slots[i])]->gpu, sampler};
    SDL_BindGPUFragmentSamplers(pass, 0, bindings.data(), bindings.size());
    struct Properties { SDL_FColor tint; Uint32 cutout, unlit, transition, padding; };
    const Properties params{material.tint, material.type == MaterialType::AlphaCutout, unlit, transition, 0};
    SDL_PushGPUFragmentUniformData(commands, 1, &params, sizeof(params));
}

SDL_FColor SurfaceMaterial::Shade(float u, float v, float footprint, const WorldLighting& lights, Vector3 normal,
                                  Vector3 tangent, float handedness, Vector3 view, float vertexRed) const
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
    auto color = sample(MaterialSlot::BaseColor);
    if (type == MaterialType::AlphaCutout && color[3] * sample(MaterialSlot::Opacity)[0] < 0.5f)
        return {};
    auto orm = sample(MaterialSlot::ORM);
    auto mappedNormal = DecodeNormalXY(sample(MaterialSlot::Normal));
    std::array<float, 4> glow{};
    if (type == MaterialType::Transition)
    {
        const float weight = std::clamp(vertexRed, 0.f, 1.f);
        color = BlendMaterialSamples(sample(MaterialSlot::OuterBaseColor), color, weight);
        orm = BlendMaterialSamples(sample(MaterialSlot::OuterORM), orm, weight);
        const auto outerNormal = DecodeNormalXY(sample(MaterialSlot::OuterNormal));
        mappedNormal = outerNormal + (mappedNormal - outerNormal) * weight;
    }
    else glow = sample(MaterialSlot::Emissive);
    // Re-orthogonalize the projected U axis against the interpolated shading normal.
    tangent = Math::Normalize(tangent - normal * Math::Dot(normal, tangent));
    const Vector3 bitangent = Math::Cross(normal, tangent) * handedness;
    const Vector3 mapped = Math::Normalize(tangent * mappedNormal.x + bitangent * mappedNormal.y + normal * mappedNormal.z);
    return lights.Shade(mapped, view, {color[0], color[1], color[2], 1}, {orm[0], orm[1], orm[2]},
                        {glow[0], glow[1], glow[2]});
}
