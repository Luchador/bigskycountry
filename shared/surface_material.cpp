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
    { std::lock_guard lock(mutex); stopping = true; job.reset(); }
    condition.notify_one();
    if (worker.joinable()) worker.join();
    ClearTextures();
    if (sampler) SDL_ReleaseGPUSampler(device, sampler);
}

void SurfaceMaterialCache::ClearTextures()
{
    for (auto& [key, texture] : textures)
        if (texture && texture->gpu) SDL_ReleaseGPUTexture(device, texture->gpu);
    textures.clear();
}

std::size_t SurfaceMaterialCache::Bytes() const
{
    std::size_t bytes = 0;
    for (const auto& [key, texture] : textures) if (texture) bytes += texture->bytes;
    return bytes;
}
std::size_t SurfaceMaterialCache::TextureCount() const
{
    return std::count_if(textures.begin(), textures.end(), [](const auto& e) { return e.second && e.first.path.front() != ':'; });
}
void SurfaceMaterialCache::Run()
{
    for (;;)
    {
        Job request;
        {
            std::unique_lock lock(mutex);
            condition.wait(lock, [&] { return stopping || job.has_value(); });
            if (stopping) return;
            request = std::move(*job); job.reset();
        }
        Completion done; done.job = request;
        try
        {
            if (!done.image.Load(DdsTexture::CookedPath(request.key.path, request.key.format, request.key.preserveCoverage), request.key.format))
                done.error = SDL_GetError();
        }
        catch (const std::exception& e) { done.error = e.what(); }
        std::lock_guard lock(mutex);
        if (stopping) return;
        completion = std::move(done);
    }
}
void SurfaceMaterialCache::Publish(const TextureKey& key, DdsTexture image)
{
    auto& texture = textures[key];
    if (device)
    {
        if (auto* gpu = image.Upload(device))
        {
            auto& value = texture.emplace(); value.gpu = gpu; value.bytes = image.DataBytes(); value.lastUsed = frame;
        }
    }
    else
    {
        image.Decode();
        auto& value = texture.emplace(); value.bytes = image.DataBytes(); value.cpu = std::move(image); value.lastUsed = frame;
    }
}
void SurfaceMaterialCache::Touch(SurfaceMaterial& material)
{
    material.lastUsed = frame;
    // Each map can be referenced by arbitrarily many materials, with one residency charge.
    for (auto* map : material.maps) if (map) const_cast<SurfaceMaterialTexture*>(map)->lastUsed = frame;
}
void SurfaceMaterialCache::BeginFrame(SDL_GPUDevice* target, std::size_t budget)
{
    device = target;
    ++frame;
    // Keep stable bindings across frames; invalidate only after publication,
    // eviction or an authoring revision, before new draw pointers are acquired.
    bool bindingsChanged = false;
    if (revision != Material::Revision())
    {
        definitions.clear(); revision = Material::Revision(); bindingsChanged = true;
        if (imageRevision != Material::ImageRevision())
        {
            ++epoch; ClearTextures(); requested.clear(); imageRevision = Material::ImageRevision();
        }
    }
    asynchronous = location == AssetRoot::Cooked;
    std::optional<Completion> done;
    {
        std::lock_guard lock(mutex);
        if (completion) { done = std::move(completion); completion.reset(); pending.reset(); }
    }
    if (done && done->job.epoch == epoch)
    {
        const auto request = requested.find(done->job.key);
        // A turn away must not cancel a useful read that is already in flight.
        // Publish once and let the normal memory budget decide its lifetime.
        if (request != requested.end())
        {
            if (done->error.empty()) { Publish(done->job.key, std::move(done->image)); bindingsChanged = true; }
            else
            {
                textures.try_emplace(done->job.key); // Failed maps use neutral fallback until Refresh.
                SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Streaming texture '%s': %s", done->job.key.path.c_str(), done->error.c_str());
            }
        }
    }
    std::erase_if(requested, [&](const auto& e) {
        return (!pending || pending->key != e.first) && e.second + 2 < frame;
    });
    std::size_t bytes = Bytes();
    // Keep warm maps while there is room. A frame-count timeout reloaded them
    // every time the camera spun around, even with hundreds of MiB to spare.
    for (;;)
    {
        auto victim = textures.end();
        for (auto it = textures.begin(); it != textures.end(); ++it)
        {
            if (!it->second || it->first.path.front() == ':' || it->second->lastUsed + 1 >= frame) continue;
            if (bytes <= budget) continue;
            if (victim == textures.end() || it->second->lastUsed < victim->second->lastUsed) victim = it;
        }
        if (victim == textures.end()) break;
        bytes -= victim->second->bytes;
        if (victim->second->gpu) SDL_ReleaseGPUTexture(device, victim->second->gpu);
        textures.erase(victim); bindingsChanged = true;
    }
    if (bindingsChanged) materials.clear();
}

const SurfaceMaterialTexture* SurfaceMaterialCache::Map(const std::string& path, MaterialSlot slot, bool whiteEmission, bool preserveCoverage)
{
    // Both layers share the same texture formats, cache entries and neutral maps.
    if (slot == MaterialSlot::OuterBaseColor) slot = MaterialSlot::BaseColor;
    if (slot == MaterialSlot::OuterNormal) slot = MaterialSlot::Normal;
    if (slot == MaterialSlot::OuterORM) slot = MaterialSlot::ORM;
    // Subsurface RGB is sRGB color; its alpha is a linear transmission mask.
    if (slot == MaterialSlot::Subsurface) slot = MaterialSlot::BaseColor;
    const auto index = static_cast<size_t>(slot);
    const auto format = formats[index];
    // ':' cannot occur in an asset path. Give each neutral map its own cache key.
    const auto key = path.empty() ? (whiteEmission ? ":white" : ":" + std::to_string(index)) : path;
    const TextureKey textureKey{key, format, preserveCoverage && !path.empty()};
    if (asynchronous && !path.empty() && !textures.contains(textureKey))
    {
        requested[textureKey] = frame;
        if (!pending)
        {
            if (!worker.joinable()) worker = std::thread([this] { Run(); });
            pending = Job{textureKey, epoch};
            { std::lock_guard lock(mutex); job = pending; }
            condition.notify_one();
        }
        return Map({}, slot, whiteEmission);
    }
    auto [it, inserted] = textures.try_emplace(textureKey);
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
            const std::string cooked = DdsTexture::CookedPath(path, format, preserveCoverage);
            SDL_PathInfo src{}, dds{};
            loaded = (!editor || (SDL_GetPathInfo(source.c_str(), &src) && SDL_GetPathInfo(cooked.c_str(), &dds) &&
                                  dds.modify_time >= src.modify_time)) && image.Load(cooked, format);
            if (editor && !loaded)
            {
                // Preview newly added or edited source maps before the next cook.
                std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> pixels(
                    TextureLoader::LoadImageFile(source), SDL_DestroySurface);
                loaded = pixels && image.FromImage(pixels.get(), format, preserveCoverage);
            }
        }
        if (loaded) Publish(textureKey, std::move(image));
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
        definitions.clear();
        revision = Material::Revision();
        if (imageRevision != Material::ImageRevision())
        {
            ++epoch; ClearTextures(); requested.clear();
            imageRevision = Material::ImageRevision();
        }
    }
    auto [it, inserted] = materials.try_emplace(path);
    if (inserted)
    {
        auto [definitionIt, newDefinition] = definitions.try_emplace(path);
        if (newDefinition)
        {
            Material value;
            if (Material::ValidAssetPath(path) && value.Load(root + path)) definitionIt->second = std::move(value);
            else SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Surface material '%s': %s", path.c_str(), SDL_GetError());
        }
        if (!definitionIt->second) return nullptr;
        const auto& definition = *definitionIt->second;
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
        material.water = definition.water;
        material.useVertexAlphaAO = Material::SupportsVertexAlphaAO(definition.type) && definition.useVertexAlphaAO;
        material.subsurfaceStrength = definition.subsurfaceStrength;
        material.alphaCutoff = definition.alphaCutoff;
        material.hasSubsurfaceMap = !definition.textures[static_cast<size_t>(MaterialSlot::Subsurface)].empty();
        const auto& tint = definition.emissiveTint;
        material.tint = Srgb::ToLinear({tint[0], tint[1], tint[2], definition.translucency});
        for (size_t i = 0; i < materialSlotCount; ++i)
        {
            const auto slot = static_cast<MaterialSlot>(i);
            if (!definition.Uses(slot)) continue;
            material.maps[i] = Map(definition.textures[i], slot, material.type == MaterialType::Unlit && slot == MaterialSlot::Emissive,
                                   material.type == MaterialType::Foliage && (slot == MaterialSlot::BaseColor || slot == MaterialSlot::Opacity));
            if (!material.maps[i]) return nullptr;
        }
        it->second = material;
    }
    if (!inserted && it->second && it->second->lastUsed != frame)
    {
        const auto& definition = *definitions.at(path);
        for (std::size_t i = 0; i < materialSlotCount; ++i)
        {
            const auto slot = static_cast<MaterialSlot>(i);
            if (definition.Uses(slot))
                it->second->maps[i] = Map(definition.textures[i], slot, definition.type == MaterialType::Unlit && slot == MaterialSlot::Emissive,
                                        definition.type == MaterialType::Foliage && (slot == MaterialSlot::BaseColor || slot == MaterialSlot::Opacity));
        }
    }
    if (it->second) Touch(*it->second);
    return it->second ? &*it->second : nullptr;
}

void SurfaceMaterialCache::Bind(const SurfaceMaterial& material, SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, Uint32 firstSampler) const
{
    if (material.type == MaterialType::Water)
    {
        const SDL_GPUTextureSamplerBinding binding{material.maps[static_cast<size_t>(MaterialSlot::Normal)]->gpu, sampler};
        SDL_BindGPUFragmentSamplers(pass, firstSampler, &binding, 1);
        return; // The water pass supplies its own uniforms and scene buffers.
    }
    std::array<SDL_GPUTextureSamplerBinding, SurfaceMaterial::shaderMapCount> bindings{};
    const bool unlit = material.type == MaterialType::Unlit;
    const bool transition = material.type == MaterialType::Transition;
    using Slot = MaterialSlot;
    // The second triplet carries either outer-layer maps or the emission and alpha maps.
    // Bind a valid texture even for shader inputs unused by the current type.
    constexpr std::array standardSlots{Slot::BaseColor, Slot::Normal, Slot::ORM, Slot::Emissive, Slot::Opacity, Slot::ORM};
    constexpr std::array transitionSlots{Slot::BaseColor, Slot::Normal, Slot::ORM, Slot::OuterBaseColor, Slot::OuterNormal, Slot::OuterORM};
    constexpr std::array unlitSlots{Slot::Emissive, Slot::Emissive, Slot::Emissive, Slot::Emissive, Slot::Translucency, Slot::Emissive};
    constexpr std::array foliageSlots{Slot::BaseColor, Slot::Normal, Slot::ORM, Slot::Emissive, Slot::Opacity, Slot::Subsurface};
    const bool foliage = material.type == MaterialType::Foliage;
    const auto& slots = foliage ? foliageSlots : transition ? transitionSlots : unlit ? unlitSlots : standardSlots;
    for (size_t i = 0; i < slots.size(); ++i)
        bindings[i] = {material.maps[static_cast<size_t>(slots[i])]->gpu, sampler};
    SDL_BindGPUFragmentSamplers(pass, firstSampler, bindings.data(), bindings.size());
    struct Properties { SDL_FColor tint; Uint32 cutout, unlit, transition, foliage; float strength, hasMap, vertexAO, alphaCutoff; };
    static_assert(sizeof(Properties) == 48);
    const Properties params{material.tint, Material::IsCutout(material.type), unlit, transition, foliage,
                            material.subsurfaceStrength, material.hasSubsurfaceMap ? 1.f : 0.f,
                            material.useVertexAlphaAO ? 1.f : 0.f, material.alphaCutoff};
    SDL_PushGPUFragmentUniformData(commands, 1, &params, sizeof(params));
}

void SurfaceMaterialCache::BindShadow(const SurfaceMaterial& material, SDL_GPURenderPass* pass) const
{
    const SDL_GPUTextureSamplerBinding maps[]{
        {material.maps[static_cast<size_t>(MaterialSlot::BaseColor)]->gpu, sampler},
        {material.maps[static_cast<size_t>(MaterialSlot::Opacity)]->gpu, sampler}};
    SDL_BindGPUFragmentSamplers(pass, 0, maps, 2);
}

SDL_FColor SurfaceMaterial::ShadeLinear(float u, float v, float footprint, const WorldLighting& lights, Vector3 normal,
                                  Vector3 tangent, float handedness, Vector3 view, float vertexRed, const Projection& outer, bool backFace, float vertexAO) const
{
    if (type == MaterialType::Water)
    {
        const auto c = Srgb::ToLinear({water.color[0], water.color[1], water.color[2], 1});
        return lights.ShadeLinear(normal, view, c, {1, water.roughness, 0});
    }
    const auto sample = [&](MaterialSlot slot) {
        return maps[static_cast<size_t>(slot)]->cpu.Sample(u, v, footprint);
    };
    if (type == MaterialType::Unlit)
    {
        const auto glow = sample(MaterialSlot::Emissive);
        return {glow[0] * tint.r, glow[1] * tint.g, glow[2] * tint.b,
                sample(MaterialSlot::Translucency)[0] * tint.a};
    }
    auto color = sample(MaterialSlot::BaseColor);
    if (Material::IsCutout(type) && color[3] * sample(MaterialSlot::Opacity)[0] < alphaCutoff)
        return {};
    auto orm = sample(MaterialSlot::ORM);
    auto mappedNormal = DecodeNormalXY(sample(MaterialSlot::Normal));
    // Re-orthogonalize each projected U axis against the shading normal.
    const auto mapNormal = [&](Vector3 sample, Vector3 axis, float sign) {
        const Vector3 t = Math::Normalize(axis - normal * Math::Dot(normal, axis));
        const Vector3 b = Math::Cross(normal, t) * sign;
        return Math::Normalize(t * sample.x + b * sample.y + normal * sample.z);
    };
    Vector3 mapped = mapNormal(mappedNormal, tangent, handedness);
    std::array<float, 4> glow{};
    if (type == MaterialType::Transition)
    {
        const auto outerSample = [&](MaterialSlot slot) {
            return maps[static_cast<size_t>(slot)]->cpu.Sample(outer.uv.x, outer.uv.y, outer.footprint);
        };
        const float weight = std::clamp(vertexRed, 0.f, 1.f);
        color = BlendMaterialSamples(outerSample(MaterialSlot::OuterBaseColor), color, weight);
        orm = BlendMaterialSamples(outerSample(MaterialSlot::OuterORM), orm, weight);
        const auto outerNormal = mapNormal(DecodeNormalXY(outerSample(MaterialSlot::OuterNormal)), outer.tangent, outer.handedness);
        mapped = Math::Normalize(outerNormal + (mapped - outerNormal) * weight);
    }
    else glow = sample(MaterialSlot::Emissive);
    if (useVertexAlphaAO) orm[0] *= std::clamp(vertexAO, 0.f, 1.f);
    Vector3 transmission{};
    if (type == MaterialType::Foliage)
    {
        if (backFace) mapped = mapped * -1;
        const auto subsurface = hasSubsurfaceMap ? sample(MaterialSlot::Subsurface) :
                                                   std::array{color[0], color[1], color[2], 1.f};
        transmission = Vector3{subsurface[0], subsurface[1], subsurface[2]} * (subsurface[3] * subsurfaceStrength);
    }
    return lights.ShadeLinear(mapped, view, {color[0], color[1], color[2], 1}, {orm[0], orm[1], orm[2]},
                        {glow[0], glow[1], glow[2]}, transmission);
}
