#pragma once

#include "asset_paths.hpp"
#include "dds_texture.hpp"
#include "material.hpp"
#include "world_lighting.hpp"
#include <map>
#include <optional>
#include <utility>

struct SurfaceMaterialTexture
{
    DdsTexture cpu;
    SDL_GPUTexture* gpu = nullptr;
};

struct SurfaceMaterial
{
    static constexpr double tileMeters = 4.0;
    static constexpr unsigned shaderMapCount = 6;
    std::array<const SurfaceMaterialTexture*, materialSlotCount> maps{};
    MaterialType type{};
    SDL_FColor tint{1, 1, 1, 1}; // Linear RGB with the translucency multiplier in alpha.
    SDL_FColor Shade(float u, float v, float footprint, const WorldLighting& lights, Vector3 normal,
                     Vector3 tangent, float handedness, Vector3 view, float vertexRed) const;
};

// One cache per renderer/device. Different materials can reference the same texture.
class SurfaceMaterialCache
{
public:
    explicit SurfaceMaterialCache(AssetRoot location) : location(location), root(AssetPaths::Root(location) + "textures/") {}
    ~SurfaceMaterialCache();
    SurfaceMaterialCache(const SurfaceMaterialCache&) = delete;
    SurfaceMaterialCache& operator=(const SurfaceMaterialCache&) = delete;
    const SurfaceMaterial* Get(const std::string& path, SDL_GPUDevice* target);
    void Bind(const SurfaceMaterial& material, SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass) const;

private:
    void ClearTextures();
    const SurfaceMaterialTexture* Map(const std::string& path, MaterialSlot slot, bool whiteEmission);
    const AssetRoot location;
    const std::string root;
    SDL_GPUDevice* device = nullptr;
    SDL_GPUSampler* sampler = nullptr;
    unsigned revision = 0, imageRevision = 0;
    std::map<std::string, std::optional<SurfaceMaterial>> materials;
    using TextureKey = std::pair<std::string, DdsTexture::Format>;
    std::map<TextureKey, std::optional<SurfaceMaterialTexture>> textures;
};
