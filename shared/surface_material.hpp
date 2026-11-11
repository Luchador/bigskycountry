#pragma once

#include "asset_paths.hpp"
#include "dds_texture.hpp"
#include "material.hpp"
#include <map>
#include <optional>
#include <utility>

struct SurfaceLighting
{
    std::array<float, 3> light{}, view{};
};

struct SurfaceMaterialTexture
{
    DdsTexture cpu;
    SDL_GPUTexture* gpu = nullptr;
};

struct SurfaceMaterial
{
    static constexpr double tileMeters = 4.0;
    std::array<const SurfaceMaterialTexture*, materialSlotCount> maps{};
    MaterialType type{};
    SDL_FColor Shade(float u, float v, float footprint, const SurfaceLighting& basis) const;
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
    const SurfaceMaterialTexture* Map(const std::string& path, MaterialSlot slot);
    const AssetRoot location;
    const std::string root;
    SDL_GPUDevice* device = nullptr;
    SDL_GPUSampler* sampler = nullptr;
    unsigned revision = 0, imageRevision = 0;
    std::map<std::string, std::optional<SurfaceMaterial>> materials;
    using TextureKey = std::pair<std::string, DdsTexture::Format>;
    std::map<TextureKey, std::optional<SurfaceMaterialTexture>> textures;
};
