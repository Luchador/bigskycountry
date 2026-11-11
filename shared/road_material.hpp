#pragma once

#include "asset_paths.hpp"
#include "dds_texture.hpp"
#include "material.hpp"
#include <map>
#include <optional>
#include <utility>

struct PavementLighting
{
    std::array<float, 3> light{}, view{};
};

struct RoadMaterialTexture
{
    DdsTexture cpu;
    SDL_GPUTexture* gpu = nullptr;
};

struct RoadMaterial
{
    static constexpr double tileMeters = 4.0;
    std::array<const RoadMaterialTexture*, materialSlotCount> maps{};
    MaterialType type{};
    SDL_FColor Shade(float u, float v, float footprint, const PavementLighting& basis) const;
};

// One cache per renderer/device. Different materials can reference the same texture.
class RoadMaterials
{
public:
    explicit RoadMaterials(AssetRoot location) : location(location), root(AssetPaths::Root(location) + "textures/") {}
    ~RoadMaterials();
    RoadMaterials(const RoadMaterials&) = delete;
    RoadMaterials& operator=(const RoadMaterials&) = delete;
    const RoadMaterial* Get(const std::string& path, SDL_GPUDevice* target);
    void Bind(const RoadMaterial& material, SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass) const;

private:
    void ClearTextures();
    const RoadMaterialTexture* Map(const std::string& path, MaterialSlot slot);
    const AssetRoot location;
    const std::string root;
    SDL_GPUDevice* device = nullptr;
    SDL_GPUSampler* sampler = nullptr;
    unsigned revision = 0, imageRevision = 0;
    std::map<std::string, std::optional<RoadMaterial>> materials;
    using TextureKey = std::pair<std::string, DdsTexture::Format>;
    std::map<TextureKey, std::optional<RoadMaterialTexture>> textures;
};
