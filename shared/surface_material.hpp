#pragma once

#include "asset_paths.hpp"
#include "dds_texture.hpp"
#include "material.hpp"
#include "world_lighting.hpp"
#include <map>
#include <compare>
#include <optional>
#include <utility>
#include <thread>
#include <mutex>
#include <condition_variable>

struct SurfaceMaterialTexture
{
    DdsTexture cpu;
    SDL_GPUTexture* gpu = nullptr;
    std::size_t bytes = 0;
    std::uint64_t lastUsed = 0;
};

struct SurfaceMaterial
{
    static constexpr double tileMeters = 4.0;
    static constexpr unsigned shaderMapCount = 6;
    std::array<const SurfaceMaterialTexture*, materialSlotCount> maps{};
    MaterialType type{};
    std::uint64_t lastUsed = 0;
    SDL_FColor tint{1, 1, 1, 1}; // Linear RGB with the translucency multiplier in alpha.
    float subsurfaceStrength = .5f;
    bool hasSubsurfaceMap = false;
    // Linear radiance: apply atmosphere before the final display conversion.
    SDL_FColor ShadeLinear(float u, float v, float footprint, const WorldLighting& lights, Vector3 normal,
                     Vector3 tangent, float handedness, Vector3 view, float vertexRed, bool backFace = false, float vertexAO = 1) const;
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
    // Called before any material pointers for this frame are acquired.
    void BeginFrame(SDL_GPUDevice* target, std::size_t budget);
    std::size_t Bytes() const;
    std::size_t TextureCount() const;
    void Bind(const SurfaceMaterial& material, SDL_GPUCommandBuffer* commands, SDL_GPURenderPass* pass, Uint32 firstSampler = 0) const;
    void BindShadow(const SurfaceMaterial& material, SDL_GPURenderPass* pass) const;

private:
    void ClearTextures();
    const SurfaceMaterialTexture* Map(const std::string& path, MaterialSlot slot, bool whiteEmission, bool preserveCoverage = false);
    const AssetRoot location;
    const std::string root;
    SDL_GPUDevice* device = nullptr;
    SDL_GPUSampler* sampler = nullptr;
    unsigned revision = 0, imageRevision = 0;
    std::map<std::string, std::optional<SurfaceMaterial>> materials;
    struct TextureKey
    {
        std::string path;
        DdsTexture::Format format;
        bool preserveCoverage = false;
        auto operator<=>(const TextureKey&) const = default;
    };
    std::map<TextureKey, std::optional<SurfaceMaterialTexture>> textures;
    std::map<std::string, std::optional<Material>> definitions;
    std::uint64_t frame = 0, epoch = 0;
    bool asynchronous = false;
    struct Job { TextureKey key; std::uint64_t epoch; };
    struct Completion { Job job; DdsTexture image; std::string error; };
    std::map<TextureKey, std::uint64_t> requested;
    std::optional<Job> pending, job;
    std::optional<Completion> completion;
    std::mutex mutex;
    std::condition_variable condition;
    std::thread worker;
    bool stopping = false;
    void Run();
    void Publish(const TextureKey& key, DdsTexture image);
    void Touch(SurfaceMaterial& material);
};
