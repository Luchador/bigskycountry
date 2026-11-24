#pragma once

#include "mesh.hpp"
#include <optional>
#include "surface_material.hpp"
#include "world_fog.hpp"
#include "shadow_cascades.hpp"
#include <array>
#include <SDL3/SDL.h>

class CameraView;
class TerrainChunk;
struct RoadGeometry;
struct RailroadTieRun;
inline constexpr double worldNearDepth = 0.1;

// Packed world-space vectors used by the CPU rasterizer and the GPU vertex layout.
struct WorldFloat3
{
    float x{}, y{}, z{};
    WorldFloat3() = default;
    WorldFloat3(Vector3 v) : x(float(v.x)), y(float(v.y)), z(float(v.z)) {}
    operator Vector3() const { return {x, y, z}; }
};

// Dynamic vertices use homogeneous clip positions with reversed depth. Resident
// GPU vertices use xyz relative to a batch origin and store their residual in view.
struct WorldDepthVertex
{
    float x, y, z, w;
    SDL_FColor color;
    SDL_FPoint uv{};
    WorldFloat3 normal{};
    float lit = 0;
    WorldFloat3 tangent{};
    float handedness = 1;
    WorldFloat3 view{};
};

class WorldRenderer
{
public:
    explicit WorldRenderer(AssetRoot location = AssetRoot::Cooked) : materialCache(location) {}
    ~WorldRenderer();
    WorldRenderer(const WorldRenderer&) = delete;
    WorldRenderer& operator=(const WorldRenderer&) = delete;

    // One batch per 3D viewport. All submissions share depth and stencil until End.
    // Supply the sky lighting for a world view. Omit it for a prelit editor preview.
    // Fog and shadows are opt-in for world viewports; previews stay clear.
    // Disable ambientOcclusion for raw vertex-color inspection.
    void Forget(const void* key);
    std::size_t ResidentBytes() const;
    // Per-frame work counters, also displayed by the F3 overlay.
    std::size_t PreparedVertices() const { return preparedVertices; }
    std::size_t UploadedBytes() const { return uploadedBytes; }
    std::size_t ReadyBuffers() const;
    bool DrawCacheStatistics(SDL_Renderer* target, float y = 36) const;
    std::size_t TextureBytes() const { return materialCache.Bytes(); }
    std::size_t TextureCount() const { return materialCache.TextureCount(); }
    int MsaaSamples() const { return sampleCount == SDL_GPU_SAMPLECOUNT_4 ? 4 : sampleCount == SDL_GPU_SAMPLECOUNT_2 ? 2 : 0; }
    bool Begin(SDL_Renderer* renderer, const CameraView& camera, const WorldLighting* lights = nullptr, const WorldFog* fog = nullptr,
               double viewDistance = 0, bool shadows = false, bool ambientOcclusion = true);
    // Spot positions supplied here are in world space. Rebuilt each frame, never cached with meshes.
    void SubmitSpotLight(SpotLight light);
    void SubmitEmissiveTriangle(Vector3 a, Vector3 b, Vector3 c, SDL_FColor linearColor);
    void BeginUnlit() { shadowEnd = vertices.size(); lightGeometry = false; }
    // World colors are linear albedo. Editor overlay colors are display-encoded.
    void SubmitTriangle(Vector3 a, Vector3 b, Vector3 c, SDL_FColor color);
    void SubmitTriangle(const std::array<Vector3, 3>& positions, const std::array<SDL_FColor, 3>& colors);
    void SubmitTerrain(const TerrainChunk& chunk, bool selected = false);
    void SubmitRailroadTies(const RailroadTieRun& run, std::uint64_t revision, bool selected = false);
    void SubmitRoad(const RoadGeometry& road, const std::string& material, bool selected);
    bool SphereVisible(Vector3 center, double radius) const;
    bool BoundsVisible(const Bounds3& bounds, const MeshTransform* placement = nullptr) const;
    // Includes off-screen casters intersecting an active cascade, within the view radius.
    bool BoundsNeeded(const Bounds3& bounds, const MeshTransform* placement = nullptr) const;
    void SubmitMesh(const MeshInstance& instance, SDL_FColor tint = {1, 1, 1, 1});
    enum class MeshUVs { Authored, TerrainProjection };
    void SubmitMaterialMesh(const MeshInstance& instance, const std::string& material, MeshUVs mapping = MeshUVs::Authored, SDL_FColor tint = {1, 1, 1, 1});
    // Long-lived world placements can reuse their GPU vertices across frames.
    void SubmitWorldMesh(const MeshInstance& instance, const std::string& material = {},
                         MeshUVs mapping = MeshUVs::Authored, SDL_FColor tint = {1, 1, 1, 1}, int lodOverride = -1);
    std::size_t MeshLOD(const MeshInstance& instance, int lodOverride = -1) const;
    // Mark the world triangles before submitting solid editor handles and overlays.
    void WireframeSubmittedGeometry();
    // Subsequent solid editor handles depth-test against each other in front of the world.
    void BeginForeground() { foregroundStart = vertices.size(); }
    bool End();

private:
    bool PrepareTarget(SDL_Renderer* renderer, int width, int height);
    bool CreatePipelines();
    bool DrawGPU();
    // Three float4s per light followed by per-tile light bitsets, packed as uint4s.
    std::vector<SDL_FColor> BuildSpotLightData() const;
    static constexpr int lightTilePixels = 64;
    static constexpr size_t maxSpotLights = 512;
    SDL_GPUBuffer* spotBuffer = nullptr;
    Uint32 spotBufferBytes = 0;
    bool PrepareShadowTarget(int resolution);
    bool DrawShadows(SDL_GPUCommandBuffer* commands);
    bool CreateSSAOFallback();
    bool PrepareSSAO();
    bool CreateSSAOPipelines();
    bool DrawSSAO(SDL_GPUCommandBuffer* commands);
    void ReleaseSSAO();
    bool ssaoRequested = false, ssaoActive = false, ssaoFailed = false, ssaoInitialized = false;
    int ssaoWidth = 0, ssaoHeight = 0, ssaoFullWidth = 0, ssaoFullHeight = 0;
    SDL_GPUTexture* ssaoWhite = nullptr;
    SDL_GPUTexture* ssaoDepth = nullptr;
    SDL_GPUTexture* ssaoResult = nullptr;
    SDL_GPUTexture* ssaoTemp = nullptr;
    SDL_GPUSampler* ssaoSampler = nullptr;
    SDL_GPUGraphicsPipeline* ssaoDepthPipeline = nullptr;
    SDL_GPUGraphicsPipeline* ssaoCutoutPipeline = nullptr;
    SDL_GPUGraphicsPipeline* ssaoFoliagePipeline = nullptr;
    SDL_GPUGraphicsPipeline* ssaoPipeline = nullptr;
    SDL_GPUGraphicsPipeline* ssaoBlurPipeline = nullptr;
    bool DrawSoftware();
    void BuildDrawRanges();

    void SubmitMaterialTriangle(const std::array<Vector3, 3>& points, const std::array<SDL_FColor, 3>& colors,
                                const std::array<SDL_FPoint, 3>& uv, const std::array<Vector3, 3>* smoothNormals = nullptr);
    WorldDepthVertex WorldToClip(Vector3 position, SDL_FColor color) const;
    SDL_Renderer* renderer = nullptr;
    const CameraView* camera = nullptr;
    SDL_Rect viewport{};
    double focal = 0, scaleX = 0, scaleY = 0;
    double viewDistance = 0; // Zero disables distance culling for previews.
    std::vector<WorldDepthVertex> vertices;
    std::vector<WorldDepthVertex> projectedMesh;
    std::vector<WorldDepthVertex> outline;
    struct MaterialRange
    {
        size_t first, end;
        std::string path;
        const SurfaceMaterial* material = nullptr;
        bool transitionOnly = false;
    };
    std::vector<MaterialRange> materialRanges;
    struct DrawRange
    {
        std::size_t first, end;
        const SurfaceMaterial* material;
        bool selected;
    };
    std::vector<DrawRange> draws;
    struct ResidentGeometry
    {
        SDL_GPUDevice* device = nullptr;
        SDL_GPUBuffer* buffer = nullptr;
        std::size_t bufferBytes = 0;
        std::size_t uploadedBytes = 0, uploadEnd = 0;
        bool complete = false; // CPU packing finished; GPU upload may still be in progress.
        bool meshOwned = false;
        ~ResidentGeometry();
        Vector3 origin{};
        Bounds3 bounds;
        std::vector<WorldDepthVertex> pendingUpload;
        std::vector<MaterialRange> parts; // Contiguous, including the untextured gaps.
        std::string material;
        std::uint64_t revision = 0, lastFrame = 0, uploadFrame = 0, choiceFrame = 0;
        std::weak_ptr<const Mesh> mesh;
        MeshTransform transform;
        bool moving = false;
        MeshUVs mapping = MeshUVs::Authored;
        SDL_FColor tint{};
    };
    struct ResidentDraw
    {
        std::shared_ptr<ResidentGeometry> geometry;
        size_t before; // Position in the dynamic stream, preserving submission order.
        bool selected;
        bool uploadOnly = false; // A ready LOD remains visible while this one uploads.
    };
    struct ResidentKey
    {
        const void* owner;
        std::size_t lod;
        ResidentKey(const void* owner, std::size_t lod = 0) : owner(owner), lod(lod) {}
        bool operator==(const ResidentKey&) const = default;
    };
    struct ResidentHash
    {
        std::size_t operator()(ResidentKey key) const { return std::hash<const void*>{}(key.owner) ^ (key.lod * 0x9e3779b9u); }
    };
    std::unordered_map<ResidentKey, std::shared_ptr<ResidentGeometry>, ResidentHash> resident;
    std::vector<ResidentDraw> residentDraws;
    std::uint64_t frame = 0;
    std::size_t preparedVertices = 0, uploadedBytes = 0;
    std::uint64_t preparationNanos = 0;
    // Bound both triangle work and elapsed preparation time. Large meshes resume
    // next frame, rather than escaping the budget as a single oversized batch.
    static constexpr std::size_t maxPreparedVertices = 8192 * 3;
    static constexpr std::uint64_t maxPreparationNanos = 2'000'000;
    bool CanPrepare() const;
    void PrepareMesh(ResidentGeometry& geometry, const MeshInstance& instance);
    bool residentWire = false;
    std::optional<Vector3> packingOrigin;
    bool CanRetain(const std::string& material);
    void CaptureGeometry(ResidentGeometry& geometry, size_t start, size_t firstMaterial);
    void EmitRoadSurface(const RoadGeometry& road, const std::string& material);
    void AppendSelectionVertex(SDL_FPoint point, double depth, double lift = 0);
    SurfaceMaterialCache materialCache;
    WorldLighting lights{};
    std::optional<WorldFog> fog;
    bool lightGeometry = false;
    std::size_t wireframeEnd = 0, foregroundStart = 0;
    struct VertexRange { std::size_t first, end; };
    std::vector<VertexRange> selectedRanges; // Ordered, disjoint spans of selected world geometry.
    std::size_t outlineStart = 0;
    SDL_Texture* colorTarget = nullptr;

    ShadowCascades shadows;
    std::size_t shadowEnd = 0;
    SDL_GPUTexture* shadowTarget = nullptr;
    SDL_GPUSampler* shadowSampler = nullptr;
    SDL_GPUGraphicsPipeline* shadowPipeline = nullptr;
    SDL_GPUGraphicsPipeline* shadowCutoutPipeline = nullptr;
    SDL_GPUTextureFormat shadowFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    int shadowResolution = 0;
    bool shadowInitialized = false;

    SDL_GPUDevice* device = nullptr; // Borrowed from the SDL renderer.
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    SDL_GPUSampleCount sampleCount = SDL_GPU_SAMPLECOUNT_1;
    SDL_GPUTexture* multisampleColor = nullptr;
    SDL_GPUTexture* depthTarget = nullptr;
    SDL_GPUGraphicsPipeline* surfacePipeline = nullptr;
    SDL_GPUGraphicsPipeline* texturedPipeline = nullptr;
    SDL_GPUGraphicsPipeline* cutoutPipeline = nullptr;
    SDL_GPUGraphicsPipeline* foliagePipeline = nullptr;
    SDL_GPUGraphicsPipeline* translucentPipeline = nullptr;
    SDL_GPUGraphicsPipeline* wireframePipeline = nullptr;
    SDL_GPUGraphicsPipeline* outlinePipeline = nullptr;
    SDL_GPUBuffer* vertexBuffer = nullptr;
    SDL_GPUTransferBuffer* upload = nullptr;
    Uint32 bufferBytes = 0, uploadBytes = 0;

    struct Sample { float depth; bool selected; };
    std::vector<Sample> softwareDepth;
};
