#pragma once

#include "mesh.hpp"
#include <optional>
#include "surface_material.hpp"
#include <array>
#include <SDL3/SDL.h>

class FreeCamera;
class TerrainChunk;
struct RoadGeometry;
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
    bool Begin(SDL_Renderer* renderer, const FreeCamera& camera, const WorldLighting* lights = nullptr);
    void BeginUnlit() { lightGeometry = false; }
    // World colors are linear albedo. Editor overlay colors are display-encoded.
    void SubmitTriangle(Vector3 a, Vector3 b, Vector3 c, SDL_FColor color);
    void SubmitTriangle(const std::array<Vector3, 3>& positions, const std::array<SDL_FColor, 3>& colors);
    void SubmitTerrain(const TerrainChunk& chunk);
    void SubmitRoad(const RoadGeometry& road, const std::string& material, bool selected);
    bool SphereVisible(Vector3 center, double radius) const;
    void SubmitMesh(const MeshInstance& instance, SDL_FColor tint = {1, 1, 1, 1});
    enum class MeshUVs { Authored, TerrainProjection };
    void SubmitMaterialMesh(const MeshInstance& instance, const std::string& material, MeshUVs mapping = MeshUVs::Authored);
    // Long-lived world placements can reuse their GPU vertices across frames.
    void SubmitWorldMesh(const MeshInstance& instance, const std::string& material = {},
                         MeshUVs mapping = MeshUVs::Authored, SDL_FColor tint = {1, 1, 1, 1});
    // Mark the world triangles before submitting solid editor handles and overlays.
    void WireframeSubmittedGeometry();
    // Subsequent solid editor handles depth-test against each other in front of the world.
    void BeginForeground() { foregroundStart = vertices.size(); }
    bool End();

private:
    bool BoundsVisible(const Bounds3& bounds, const MeshTransform* placement = nullptr) const;
    bool PrepareTarget(SDL_Renderer* renderer, int width, int height);
    bool CreatePipelines();
    bool DrawGPU();
    bool DrawSoftware();
    void BuildDrawRanges();

    void SubmitMaterialTriangle(const std::array<Vector3, 3>& points, const std::array<SDL_FColor, 3>& colors,
                                const std::array<SDL_FPoint, 3>& uv, const std::array<Vector3, 3>* smoothNormals = nullptr);
    WorldDepthVertex WorldToClip(Vector3 position, SDL_FColor color) const;
    SDL_Renderer* renderer = nullptr;
    const FreeCamera* camera = nullptr;
    SDL_Rect viewport{};
    double focal = 0, scaleX = 0, scaleY = 0;
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
        ~ResidentGeometry();
        Vector3 origin{};
        std::vector<WorldDepthVertex> pendingUpload;
        std::vector<MaterialRange> parts; // Contiguous, including the untextured gaps.
        std::string material;
        std::uint64_t revision = 0, lastFrame = 0, uploadFrame = 0;
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
    };
    std::unordered_map<const void*, std::shared_ptr<ResidentGeometry>> resident;
    std::vector<ResidentDraw> residentDraws;
    std::uint64_t frame = 0;
    bool residentWire = false;
    std::optional<Vector3> packingOrigin;
    bool CanRetain(const std::string& material);
    void CaptureGeometry(ResidentGeometry& geometry, size_t start, size_t firstMaterial);
    void EmitRoadSurface(const RoadGeometry& road, const std::string& material);
    SurfaceMaterialCache materialCache;
    WorldLighting lights{};
    bool lightGeometry = false;
    std::size_t wireframeEnd = 0, foregroundStart = 0;
    struct VertexRange { std::size_t first, end; };
    std::vector<VertexRange> selectedRanges; // Ordered, disjoint spans of selected world geometry.
    std::size_t outlineStart = 0;
    SDL_Texture* colorTarget = nullptr;

    SDL_GPUDevice* device = nullptr; // Borrowed from the SDL renderer.
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    SDL_GPUTexture* depthTarget = nullptr;
    SDL_GPUGraphicsPipeline* surfacePipeline = nullptr;
    SDL_GPUGraphicsPipeline* texturedPipeline = nullptr;
    SDL_GPUGraphicsPipeline* translucentPipeline = nullptr;
    SDL_GPUGraphicsPipeline* wireframePipeline = nullptr;
    SDL_GPUGraphicsPipeline* outlinePipeline = nullptr;
    SDL_GPUBuffer* vertexBuffer = nullptr;
    SDL_GPUTransferBuffer* upload = nullptr;
    Uint32 bufferBytes = 0, uploadBytes = 0;

    struct Sample { float depth; bool selected; };
    std::vector<Sample> softwareDepth;
};
