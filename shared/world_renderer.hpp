#pragma once

#include "mesh.hpp"
#include "surface_material.hpp"
#include <array>
#include <SDL3/SDL.h>

class FreeCamera;
struct RoadGeometry;
class TerrainChunk;
inline constexpr double worldNearDepth = 0.1;

// Packed world-space vectors used by the CPU rasterizer and the GPU vertex layout.
struct WorldFloat3
{
    float x{}, y{}, z{};
    WorldFloat3() = default;
    WorldFloat3(Vector3 v) : x(float(v.x)), y(float(v.y)), z(float(v.z)) {}
    operator Vector3() const { return {x, y, z}; }
};

// Homogeneous clip position. z / w is near / viewZ, with 1 at the near plane.
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
    // Mark the world triangles before submitting solid editor handles and overlays.
    void WireframeSubmittedGeometry();
    // Subsequent solid editor handles depth-test against each other in front of the world.
    void BeginForeground() { foregroundStart = vertices.size(); }
    bool End();

private:
    bool PrepareTarget(SDL_Renderer* renderer, int width, int height);
    bool CreatePipelines();
    bool DrawGPU();
    bool DrawSoftware();

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
    };
    std::vector<MaterialRange> materialRanges;
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
    SDL_GPUGraphicsPipeline* wireframePipeline = nullptr;
    SDL_GPUGraphicsPipeline* outlinePipeline = nullptr;
    SDL_GPUBuffer* vertexBuffer = nullptr;
    SDL_GPUTransferBuffer* upload = nullptr;
    Uint32 bufferBytes = 0;

    struct Sample { float depth; bool selected; };
    std::vector<Sample> softwareDepth;
};
