#pragma once

#include "mesh.hpp"
#include "asphalt_material.hpp"
#include <array>
#include <SDL3/SDL.h>

class FreeCamera;
struct Road;
inline constexpr double worldNearDepth = 0.1;

// Homogeneous clip position. z / w is near / viewZ, with 1 at the near plane.
struct WorldDepthVertex
{
    float x, y, z, w;
    SDL_FColor color;
    SDL_FPoint uv{};
    PavementLighting lighting{};
};

class WorldRenderer
{
public:
    WorldRenderer() = default;
    ~WorldRenderer();
    WorldRenderer(const WorldRenderer&) = delete;
    WorldRenderer& operator=(const WorldRenderer&) = delete;

    // One batch per 3D viewport. All submissions share depth and stencil until End.
    bool Begin(SDL_Renderer* renderer, const FreeCamera& camera);
    void SubmitTriangle(Vector3 a, Vector3 b, Vector3 c, SDL_FColor color);
    void SubmitTriangle(const std::array<Vector3, 3>& positions, const std::array<SDL_FColor, 3>& colors);
    void SubmitRoad(const Road& road, bool selected);
    void SubmitMesh(const MeshInstance& instance, SDL_FColor tint = {1, 1, 1, 1});
    bool End();

private:
    bool PrepareTarget(SDL_Renderer* renderer, int width, int height);
    bool CreatePipelines();
    bool DrawGPU();
    bool DrawSoftware();

    WorldDepthVertex WorldToClip(Vector3 position, SDL_FColor color) const;
    SDL_Renderer* renderer = nullptr;
    const FreeCamera* camera = nullptr;
    SDL_Rect viewport{};
    double focal = 0, scaleX = 0, scaleY = 0;
    std::vector<WorldDepthVertex> vertices;
    std::vector<WorldDepthVertex> projectedMesh;
    std::vector<WorldDepthVertex> outline;
    struct PavementRange { size_t first, end; };
    std::vector<PavementRange> pavement;
    AsphaltMaterial asphalt;
    Vector3 pavementSun{};
    bool drawAsphalt = false;
    std::size_t selectedStart = 0, selectedEnd = 0, outlineStart = 0;
    SDL_Texture* colorTarget = nullptr;

    SDL_GPUDevice* device = nullptr; // Borrowed from the SDL renderer.
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    SDL_GPUTexture* depthTarget = nullptr;
    SDL_GPUGraphicsPipeline* surfacePipeline = nullptr;
    SDL_GPUGraphicsPipeline* asphaltPipeline = nullptr;
    SDL_GPUGraphicsPipeline* outlinePipeline = nullptr;
    SDL_GPUBuffer* vertexBuffer = nullptr;
    SDL_GPUTransferBuffer* upload = nullptr;
    Uint32 bufferBytes = 0;

    struct Sample { float depth; bool selected; };
    std::vector<Sample> softwareDepth;
};
