#pragma once

#include "road.hpp"
#include <SDL3/SDL.h>

class FreeCamera;

// Homogeneous clip position. z / w is near / viewZ, with 1 at the near plane.
struct RoadDepthVertex
{
    float x, y, z, w;
    SDL_FColor color;
};

class RoadDepthRenderer
{
public:
    RoadDepthRenderer() = default;
    ~RoadDepthRenderer();
    RoadDepthRenderer(const RoadDepthRenderer&) = delete;
    RoadDepthRenderer& operator=(const RoadDepthRenderer&) = delete;

    bool Draw(SDL_Renderer* renderer, const FreeCamera& view, const std::vector<Road>& roads, const Road* selected);

private:
    bool PrepareTarget(SDL_Renderer* renderer, int width, int height);
    bool CreatePipelines();
    bool DrawGPU();
    bool DrawSoftware();

    std::vector<RoadDepthVertex> vertices;
    std::size_t selectedStart = 0, selectedEnd = 0, outlineStart = 0;
    SDL_Texture* colorTarget = nullptr;

    SDL_GPUDevice* device = nullptr; // Borrowed from the SDL renderer.
    SDL_GPUTextureFormat depthFormat = SDL_GPU_TEXTUREFORMAT_INVALID;
    SDL_GPUTexture* depthTarget = nullptr;
    SDL_GPUGraphicsPipeline* surfacePipeline = nullptr;
    SDL_GPUGraphicsPipeline* outlinePipeline = nullptr;
    SDL_GPUBuffer* vertexBuffer = nullptr;
    SDL_GPUTransferBuffer* upload = nullptr;
    Uint32 bufferBytes = 0;

    struct Sample { float depth; bool selected; };
    std::vector<Sample> softwareDepth;
};
