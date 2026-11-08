#pragma once

#include <SDL3/SDL.h>

class SunShader
{
public:
    SunShader() = default;
    ~SunShader();
    SunShader(const SunShader&) = delete;
    SunShader& operator=(const SunShader&) = delete;

    bool Initialize(SDL_Renderer* renderer, SDL_Texture* mask);
    bool Draw(SDL_Renderer* renderer, SDL_Texture* mask, const SDL_Vertex* vertices, int vertexCount,
              const int* indices, int indexCount) const;

private:
    SDL_GPUDevice* device = nullptr; // Owned by the renderer.
    SDL_GPUShader* shader = nullptr;
    SDL_GPURenderState* state = nullptr;
};
