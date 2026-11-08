#include "sun_shader.hpp"

SunShader::~SunShader()
{
    SDL_DestroyGPURenderState(state);
    if (shader)
    {
        SDL_ReleaseGPUShader(device, shader);
    }
}

bool SunShader::Initialize(SDL_Renderer* renderer, SDL_Texture* mask)
{
    // The built-in renderer returns straight RGB and mask alpha, so its blend mode
    // applies coverage once. The custom shader applies coverage before blending.
    if (!SDL_SetTextureBlendMode(mask, SDL_BLENDMODE_ADD))
    {
        return false;
    }

    return true;
}

bool SunShader::Draw(SDL_Renderer* renderer, SDL_Texture* mask, const SDL_Vertex* vertices, int vertexCount,
                     const int* indices, int indexCount) const
{
    if (state && !SDL_SetGPURenderState(renderer, state))
    {
        return false;
    }

    const bool drawn = SDL_RenderGeometry(renderer, mask, vertices, vertexCount, indices, indexCount);

    if (state && !SDL_SetGPURenderState(renderer, nullptr))
    {
        return false;
    }

    return drawn;
}
