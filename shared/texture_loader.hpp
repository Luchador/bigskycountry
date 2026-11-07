#pragma once

#include <SDL3/SDL.h>

class TextureLoader
{
public:
    TextureLoader() = delete;

    // Paths are relative to assets/textures, e.g. "world/sun.tga".
    // Call on the main thread. The caller owns the result and releases it with
    // SDL_DestroyTexture. On failure, returns nullptr and sets SDL_GetError().
    static SDL_Texture* Load(SDL_Renderer* renderer, const char* relativePath);
};
