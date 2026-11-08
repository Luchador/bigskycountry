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

    // Preserve the image colors and return the bounds of texels with at least half opacity.
    static SDL_Texture* Load(SDL_Renderer* renderer, const char* relativePath, SDL_Rect& alphaBounds);

    // Turn brightness into opacity on a white texture, multiplying by any source alpha.
    // discBounds encloses texels with at least half opacity, excluding the sprite's padding.
    static SDL_Texture* LoadMask(SDL_Renderer* renderer, const char* relativePath, SDL_Rect& discBounds);
};
