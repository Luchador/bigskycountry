#pragma once

#include <SDL3/SDL.h>
#include <string>

class TextureLoader
{
public:
    TextureLoader() = delete;

    // Identify a cooked RGBA sprite by its source path, e.g. "world/sun.tga".
    // Call on the main thread. The caller owns the result and releases it with
    // SDL_DestroyTexture. On failure, returns nullptr and sets SDL_GetError().
    static SDL_Texture* Load(SDL_Renderer* renderer, const char* relativePath);

    // Source image in straight-alpha RGBA32, for uncooked materials and browser previews.
    static SDL_Surface* LoadImageFile(const std::string& path);

    // Decode a TGA at the supplied filesystem path for editor previews. The caller owns the surface.
    static SDL_Surface* LoadTgaSurfaceFromFile(const char* path);

    // Preserve the image colors and return the bounds of texels with at least half opacity.
    static SDL_Texture* Load(SDL_Renderer* renderer, const char* relativePath, SDL_Rect& alphaBounds);

    // Turn brightness into opacity on a white texture, multiplying by any source alpha.
    // discBounds encloses texels with at least half opacity, excluding the sprite's padding.
    static SDL_Texture* LoadMask(SDL_Renderer* renderer, const char* relativePath, SDL_Rect& discBounds);
};
