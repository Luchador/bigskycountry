#include "texture_loader.hpp"
#include <SDL3/SDL_main.h>
#include <cstdio>

int main(int argc, char** argv)
{
    if (argc != 3) return 1;
    SDL_Surface* image = TextureLoader::LoadImageFile(argv[1]);
    const bool saved = image && SDL_SavePNG(image, argv[2]);
    if (!saved) std::fprintf(stderr, "%s: %s\n", argv[1], SDL_GetError());
    SDL_DestroySurface(image);
    return saved ? 0 : 1;
}
