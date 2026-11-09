#include "transform_space.hpp"
#include <SDL3/SDL.h>
#include <string>

SDL_Surface* LoadTransformSpaceIcon(GizmoSpace space)
{
    const char* base = SDL_GetBasePath();
    if (!base)
    {
        return nullptr;
    }
    const char* name = space == GizmoSpace::World ? "btn_transform_global.png" : "btn_transform_local.png";
    const std::string path = std::string(base) + "assets/textures/editor/" + name;
    SDL_Surface* surface = SDL_LoadPNG(path.c_str());
    if (!surface)
    {
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Toolbar icon %s: %s", path.c_str(), SDL_GetError());
    }
    return surface;
}
