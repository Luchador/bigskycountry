#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>
#include <memory>

class FreeCamera;

class Moon
{
public:
    // Repeating full-moon path tied to the world's apparent sun direction.
    static Vector3 Direction(Vector3 sunDirection);
    bool Draw(SDL_Renderer* renderer, const FreeCamera& camera, Vector3 direction, SDL_Rect viewport);

private:
    bool Load(SDL_Renderer* renderer);
    std::unique_ptr<SDL_Texture, decltype(&SDL_DestroyTexture)> texture{nullptr, SDL_DestroyTexture};
    SDL_Rect discBounds{};
};
