#pragma once

#include "math.hpp"
#include "sun_shader.hpp"
#include <SDL3/SDL.h>
#include <memory>

class FreeCamera;

class Sun
{
public:
    // Local civil minutes since midnight. Returns an apparent direction in east/up/north axes.
    static Vector3 Direction(double minutes);
    static SDL_FColor Tint(double elevationRadians);
    bool Draw(SDL_Renderer* renderer, const FreeCamera& camera, Vector3 direction, SDL_Rect viewport,
              SDL_FColor baseSkyAtSun = {0.0f, 0.0f, 0.0f, 1.0f});

private:
    bool Load(SDL_Renderer* renderer);
    std::unique_ptr<SDL_Texture, decltype(&SDL_DestroyTexture)> texture{nullptr, SDL_DestroyTexture};
    SDL_Rect discBounds{};
    SunShader discShader;
};
