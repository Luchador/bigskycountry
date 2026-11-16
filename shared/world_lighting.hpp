#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>

struct DirectionalLight
{
    Vector3 direction; // Unit vector from the surface toward the light.
    Vector3 radiance;  // Linear RGB, including intensity.
};

struct WorldLighting
{
    DirectionalLight sun, moon;
    Vector3 ambient;

    // ShadeLinear mirrors ShadeSurface in shaders/world.hlsl. Shade also encodes to sRGB.
    SDL_FColor Shade(Vector3 normal, Vector3 view, SDL_FColor linearBase,
                     Vector3 orm = {1, 1, 0}, Vector3 emission = {}) const;
    SDL_FColor ShadeLinear(Vector3 normal, Vector3 view, SDL_FColor linearBase,
                           Vector3 orm = {1, 1, 0}, Vector3 emission = {}) const;
};

namespace Srgb
{
    float ToLinear(float display);
    float ToDisplay(float linear);
    SDL_FColor ToLinear(SDL_FColor display);
    SDL_FColor ToDisplay(SDL_FColor linear);
}
