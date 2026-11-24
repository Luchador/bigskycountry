#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>
#include <vector>

struct DirectionalLight
{
    Vector3 direction; // Unit vector from the surface toward the light.
    Vector3 radiance;  // Linear RGB, including intensity.
};

// Position is relative to the render camera; direction points out along the beam.
// Smooth cone and distance falloffs are shared by the GPU and software renderer.
struct SpotLight
{
    Vector3 position, direction, intensity;
    double range = 1, innerCos = 1, outerCos = 0;
    Vector3 Radiance(Vector3 surfaceToEye) const;
};

struct WorldLighting
{
    DirectionalLight sun, moon;
    Vector3 ambient;
    std::vector<SpotLight> spots;

    // ShadeLinear mirrors ShadeSurface in shaders/world.hlsl. Shade also encodes to sRGB.
    SDL_FColor Shade(Vector3 normal, Vector3 view, SDL_FColor linearBase,
                     Vector3 orm = {1, 1, 0}, Vector3 emission = {}) const;
    SDL_FColor ShadeLinear(Vector3 normal, Vector3 view, SDL_FColor linearBase,
                           Vector3 orm = {1, 1, 0}, Vector3 emission = {}, Vector3 transmission = {}) const;
};

namespace Srgb
{
    float ToLinear(float display);
    float ToDisplay(float linear);
    SDL_FColor ToLinear(SDL_FColor display);
    SDL_FColor ToDisplay(SDL_FColor linear);
}
