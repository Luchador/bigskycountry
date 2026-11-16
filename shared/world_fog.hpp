#pragma once

#include "world_lighting.hpp"

// Shared by game/editor and GPU/software rendering. Distances are engine meters.
// A thin uniform haze plus exponentially decreasing ground-level aerosol density.
struct WorldFog
{
    double density = 0.00045;       // Extinction per meter at baseHeight.
    double heightFalloff = 1.0 / 800.0;
    double baseHeight = 0.0;
    double startDistance = 50.0;   // Keep nearby surfaces clear, without a hard edge.
    double airDensity = 0.00002;   // Distant mountains still fade above the ground layer.

    // Display-encoded sky palette; match the sky gradient before converting to linear.
    SDL_FColor zenith{}, oppositeHorizon{}, sunwardHorizon{};
    Vector3 horizontalSun{};

    double Transmittance(Vector3 eyeToSurface, double eyeHeight) const;
    SDL_FColor Apply(SDL_FColor linearSurface, Vector3 surfaceToEye, double eyeHeight) const;
};
