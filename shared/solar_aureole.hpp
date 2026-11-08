#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>

// Diffuse light scattered toward the viewer around the sun or moon.
class SolarAureole
{
public:
    // A relative strength of 1.0 gives the original solar peak.
    SolarAureole(Vector3 lightDirection, SDL_FColor lightTint, double relativeStrength = 1.0);
    bool Visible() const { return strength > 0.0; }
    SDL_FColor Brighten(SDL_FColor sky, Vector3 unitRay) const;

private:
    Vector3 direction;
    SDL_FColor linearTint;
    double strength;
};
