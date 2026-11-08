#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>

// Diffuse sunlight scattered toward the viewer by aerosols.
class SolarAureole
{
public:
    SolarAureole(Vector3 sunDirection, SDL_FColor sunTint);
    bool Visible() const { return strength > 0.0; }
    SDL_FColor Brighten(SDL_FColor sky, Vector3 unitRay) const;

private:
    Vector3 direction;
    SDL_FColor linearTint;
    double strength;
};
