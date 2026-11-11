#pragma once

#include "moon.hpp"
#include "sun.hpp"
#include "world_lighting.hpp"
#include <SDL3/SDL.h>
#include <vector>

class FreeCamera;

struct SkyPalette
{
    SDL_FColor zenith, oppositeHorizon, sunwardHorizon;
};

struct SkyEnvironment
{
    SkyPalette colors;
    SDL_FColor sunTint;
    WorldLighting lights;
    static SkyEnvironment At(double timeOfDayMinutes);
};

class Sky
{
public:
    // Use one Sky per renderer and destroy it before the renderer.
    bool Draw(SDL_Renderer* renderer, const FreeCamera& camera, const SkyEnvironment& environment);

private:
    Sun sun;
    Moon moon;
    void UpdateGrid();
    std::vector<double> sampleX;
    std::vector<double> sampleY;
    std::vector<SDL_Vertex> vertices;
    std::vector<int> indices;
    int columns = 0;
    int rows = 0;
};
