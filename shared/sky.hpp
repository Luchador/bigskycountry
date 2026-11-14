#pragma once

#include "moon.hpp"
#include "sun.hpp"
#include "world_lighting.hpp"
#include <SDL3/SDL.h>
#include <vector>

class CameraView;

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
    bool Draw(SDL_Renderer* renderer, const CameraView& camera, double timeOfDayMinutes);
    const WorldLighting& Lighting() const { return environment.lights; }

private:
    Sun sun;
    Moon moon;
    void UpdateGrid();
    void BuildGradient(const CameraView& camera, int width, int height);
    SkyEnvironment environment{};
    double lastTime = -1;
    Vector3 lastDirection{};
    int lastWidth = 0, lastHeight = 0;
    std::vector<double> sampleX;
    std::vector<double> sampleY;
    std::vector<SDL_Vertex> vertices;
    std::vector<int> indices;
    int columns = 0;
    int rows = 0;
};
