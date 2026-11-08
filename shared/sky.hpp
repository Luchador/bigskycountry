#pragma once

#include "sun.hpp"
#include <SDL3/SDL.h>
#include <vector>

class FreeCamera;

class Sky
{
public:
    // Use one Sky per renderer and destroy it before the renderer.
    bool Draw(SDL_Renderer* renderer, const FreeCamera& camera, double timeOfDayMinutes);

private:
    Sun sun;
    void UpdateGrid();
    std::vector<double> sampleX;
    std::vector<double> sampleY;
    std::vector<SDL_Vertex> vertices;
    std::vector<int> indices;
    int columns = 0;
    int rows = 0;
};
