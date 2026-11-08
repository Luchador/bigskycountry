#pragma once

#include <SDL3/SDL.h>
#include <vector>

class FreeCamera;

class Sky
{
public:
    bool Draw(SDL_Renderer* renderer, const FreeCamera& camera, double timeOfDayMinutes);

private:
    void Resize(int width, int height);
    std::vector<SDL_Vertex> vertices;
    std::vector<int> indices;
    int width = 0;
    int height = 0;
    int columns = 0;
    int rows = 0;
};
