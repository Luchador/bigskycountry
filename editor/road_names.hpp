#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>
#include <memory>

class CameraView;
class WorldDocument;

// Editor-only map labels. Text is cached as small textures, independent of road meshes.
class RoadNames
{
public:
    RoadNames();
    ~RoadNames();
    bool Draw(SDL_Renderer* renderer, const WorldDocument& world, const CameraView& camera,
              SDL_Rect scene, bool planView, Vector3 planCenter, double planScale);

private:
    struct Cache;
    std::unique_ptr<Cache> cache;
};
