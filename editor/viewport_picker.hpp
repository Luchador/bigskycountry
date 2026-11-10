#pragma once

#include "mesh.hpp"
#include <limits>

class FreeCamera;
struct Road;

double ViewportFocalLength(SDL_Rect scene);
// World-space ray through the center of a viewport-local pixel, with view-space Z equal to one.
Vector3 ViewportRay(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse);

// Share a picker across scene geometry and editor handles. Each test returns true
// only when it replaces the nearest front-facing hit found so far.
class ViewportPicker
{
public:
    ViewportPicker(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse);
    bool TestTriangle(Vector3 a, Vector3 b, Vector3 c);
    bool TestRoad(const Road& road);
    bool TestMesh(const MeshInstance& instance);
    // Valid after a successful test.
    Vector3 HitPosition() const { return eye + ray * depth; }

    const FreeCamera& camera;
    const SDL_FPoint mouse;
    const Vector3 eye, ray;

private:
    double depth = std::numeric_limits<double>::infinity();
};
