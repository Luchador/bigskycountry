#pragma once

#include "camera_view.hpp"
#include "mesh.hpp"
#include "scene_hierarchy.hpp"
#include <array>
#include <set>

class WorldDocument;
class TerrainChunk;

// A selection rectangle extruded through the perspective view. Coordinates are
// relative to the viewport; either corner order is allowed. Selection ignores
// occlusion but clips at the camera's near plane.
class ViewportSelectionBox
{
public:
    ViewportSelectionBox(const CameraView& camera, SDL_Rect viewport, SDL_FPoint a, SDL_FPoint b);
    bool Contains(Vector3 point) const;
    bool Contains(const Bounds3& bounds) const;
    bool Intersects(const Bounds3& bounds) const;
    bool Intersects(Vector3 a, Vector3 b, Vector3 c) const;
    bool Intersects(const MeshInstance& instance) const;
private:
    struct Plane { Vector3 normal; double offset; };
    CameraView camera;
    std::array<Plane, 5> planes;
};

std::set<SceneKey> SelectViewportBox(const WorldDocument& world, const ViewportSelectionBox& box);
std::set<int> SelectWaterBox(const TerrainChunk& chunk, const ViewportSelectionBox& box);
