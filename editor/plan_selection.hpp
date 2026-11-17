#pragma once

#include "mesh.hpp"
#include "scene_hierarchy.hpp"
#include <set>

class WorldDocument;

// Top-down selection ignores elevation. Corners may arrive in any drag direction.
struct PlanSelectionRect
{
    double minX, minZ, maxX, maxZ;
    PlanSelectionRect(Vector3 a, Vector3 b);
    bool Contains(Vector3 point) const;
    bool Contains(const Bounds3& bounds) const;
    bool Intersects(const Bounds3& bounds) const;
    bool Intersects(Vector3 a, Vector3 b) const;
    bool Intersects(Vector3 a, Vector3 b, Vector3 c) const;
};

Bounds3 PlanMeshBounds(const MeshInstance& instance);
bool PlanMeshIntersects(const MeshInstance& instance, const PlanSelectionRect& rectangle);
std::set<SceneKey> SelectPlanBox(const WorldDocument& world, const PlanSelectionRect& rectangle);

enum class BoxSelectionMode { Replace, Add, Subtract };
std::set<SceneKey> CombineBoxSelection(const std::set<SceneKey>& previous,
    const std::set<SceneKey>& hits, BoxSelectionMode mode);
