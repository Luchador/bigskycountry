#include "plan_selection.hpp"
#include "world_document.hpp"
#include "spatial_index.hpp"
#include <algorithm>
#include <cmath>

PlanSelectionRect::PlanSelectionRect(Vector3 a, Vector3 b)
    : minX(std::min(a.x, b.x)), minZ(std::min(a.z, b.z)), maxX(std::max(a.x, b.x)), maxZ(std::max(a.z, b.z)) {}

bool PlanSelectionRect::Contains(Vector3 p) const
{ return p.x >= minX && p.x <= maxX && p.z >= minZ && p.z <= maxZ; }
bool PlanSelectionRect::Contains(const Bounds3& b) const
{ return !b.Empty() && Contains(b.min) && Contains(b.max); }
bool PlanSelectionRect::Intersects(const Bounds3& b) const
{ return !b.Empty() && b.max.x >= minX && b.min.x <= maxX && b.max.z >= minZ && b.min.z <= maxZ; }

bool PlanSelectionRect::Intersects(Vector3 a, Vector3 b) const
{
    double low = 0, high = 1;
    const auto clip = [&](double origin, double delta, double minimum, double maximum) {
        if (delta == 0) return origin >= minimum && origin <= maximum;
        double enter = (minimum - origin) / delta, leave = (maximum - origin) / delta;
        if (enter > leave) std::swap(enter, leave);
        low = std::max(low, enter); high = std::min(high, leave);
        return low <= high;
    };
    return clip(a.x, b.x - a.x, minX, maxX) && clip(a.z, b.z - a.z, minZ, maxZ);
}

bool PlanSelectionRect::Intersects(Vector3 a, Vector3 b, Vector3 c) const
{
    if (Contains(a) || Contains(b) || Contains(c) || Intersects(a, b) || Intersects(b, c) || Intersects(c, a)) return true;
    // The box may lie completely inside a large triangle. Degenerate projections
    // (vertical walls, poles) are handled by their edges above, not by this test.
    const auto cross = [](Vector3 u, Vector3 v) { return u.x * v.z - u.z * v.x; };
    if (cross(b - a, c - a) == 0) return false;
    const Vector3 p{minX, 0, minZ};
    const double x = cross(b - a, p - a), y = cross(c - b, p - b), z = cross(a - c, p - c);
    return (x >= 0 && y >= 0 && z >= 0) || (x <= 0 && y <= 0 && z <= 0);
}

Bounds3 PlanMeshBounds(const MeshInstance& instance)
{
    if (instance.mesh && !instance.mesh->bounds.Empty()) return WorldBounds(instance.mesh->bounds, instance.transform);
    Bounds3 bounds; bounds.Include(instance.transform.origin); return bounds;
}

bool PlanMeshIntersects(const MeshInstance& instance, const PlanSelectionRect& rectangle)
{
    const auto bounds = PlanMeshBounds(instance);
    if (!rectangle.Intersects(bounds)) return false;
    if (rectangle.Contains(bounds)) return true;
    if (!instance.mesh) return rectangle.Contains(instance.transform.origin);
    const auto& mesh = *instance.mesh;
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
        if (rectangle.Intersects(instance.transform.Point(mesh.vertices[mesh.indices[i]].position),
                                 instance.transform.Point(mesh.vertices[mesh.indices[i + 1]].position),
                                 instance.transform.Point(mesh.vertices[mesh.indices[i + 2]].position))) return true;
    return false;
}

std::set<SceneKey> SelectPlanBox(const WorldDocument& world, const PlanSelectionRect& rectangle)
{
    std::set<SceneKey> selected;
    const auto visible = [&](SceneKey key) { return !world.hierarchy.EditorHidden(key); };
    const auto road = [&](const RoadGeometry& geometry) {
        if (!rectangle.Intersects(geometry.bounds)) return false;
        if (rectangle.Contains(geometry.bounds)) return true;
        const auto triangles = [&](const auto& list) {
            return std::ranges::any_of(list, [&](const auto& t) {
                return rectangle.Intersects(t[0].position, t[1].position, t[2].position);
            });
        };
        return triangles(geometry.pavementTriangles) || triangles(geometry.edgeTriangles) || triangles(geometry.bridgeTriangles) ||
            std::ranges::any_of(geometry.transitionStrips, [&](const auto& strip) { return triangles(strip.second); });
    };
    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
    {
        const SceneKey key{SceneKind::Road, i};
        if (!visible(key)) continue;
        const auto& item = world.roads[i];
        if (road(item) || (item.bounds.Empty() && std::ranges::any_of(item.points,
            [&](const auto& point) { return rectangle.Contains(point.position); }))) selected.insert(key);
    }
    for (const auto& [id, item] : world.intersections)
    {
        const SceneKey key{SceneKind::Intersection, id};
        if (!visible(key)) continue;
        bool hit = rectangle.Contains(item.position);
        for (int side = 0; side < IntersectionAnchor::sideCount && !hit; ++side)
            if (item.HasAnchor(side)) hit = rectangle.Contains(item.AnchorPosition(side));
        const auto surface = world.IntersectionSurfaces().find(id);
        if (hit || (surface != world.IntersectionSurfaces().end() && road(surface->second.geometry))) selected.insert(key);
    }
    const auto mesh = [&](SceneKey key, const MeshInstance& instance) {
        if (visible(key) && PlanMeshIntersects(instance, rectangle)) selected.insert(key);
    };
    for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
    {
        const auto& chunk = *world.Terrain()[i];
        mesh({SceneKind::Terrain, i}, chunk.HasWater() ? chunk.Water() : chunk.Instance());
    }
    for (int i = 0; i < static_cast<int>(world.Objects().size()); ++i) mesh({SceneKind::Object, i}, world.Objects()[i]->Instance());
    for (int i = 0; i < static_cast<int>(world.meshes.size()); ++i) mesh({SceneKind::Mesh, i}, world.meshes[i]);
    return selected;
}

std::set<SceneKey> CombineBoxSelection(const std::set<SceneKey>& previous, const std::set<SceneKey>& hits, BoxSelectionMode mode)
{
    if (mode == BoxSelectionMode::Replace) return hits;
    auto result = previous;
    if (mode == BoxSelectionMode::Add) result.insert(hits.begin(), hits.end());
    else for (auto key : hits) result.erase(key);
    return result;
}
