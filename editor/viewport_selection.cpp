#include "viewport_selection.hpp"
#include "spatial_index.hpp"
#include "view_geometry.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"
#include <algorithm>

namespace
{
std::array<Vector3, 8> Corners(const Bounds3& bounds)
{
    std::array<Vector3, 8> corners;
    int i = 0;
    for (double x : {bounds.min.x, bounds.max.x})
        for (double y : {bounds.min.y, bounds.max.y})
            for (double z : {bounds.min.z, bounds.max.z}) corners[i++] = {x, y, z};
    return corners;
}
}

ViewportSelectionBox::ViewportSelectionBox(const CameraView& view, SDL_Rect viewport, SDL_FPoint a, SDL_FPoint b) : camera(view)
{
    const double focal = .5 * std::min(viewport.w, viewport.h);
    const double cx = .5 * viewport.w, cy = .5 * viewport.h;
    planes = {{{{0, 0, 1}, worldNearDepth},
               {{focal, 0, cx - std::min(a.x, b.x)}, 0},
               {{-focal, 0, std::max(a.x, b.x) - cx}, 0},
               {{0, -focal, cy - std::min(a.y, b.y)}, 0},
               {{0, focal, std::max(a.y, b.y) - cy}, 0}}};
}

bool ViewportSelectionBox::Contains(Vector3 point) const
{
    const auto p = camera.WorldToView(point);
    return std::ranges::all_of(planes, [&](const Plane& plane) { return Math::Dot(p, plane.normal) >= plane.offset; });
}

bool ViewportSelectionBox::Contains(const Bounds3& bounds) const
{
    return !bounds.Empty() && std::ranges::all_of(Corners(bounds), [&](Vector3 p) { return Contains(p); });
}

bool ViewportSelectionBox::Intersects(const Bounds3& bounds) const
{
    if (bounds.Empty()) return false;
    auto corners = Corners(bounds);
    for (auto& p : corners) p = camera.WorldToView(p);
    for (const auto& plane : planes)
        if (std::ranges::all_of(corners, [&](Vector3 p) { return Math::Dot(p, plane.normal) < plane.offset; })) return false;
    return true;
}

bool ViewportSelectionBox::Intersects(Vector3 a, Vector3 b, Vector3 c) const
{
    ViewGeometry::Polygon polygon{};
    polygon.count = 3;
    polygon.points[0] = camera.WorldToView(a);
    polygon.points[1] = camera.WorldToView(b);
    polygon.points[2] = camera.WorldToView(c);
    for (const auto& plane : planes)
    {
        polygon = ViewGeometry::Clip(polygon, plane.normal, plane.offset);
        if (polygon.count == 0) return false;
    }
    return true;
}

bool ViewportSelectionBox::Intersects(const MeshInstance& instance) const
{
    if (!instance.mesh || instance.mesh->bounds.Empty()) return Contains(instance.transform.origin);
    const auto bounds = WorldBounds(instance.mesh->bounds, instance.transform);
    if (!Intersects(bounds)) return false;
    if (Contains(bounds)) return true;
    const auto& mesh = *instance.mesh;
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
        if (Intersects(instance.transform.Point(mesh.vertices[mesh.indices[i]].position),
                       instance.transform.Point(mesh.vertices[mesh.indices[i+1]].position),
                       instance.transform.Point(mesh.vertices[mesh.indices[i+2]].position))) return true;
    return false;
}

std::set<SceneKey> SelectViewportBox(const WorldDocument& world, const ViewportSelectionBox& box)
{
    std::set<SceneKey> selected;
    const auto visible = [&](SceneKey key) { return !world.hierarchy.EditorHidden(key); };
    const auto road = [&](const RoadGeometry& geometry)
    {
        if (!box.Intersects(geometry.bounds)) return false;
        if (box.Contains(geometry.bounds)) return true;
        const auto triangles = [&](const auto& list)
        {
            return std::ranges::any_of(list, [&](const auto& t) { return box.Intersects(t[0].position, t[1].position, t[2].position); });
        };
        if (triangles(geometry.pavementTriangles) || triangles(geometry.edgeTriangles) || triangles(geometry.bridgeTriangles) ||
            std::ranges::any_of(geometry.transitionStrips, [&](const auto& strip) { return triangles(strip.second); })) return true;
        for (const auto& segment : geometry.railroadSegments)
        {
            if (triangles(segment.rails)) return true;
            for (const auto& run : segment.ties)
            {
                Bounds3 bounds;
                const auto radius = Vector3{run.Radius(), run.Radius(), run.Radius()};
                bounds.Include(run.Center() - radius); bounds.Include(run.Center() + radius);
                if (!box.Intersects(bounds)) continue;
                for (int i = 0; i < run.count; ++i)
                    if (box.Intersects(MeshInstance{RailroadTieMesh(), run.At(i)})) return true;
            }
        }
        return false;
    };
    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
    {
        const SceneKey key{SceneKind::Road, i};
        if (!visible(key)) continue;
        const auto& item = world.roads[i];
        if (road(item) || (item.bounds.Empty() && std::ranges::any_of(item.points,
            [&](const auto& point) { return box.Contains(point.position); }))) selected.insert(key);
    }
    for (const auto& [id, item] : world.intersections)
    {
        const SceneKey key{SceneKind::Intersection, id};
        if (!visible(key)) continue;
        bool hit = box.Contains(item.position);
        for (int side = 0; side < IntersectionAnchor::sideCount && !hit; ++side)
            if (item.HasAnchor(side)) hit = box.Contains(item.AnchorPosition(side));
        const auto surface = world.IntersectionSurfaces().find(id);
        if (hit || (surface != world.IntersectionSurfaces().end() && road(surface->second.geometry))) selected.insert(key);
    }
    const auto mesh = [&](SceneKey key, const MeshInstance& instance)
    {
        if (visible(key) && box.Intersects(instance)) selected.insert(key);
    };
    for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
    {
        const auto& chunk = *world.Terrain()[i];
        // Both the water surface and exposed banks/bed belong to the water object.
        const SceneKey key{SceneKind::Terrain, i};
        mesh(key, chunk.Instance());
        if (chunk.HasWater() && !selected.contains(key)) mesh(key, chunk.Water());
    }
    for (int i = 0; i < static_cast<int>(world.Objects().size()); ++i) mesh({SceneKind::Object, i}, world.Objects()[i]->Instance());
    for (int i = 0; i < static_cast<int>(world.meshes.size()); ++i) mesh({SceneKind::Mesh, i}, world.meshes[i]);
    return selected;
}

std::set<int> SelectWaterBox(const TerrainChunk& chunk, const ViewportSelectionBox& box)
{
    std::set<int> selected;
    if (!chunk.EditingWater()) return selected;
    if (const auto water = chunk.WaterSource())
        for (int i = 0; i < WaterPointCount(*water); ++i)
            if (box.Contains(chunk.Instance().transform.Point(WaterPoint(*water, i)))) selected.insert(i);
    return selected;
}
