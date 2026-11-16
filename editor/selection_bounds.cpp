#include "selection_bounds.hpp"
#include "edit_history.hpp"

#include <algorithm>

Bounds3 SelectionBounds(const WorldDocument& world, const RoadEditorState& state)
{
    Bounds3 result;
    const auto include = [&](const Bounds3& bounds)
    {
        if (bounds.Empty()) return;
        result.Include(bounds.min);
        result.Include(bounds.max);
    };
    const auto meshBounds = [&](const MeshInstance& instance)
    {
        if (!instance.mesh || instance.mesh->bounds.Empty())
        {
            result.Include(instance.transform.origin);
            return;
        }
        const auto& b = instance.mesh->bounds;
        for (double x : {b.min.x, b.max.x})
            for (double y : {b.min.y, b.max.y})
                for (double z : {b.min.z, b.max.z}) result.Include(instance.transform.Point({x, y, z}));
    };
    const auto point = [&](int road, int index)
    {
        const SceneKey key{SceneKind::Road, road};
        if (!world.Contains(key) || world.hierarchy.EditorHidden(key)) return;
        const auto& points = world.roads[road].points;
        if (index >= 0 && index < static_cast<int>(points.size())) result.Include(points[index].position);
    };
    const auto anchor = [&](IntersectionAnchor ref)
    {
        const SceneKey key{SceneKind::Intersection, ref.intersectionId};
        if (world.Contains(key) && !world.hierarchy.EditorHidden(key) && ref.side >= 0 && ref.side < IntersectionAnchor::sideCount)
            result.Include(world.intersections.at(ref.intersectionId).AnchorPosition(ref.side));
    };

    if (!state.groupEndpoints.empty())
    {
        for (const auto& [road, index] : state.groupEndpoints) point(road, index);
        return result;
    }
    if (state.point >= 0 && state.selected >= 0)
    {
        point(state.selected, state.point);
        for (int index : state.additionalPoints) point(state.selected, index);
        if (state.anchorSelection) anchor(*state.anchorSelection);
        return result;
    }
    if (state.meshSelection.kind == SelectionKind::LakePoint)
    {
        result.Include(Inspect(world, state.meshSelection).value.position);
        return result;
    }
    if (state.meshSelection.kind == SelectionKind::Anchor)
    {
        anchor({state.meshSelection.index, state.meshSelection.point});
        return result;
    }

    const auto keys = state.SelectedRows();
    const auto selected = [&](SceneKey key)
    {
        if (world.hierarchy.EditorHidden(key)) return false;
        if (keys.contains(key) || keys.contains({SceneKind::World, 0})) return true;
        for (int folder = world.hierarchy.Parent(key); folder; folder = world.hierarchy.Folders().at(folder).parent)
            if (keys.contains({SceneKind::Folder, folder})) return true;
        return false;
    };
    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
    {
        if (!selected({SceneKind::Road, i})) continue;
        const auto& road = world.roads[i];
        if (i == state.selected && !state.segments.empty())
        {
            // Frame just the selected spans, including their pavement width.
            const double width = road.MaximumHalfTravelWidth() + road.style.HalfSeparation() + road.style.EdgeWidth() +
                std::max(road.MaximumShoulderWidth(true), road.MaximumShoulderWidth(false));
            for (int segment : state.segments)
            {
                if (segment < 0 || segment + 1 >= static_cast<int>(road.points.size())) continue;
                for (auto p : road.SegmentSamples(segment))
                {
                    result.Include(p - Vector3{width, width, width});
                    result.Include(p + Vector3{width, width, width});
                }
            }
        }
        else if (!road.bounds.Empty()) include(road.bounds);
        else for (const auto& p : road.points) result.Include(p.position);
    }
    for (const auto& [id, intersection] : world.intersections)
    {
        if (!selected({SceneKind::Intersection, id})) continue;
        for (auto p : intersection.AnchorPositions()) result.Include(p);
        const auto surface = world.IntersectionSurfaces().find(id);
        if (surface != world.IntersectionSurfaces().end()) include(surface->second.geometry.bounds);
    }
    for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
        if (selected({SceneKind::Terrain, i})) meshBounds(world.Terrain()[i]->Instance());
    for (int i = 0; i < static_cast<int>(world.Objects().size()); ++i)
        if (selected({SceneKind::Object, i})) meshBounds(world.Objects()[i]->Instance());
    for (int i = 0; i < static_cast<int>(world.meshes.size()); ++i)
        if (selected({SceneKind::Mesh, i})) meshBounds(world.meshes[i]);
    return result;
}
