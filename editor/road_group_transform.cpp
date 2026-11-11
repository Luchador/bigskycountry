#include "road_group_transform.hpp"
#include "transform_inspector.hpp"

#include <cmath>

RoadGroupPose RoadGroupPose::Capture(const WorldDocument& world, const std::set<SceneKey>& selection)
{
    RoadGroupPose result;
    for (const SceneKey key : selection)
        if (key.kind == SceneKind::Intersection)
            result.junctions.emplace(key.index, world.intersections.at(key.index));

    const auto follows = [&](const RoadPoint& p)
    {
        return p.anchor && result.junctions.contains(p.anchor->intersectionId);
    };
    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
    {
        const Road& road = world.roads[i];
        if (selection.contains({SceneKind::Road, i}) || follows(road.points.front()) || follows(road.points.back()))
            result.roads.emplace(i, RoadPose{road.points, road.rotation});
    }
    return result;
}

void RoadGroupPose::Restore(WorldDocument& world) const
{
    for (const auto& [id, item] : junctions) world.intersections.at(id) = item;
    std::set<int> changed;
    for (const auto& [index, pose] : roads)
    {
        Road& road = world.roads.at(index);
        if (road.points == pose.points && road.rotation == pose.rotation) continue;
        road.points = pose.points;
        road.rotation = pose.rotation;
        changed.insert(index);
    }
    world.UpdateIntersections(changed);
}

bool RoadGroupPose::Transform(WorldDocument& world, const std::set<SceneKey>& selection, Vector3 pivot,
                              Vector3 offset, Vector3 axis, double angle) const
{
    if (!std::isfinite(Math::Length(offset)) || !std::isfinite(angle)) return false;
    if (!junctions.empty() && angle != 0 && axis != Vector3{0, 1, 0}) return false;
    RoadGroupPose next = *this;
    const MeshTransform turn = MeshTransform::AxisRotation(axis, angle);
    const auto move = [&](Vector3 p)
    {
        return angle == 0 ? p + offset : pivot + turn.Direction(p - pivot) + offset;
    };

    for (auto& [id, item] : next.junctions)
    {
        item.position = move(item.position);
        if (!std::isfinite(Math::Length(item.position))) return false;
        if (angle != 0) item.yaw = std::remainder(item.yaw + angle, 2.0 * std::numbers::pi);
    }

    for (auto& [index, pose] : next.roads)
    {
        const Road& source = world.roads.at(index);
        const bool selected = selection.contains({SceneKind::Road, index});
        if (selected)
        {
            Road staged;
            staged.style = source.style;
            staged.points = std::move(pose.points);
            staged.rotation = pose.rotation;
            ObjectTransform target{move(staged.points.front().position), pose.rotation};
            target.Rotate(axis, angle);
            if (!staged.TransformControlPoints(target.position, target.rotation)) return false;
            pose.points = std::move(staged.points);
            pose.rotation = target.rotation;
        }

        for (RoadPoint* end : {&pose.points.front(), &pose.points.back()})
        {
            if (!end->anchor) continue;
            const auto ref = *end->anchor;
            if (const auto it = next.junctions.find(ref.intersectionId); it != next.junctions.end())
            {
                end->position = it->second.AnchorPosition(ref.side);
                // A selected road's tangent already turned with the rest of its points.
                if (!selected && end->heading && angle != 0) *end->heading -= angle;
            }
            else if (selected)
            {
                // Match single-road movement when a road is pulled away from a stationary junction.
                const Vector3 anchor = world.intersections.at(ref.intersectionId).AnchorPosition(ref.side);
                if (Math::Length(end->position - anchor) > 1e-8) end->anchor.reset();
                else end->position = anchor;
            }
        }
        if (!ValidRoadSpan(source, pose.points.front().position, pose.points[1].position) ||
            !ValidRoadSpan(source, pose.points.back().position, pose.points[pose.points.size() - 2].position))
            return false;
    }

    // Publish only after all the incident spans have been validated.
    next.Restore(world);
    return true;
}
