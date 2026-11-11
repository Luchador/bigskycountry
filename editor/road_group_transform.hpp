#pragma once

#include "world_document.hpp"

// Only authored poses are retained. A junction can also affect the ends of roads outside the selection.
struct RoadGroupPose
{
    struct RoadPose
    {
        std::vector<RoadPoint> points;
        Vector3 rotation;
        bool operator==(const RoadPose&) const = default;
    };

    std::map<int, RoadPose> roads;
    std::map<int, Intersection> junctions;
    bool operator==(const RoadGroupPose&) const = default;

    static RoadGroupPose Capture(const WorldDocument& world, const std::set<SceneKey>& selection);
    void Restore(WorldDocument& world) const;
    bool Transform(WorldDocument& world, const std::set<SceneKey>& selection, Vector3 pivot,
                   Vector3 offset, Vector3 axis, double angle) const;
};
