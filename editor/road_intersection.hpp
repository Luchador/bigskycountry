#pragma once

#include "intersection.hpp"
#include "road_overlay.hpp"

struct RoadIntersectionInsertion
{
    Intersection junction;
    std::vector<Road> approaches; // Before and after the junction, with their final mouths already tested.
};

// A control-point hit uses its exact authored position. A pavement or spline hit projects onto the centerline.
std::optional<RoadIntersectionInsertion> PlanRoadIntersection(const Road& road, RoadPick hit, Vector3 position);
