#pragma once

#include "intersection.hpp"
#include "road_overlay.hpp"

struct RoadIntersectionInsertion
{
    Intersection junction;
    std::vector<Road> approaches; // One at an endpoint, two inside a road, in spline order.
};

// A control-point hit uses its exact authored position. A pavement or spline hit projects onto the centerline.
struct RoadSplineLocation { int segment; double t; };
std::optional<RoadSplineLocation> LocateRoadHit(const Road& road, RoadPick hit, Vector3 position);
std::optional<RoadIntersectionInsertion> PlanRoadIntersection(const Road& road, RoadSplineLocation at);
