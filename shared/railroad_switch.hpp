#pragma once

#include "road.hpp"

struct RailroadSwitchPlan
{
    std::vector<RoadPoint> hostPoints;
    Road branch;
};

// Left/right is viewed along direction. One branch is permitted per station.
std::optional<RailroadSwitchPlan> PlanRailroadSwitch(const Road& host, int segment, double t,
                                                    int side, int direction, int station);
// Frame at the rail head, matching the railroad mesh builder on grades.
MeshTransform RailroadSwitchFrame(const Road& road, std::size_t point, int direction);

// Height seams where the joined embankment footprints meet surrounding terrain.
std::vector<std::array<Vector3, 2>> RailroadSwitchTerrainSeams(const Road& host, const Road& branch);
