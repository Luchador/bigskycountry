#pragma once

#include "road.hpp"

class WorldDocument;

struct CurvedConnectorPlan
{
    std::vector<RoadPoint> firstPoints, secondPoints;
    Road connector;
};

bool CanHostCurvedConnector(const Road& road);
std::optional<CurvedConnectorPlan> PlanCurvedConnector(const Road& first, int firstSegment, double firstT,
    const Road& second, int secondSegment, double secondT, int nextStation);
// Move an attached end along its source spline, retaining its tangent and grade.
bool SlideCurvedConnector(WorldDocument& world, int road, int point, Vector3 destination);
std::vector<std::array<Vector3, 2>> CurvedConnectorTerrainSeams(const Road& host, const Road& branch);
