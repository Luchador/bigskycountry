#pragma once

#include "mesh.hpp"
#include <optional>
#include <vector>

struct Road;
struct RoadGeometry;

// Stable railway ID and normalized XZ stations select one crossing when a pair
// of winding splines intersects more than once. Only this link is serialized.
struct RailroadCrossingLink
{
    int railroadId = 0;
    double roadFraction = 0, railFraction = 0;
    bool Valid() const;
    bool operator==(const RailroadCrossingLink&) const = default;
};

struct TrackSection
{
    Vector3 top, side; // Rail-head center plane and cross-track axis, from the rail sweep.
    bool operator==(const TrackSection&) const = default;
};

struct CrossingProfile
{
    std::vector<TrackSection> rows;
    double levelWidth = 0, approach = 0;
    bool operator==(const CrossingProfile&) const = default;
};

struct CrossingApron
{
    std::vector<Vector3> rim; // Actual road perimeter, including all crossing cut vertices.
    Vector3 center, along;
    double halfLength = 0;
    CrossingProfile profile;
    bool operator==(const CrossingApron&) const = default;
};

struct CrossingSite
{
    RailroadCrossingLink link;
    Vector3 position;
};

std::vector<CrossingSite> FindRailroadCrossings(const Road& road, const Road& railway);
std::optional<CrossingProfile> PlanRailroadCrossing(const Road& road, const Road& railway, const CrossingSite& site);
void ApplyRailroadCrossings(RoadGeometry& road, const std::vector<CrossingProfile>& profiles);
void ApplyCrossingAprons(RoadGeometry& railway, const std::vector<CrossingApron>& aprons);
