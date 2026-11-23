#pragma once

#include "mesh.hpp"
#include <optional>
#include <vector>

struct Road;
struct RoadGeometry;

// Stable railway ID and normalized XZ stations select one crossing when a pair
// of winding splines intersects more than once. The owner can also be a railroad.
// Only this link is serialized; rail/rail links generate terrain seams only.
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
    std::vector<std::array<TrackSection, 2>> tracks; // Actual rail sweeps through a multi-track slab.
    double levelWidth = 0, approach = 0;
    std::size_t slabFirst = 0, slabLast = 0; // Inclusive rows bounding the concrete pad.
    double slabDepth = 0;                    // Solid sides extend below the unchanged embankment toes.
    bool operator==(const CrossingProfile&) const = default;
};

struct CrossingSite
{
    RailroadCrossingLink link;
    Vector3 position;
};

std::vector<CrossingSite> FindRailroadCrossings(const Road& road, const Road& railway);
std::optional<CrossingProfile> PlanRailroadCrossing(const Road& road, const Road& railway, const CrossingSite& site);
// Vertical terrain seams at the intersecting embankment rims. Neither railway
// mesh changes. Empty optional means the tracks are not an at-grade crossing.
std::optional<std::vector<std::array<Vector3, 2>>> PlanRailroadJunction(
    const Road& first, const Road& second, const CrossingSite& site);
void ApplyRailroadCrossings(Road& road, const std::vector<CrossingProfile>& profiles);
