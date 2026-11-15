#pragma once
#include <optional>
#include <set>
#include <span>

struct Road;
struct BridgeSettings
{
    double depth = 0.6;       // Vertical deck thickness, in game meters.
    double groundHeight = 0;  // Authored fallback if a lower road is removed.
    std::set<int> lowerRoads; // Stable IDs; a lower-road split adds its surviving sections.
    bool Valid() const;
    bool operator==(const BridgeSettings&) const = default;
};
// Leaves the lower roads and the upper spline untouched. SDL error explains a rejected span.
std::optional<BridgeSettings> PlanOverpass(const Road& upper, const std::set<int>& segments,
                                           std::span<const Road> roads);
