#pragma once
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <span>

struct Road;
class TerrainChunk;
struct BridgeSettings
{
    double depth = 0.6;       // Vertical deck thickness, in game meters.
    double groundHeight = 0;  // Authored fallback if a lower road is removed.
    std::set<int> lowerRoads; // Stable road/railroad IDs; splitting either adds its surviving sections.
    std::set<std::uint64_t> lowerWaterBodies; // Stable terrain placement IDs, never outliner indices.
    bool Valid() const;
    bool operator==(const BridgeSettings&) const = default;
};
// One continuous span over all crossed roads, railroads and water bodies. Leaves
// their geometry and the upper spline untouched. SDL error explains a rejected span.
std::optional<BridgeSettings> PlanOverpass(const Road& upper, const std::set<int>& segments,
                                           std::span<const Road> roads,
                                           std::span<const std::unique_ptr<TerrainChunk>> terrain = {});
