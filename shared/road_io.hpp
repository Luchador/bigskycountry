#pragma once

#include "road.hpp"

#include <iosfwd>
#include <cstdint>

// Shared by the world's ROAD section and the editor's in-memory snapshots.
// A malformed record sets failbit. Draft snapshots may have fewer than two points.
namespace RoadIO
{
    inline constexpr std::uint32_t currentVersion = 2;
    void WriteStyle(std::ostream& out, const RoadStyle& style);
    void ReadStyle(std::istream& in, RoadStyle& style, std::uint32_t version = currentVersion);
    void WriteRoad(std::ostream& out, const Road& road);
    Road ReadRoad(std::istream& in, std::uint32_t version = currentVersion);
}
