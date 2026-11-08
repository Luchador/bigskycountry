#pragma once

#include "road.hpp"

#include <iosfwd>

// Shared by the world's ROAD section and the editor's in-memory snapshots.
// A malformed record sets failbit. Draft snapshots may have fewer than two points.
namespace RoadIO
{
    void WriteStyle(std::ostream& out, const RoadStyle& style);
    void ReadStyle(std::istream& in, RoadStyle& style);
    void WriteRoad(std::ostream& out, const Road& road);
    Road ReadRoad(std::istream& in);
}
