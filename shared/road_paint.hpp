#pragma once

#include "road.hpp"

// Paint cross section looking outward from an intersection, independent of the
// source road's point order. Kept in the layout cache so style edits rebuild paint.
struct RoadPaintProfile
{
    int lanes = 2;
    double laneWidth = RoadStyle::laneWidth;
    double separation = 0;
    double divider = 0;
    bool centerTurn = false;
    int oneWay = 0; // 0 = two-way, +1 = outward, -1 = inward.
    bool enabled = false;
    bool operator==(const RoadPaintProfile&) const = default;
};

// Uses the same stripe widths, dash pattern and mesh builders as ordinary roads.
// Both profiles and mouths face outward; the connecting curve travels A -> B.
void BuildConnectingRoadPaint(RoadGeometry& geometry, const RoadMouth& a, const RoadMouth& b,
                              RoadPaintProfile profileA, RoadPaintProfile profileB);
