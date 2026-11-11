#pragma once

#include "road.hpp"
#include <bitset>

// Authoring data for a flat junction. Its center is at world elevation position.y.
struct Intersection
{
    Vector3 position{};
    double cornerRadius = 6.0; // meters
    std::string material = RoadStyle::defaultMaterial;
    std::bitset<IntersectionAnchor::sideCount> stopLines{0b1111}; // One setting per anchor, including free anchors.
    static constexpr double stopLineWidth = 0.6; // Meters along the direction of travel.
    static constexpr double anchorDistance = 10.0; // Approach centerlines, in meters from the junction center.
    Vector3 AnchorPosition(int side) const { return position + IntersectionAnchor::directions.at(side) * anchorDistance; }

    bool operator==(const Intersection&) const = default;
};

// Distances are measured looking out from the junction, regardless of road point order.
struct IntersectionApproach
{
    double left, right; // Pavement including shoulders and any median.
    RoadEdge edge;
    struct LaneSpan
    {
        double inner, outer; // Signed offsets to the left when looking outward.
        bool operator==(const LaneSpan&) const = default;
    };
    std::optional<LaneSpan> stopLine;
    static IntersectionApproach FromRoad(const Road& road, bool first, bool paintStopLine);
    bool operator==(const IntersectionApproach&) const = default;
};

struct IntersectionLayout
{
    Vector3 center;
    double radius;
    std::array<std::optional<IntersectionApproach>, IntersectionAnchor::sideCount> arms{};
    bool operator==(const IntersectionLayout&) const = default;
};

struct IntersectionSurface
{
    IntersectionLayout layout{};
    std::array<double, IntersectionAnchor::sideCount> setbacks{};
    RoadGeometry geometry;
    void Build(const IntersectionLayout& input);
};
