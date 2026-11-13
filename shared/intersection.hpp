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
    double yaw = 0; // Radians around world Y.
    bool pavedThrough = false; // Keep pavement between opposite arms of an intersection inserted in a road.
    static std::array<Vector3, IntersectionAnchor::sideCount> Directions(double yaw);
    Vector3 AnchorPosition(int side) const { return position + Directions(yaw).at(side) * anchorDistance; }
    std::array<Vector3, IntersectionAnchor::sideCount> AnchorPositions() const;

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
    std::string transitionMaterial;
    static IntersectionApproach FromRoad(const Road& road, bool first, bool paintStopLine);
    bool operator==(const IntersectionApproach&) const = default;
};

struct IntersectionLayout
{
    Vector3 center;
    double radius;
    std::array<std::optional<IntersectionApproach>, IntersectionAnchor::sideCount> arms{};
    double yaw = 0;
    bool operator==(const IntersectionLayout&) const = default;
};

struct IntersectionSurface
{
    IntersectionLayout layout{};
    std::array<double, IntersectionAnchor::sideCount> setbacks{};
    RoadGeometry geometry;
    void Build(const IntersectionLayout& input);
};
