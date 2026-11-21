#pragma once

#include "road.hpp"
#include "road_paint.hpp"
#include <bitset>

enum class IntersectionPaint { Standard, NorthSouth, EastWest };
enum class IntersectionShape { Standard, Y };

// Authoring data for a flat junction. Its center is at world elevation position.y.
struct Intersection
{
    Vector3 position{};
    double cornerRadius = 6.0; // meters
    std::string material = RoadStyle::defaultMaterial;
    std::bitset<IntersectionAnchor::sideCount> stopLines{0b1111}; // One setting per anchor, including free anchors.
    static constexpr double stopLineWidth = 0.6; // Meters along the direction of travel.
    static constexpr double anchorDistance = 10.0; // Approach centerlines, in meters from the junction center.
    IntersectionPaint paint = IntersectionPaint::Standard;
    double yaw = 0; // Radians around world Y.
    bool pavedThrough = false; // Keep pavement between opposite arms of an intersection inserted in a road.
    static constexpr double maxSkew = Math::DegreesToRadians(45.0);
    std::array<double, IntersectionAnchor::sideCount> skew{}; // Per-anchor yaw relative to its nominal outward axis.
    std::bitset<2> alignedApproaches{}; // +X/-X and +Z/-Z share a straight line through the center.
    IntersectionShape shape = IntersectionShape::Standard;
    double size = 60.0; // Y: center to each anchor, in meters; lane widths are not scaled.
    bool island = true;
    static constexpr double minSize = 10.0, maxSize = 2000.0;
    bool HasAnchor(int side) const { return side >= 0 && side < IntersectionAnchor::sideCount && (shape != IntersectionShape::Y || side != 1); }
    double AnchorDistance() const { return shape == IntersectionShape::Y ? size : anchorDistance; }
    Vector3 ApproachDirection(int side) const { return Directions(yaw + skew.at(side), shape).at(side); }
    static std::array<Vector3, IntersectionAnchor::sideCount> Directions(double yaw, IntersectionShape shape = IntersectionShape::Standard);
    bool AlignsOpposite(int side) const { return shape == IntersectionShape::Standard && HasAnchor(side) && alignedApproaches.test(side % 2); }
    bool ValidAlignment() const;
    Vector3 AnchorPosition(int side) const
    {
        return position + Directions(yaw + (AlignsOpposite(side) ? skew.at(side) : 0), shape).at(side) * AnchorDistance();
    }
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
    RoadPaintProfile paint;
    static IntersectionApproach FromRoad(const Road& road, bool first, bool paintStopLine);
    bool operator==(const IntersectionApproach&) const = default;
};

struct IntersectionLayout
{
    Vector3 center;
    double radius;
    std::array<std::optional<IntersectionApproach>, IntersectionAnchor::sideCount> arms{};
    IntersectionPaint paint = IntersectionPaint::Standard;
    double yaw = 0;
    std::array<double, IntersectionAnchor::sideCount> skew{};
    std::bitset<2> alignedApproaches{}; // +X/-X and +Z/-Z share a straight line through the center.
    IntersectionShape shape = IntersectionShape::Standard;
    double size = 60.0;
    bool island = true;
    double AnchorDistance() const { return shape == IntersectionShape::Y ? size : Intersection::anchorDistance; }
    Vector3 AnchorPosition(int side) const
    {
        const bool aligned = shape == IntersectionShape::Standard && alignedApproaches.test(side % 2);
        return center + Intersection::Directions(yaw + (aligned ? skew.at(side) : 0), shape).at(side) * AnchorDistance();
    }
    bool operator==(const IntersectionLayout&) const = default;
};

struct IntersectionSurface
{
    IntersectionLayout layout{};
    // Anchor distance plus the rendered mouth's advance along its skewed approach.
    std::array<double, IntersectionAnchor::sideCount> setbacks{};
    RoadGeometry geometry;
    // A true-width cut perpendicular to the skewed road, beyond its authoring anchor.
    RoadMouth Mouth(int side) const;
    void Build(const IntersectionLayout& input);
};
