#pragma once

#include "math.hpp"
#include <SDL3/SDL_pixels.h>
#include <SDL3/SDL_rect.h>

#include <array>
#include <cstddef>
#include <compare>
#include <cstdint>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <vector>

enum class ThreeLaneLayout
{
    TwoNorthbound = 0,
    TwoSouthbound = 1,
    CenterTurn = 2
};

enum class RoadEdge 
{ 
    None = 0, 
    Transition = 1, 
    Curb = 2 
};

namespace RoadEdgeColor
{
    inline constexpr SDL_FColor transitionInner{1, 0, 0, 1}, transitionOuter{0, 0, 0, 1};
    inline constexpr SDL_FColor apron{0.69f, 0.69f, 0.69f, 1};
    inline constexpr SDL_FColor riser{0.56f, 0.56f, 0.56f, 1};
    inline constexpr SDL_FColor top{0.78f, 0.78f, 0.78f, 1};
}

enum class RoadMedian { Concrete, Grass };

struct RoadStyle
{
    static constexpr double dashPeriod = 12.0; // One painted dash plus its gap, in meters.
    static constexpr double laneWidth = 5.0;
    static constexpr double shoulderWidth = 5.0;
    static constexpr double innerShoulderWidth = 1.0;
    static constexpr double minSeparation = 2.0 * innerShoulderWidth;
    static constexpr double stripeHeight = 0.02; // Road paint offset above the pavement.
    static constexpr double stripWidth = 0.40;
    static constexpr double curbHeight = 0.15;
    static constexpr double curbTopWidth = 0.15;
    static constexpr int maxLanes = 10;
    static constexpr const char* defaultMaterial = "surfaces/roads/M_Asphalt01.bmat";
    std::string material = defaultMaterial;
    bool leftShoulder = false;
    bool rightShoulder = false;
    bool curved = false;
    RoadMedian median = RoadMedian::Concrete;
    ThreeLaneLayout threeLaneLayout = ThreeLaneLayout::CenterTurn;
    RoadEdge edge = RoadEdge::None;

    bool operator==(const RoadStyle&) const = default;
    int GetLaneCount() const;
    void SetLaneCount(int count);
    bool CanDivide() const { return lanes % 2 == 0; }
    bool IsDivided() const { return divided; }
    void SetDivided(bool value) { divided = value && CanDivide(); }
    double DividedGap() const { return dividedGap; }
    void SetDividedGap(double meters);
    double HalfSeparation() const { return divided ? dividedGap * 0.5 : 0.0; }
    double HalfMedianWidth() const { return divided ? HalfSeparation() - innerShoulderWidth : 0.0; }
    bool HasCenterTurnLane() const;
    double LeftWidth() const;
    double RightWidth() const;
    double EdgeWidth() const;

private:
    bool divided = false;
    double dividedGap = 4.0; // Distance between opposing travel lanes, including both inner shoulders.
    int lanes = 2; // Total across both directions, including any center turn lane.
};

// Persistent reference to one of a junction's four approaches.
struct IntersectionAnchor
{
    int intersectionId;
    int side; // Index into directions, stored in ROAD records.
    static constexpr int sideCount = 4;
    static constexpr std::array<Vector3, sideCount> directions{{{1, 0, 0}, {0, 0, 1}, {-1, 0, 0}, {0, 0, -1}}};
    bool Valid() const { return intersectionId > 0 && side >= 0 && side < sideCount; }
    auto operator<=>(const IntersectionAnchor&) const = default;
};

struct RoadPoint
{
    bool operator==(const RoadPoint&) const = default;
    Vector3 position;
    std::optional<double> heading; // Radians from +X toward +Z; empty follows the neighboring points.
    double pitch = 0.0; // Radians above the XZ plane for a manual tangent.
    double roll = 0.0; // Radians about the tangent; orients the control point's local Y/Z axes.
    std::optional<IntersectionAnchor> anchor; // Only endpoints can be attached.
    // Bézier handle lengths as fractions of the adjacent chords.
    double incomingHandle = 1.0 / 3.0;
    double outgoingHandle = 1.0 / 3.0;
};

struct RoadGeometry
{
    // Monotonic token shared by all generated road and junction footprints.
    inline static std::uint64_t latestRevision = 0;
    std::uint64_t revision = 0;
    void MarkRebuilt() { revision = ++latestRevision; }

    using Triangle = std::array<Vector3, 3>;
    struct PavementVertex
    {
        Vector3 position;
        SDL_FPoint uv;
    };
    struct EdgeVertex
    {
        Vector3 position;
        SDL_FColor color;
    };

    std::vector<std::array<PavementVertex, 3>> pavementTriangles;
    std::vector<std::array<EdgeVertex, 3>> edgeTriangles; // Vertex-colored outer edges and median surfaces.
    std::vector<Vector3> outline; // Closed perimeter including shoulders, edge treatment and road ends.
    std::vector<Triangle> whiteStripes;
    std::vector<Triangle> yellowStripes;
};

struct RoadMouth
{
    Vector3 anchor, center, outward;
    bool operator==(const RoadMouth&) const = default;
};

struct RoadMouthFrame
{
    Vector3 left, up; // Junction cross-section axes in the road mesh frame.
};

struct Road : RoadGeometry
{
    int id = 0; // Persistent terrain neighbor ID. Drafts have no ID.
    RoadStyle style;
    Vector3 rotation{}; // Radians about X, Y, Z. The first control point is the road pivot.
    std::vector<RoadPoint> points;
    // Generated cut lines at active intersections. The authored spline stays editable all the way to the anchor.
    void SetMouths(const std::array<std::optional<RoadMouth>, 2>& mouths, bool rebuild = false);

    // Includes both endpoints; the span is valid until the road is rebuilt.
    std::span<const Vector3> SegmentSamples(std::size_t segment) const;
    double SplineLengthMeters() const { return splineLengthMeters; }
    Vector3 Tangent(std::size_t index) const;
    double ThreeLaneDividerOffset() const; // Signed distance left of the authored path for a 2+1 layout.
    std::array<Vector3, 3> PointAxes(std::size_t index, bool includeRoll = true) const;
    void RotatePoint(std::size_t index, double heading);
    void RotatePoint(std::size_t index, Vector3 unitAxis, double angle);
    bool SetPose(Vector3 position, Vector3 angles);
    bool TransformControlPoints(Vector3 position, Vector3 angles); // Authored data only, for batch edits.
    // An independent run of surviving segments, including both endpoint control points.
    Road Section(std::size_t first, std::size_t last) const;
    bool SubdivideSegments(const std::set<int>& segments);
    void Rebuild();

private:
    std::array<std::optional<RoadMouth>, 2> mouths;
    void BuildSurface(const std::vector<Vector3>& samples, std::span<const Vector3> tangents,
                      const std::array<std::optional<RoadMouthFrame>, 2>& caps);
    std::vector<Vector3> centerline;
    double splineLengthMeters = 0.0;
    std::vector<std::size_t> pointSamples; // Centerline index of each control point.
};
