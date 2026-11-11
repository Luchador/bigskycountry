#pragma once

#include "math.hpp"
#include <SDL3/SDL_pixels.h>

#include <array>
#include <cstddef>
#include <optional>
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

enum class RoadMedian { Concrete, Grass };

struct RoadStyle
{
    static constexpr double dashPeriod = 12.0; // One painted dash plus its gap, in meters.
    static constexpr double laneWidth = 5.0;
    static constexpr double shoulderWidth = 5.0;
    static constexpr double innerShoulderWidth = 1.0;
    static constexpr double minSeparation = 2.0 * innerShoulderWidth;
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

struct RoadPoint
{
    Vector3 position;
    std::optional<double> heading; // Radians from +X toward +Z; empty follows the neighboring points.
    double pitch = 0.0; // Radians above the XZ plane for a manual tangent.
    double roll = 0.0; // Radians about the tangent; orients the control point's local Y/Z axes.
};

struct Road
{
    using Triangle = std::array<Vector3, 3>;
    struct EdgeVertex
    {
        Vector3 position;
        SDL_FColor color;
    };

    RoadStyle style;
    Vector3 rotation{}; // Radians about X, Y, Z. The first control point is the road pivot.
    std::vector<RoadPoint> points;
    std::vector<Triangle> triangles;
    std::vector<std::array<EdgeVertex, 3>> edgeTriangles; // Vertex-colored outer edges and median surfaces.
    std::vector<Vector3> outline; // Closed perimeter including shoulders, edge treatment and road ends.
    std::vector<Triangle> whiteStripes;
    std::vector<Triangle> yellowStripes;

    // Includes both endpoints; the span is valid until the road is rebuilt.
    std::span<const Vector3> SegmentSamples(std::size_t segment) const;
    Vector3 Tangent(std::size_t index) const;
    std::array<Vector3, 3> PointAxes(std::size_t index, bool includeRoll = true) const;
    void RotatePoint(std::size_t index, double heading);
    void RotatePoint(std::size_t index, Vector3 unitAxis, double angle);
    bool SetPose(Vector3 position, Vector3 angles);
    // An independent run of surviving segments, including both endpoint control points.
    Road Section(std::size_t first, std::size_t last) const;
    void Rebuild();

private:
    void BuildSurface(const std::vector<Vector3>& samples, std::span<const Vector3> tangents);
    std::vector<Vector3> centerline;
    std::vector<std::size_t> pointSamples; // Centerline index of each control point.
};
