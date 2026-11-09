#pragma once

#include "math.hpp"
#include <SDL3/SDL_pixels.h>

#include <array>
#include <cstddef>
#include <optional>
#include <span>
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

struct RoadStyle
{
    static constexpr double laneWidth = 5.0;
    static constexpr double shoulderWidth = 5.0;
    static constexpr double stripWidth = 0.40;
    static constexpr double curbHeight = 0.15;
    static constexpr double curbTopWidth = 0.15;
    static constexpr int maxLanes = 10;
    bool leftShoulder = false;
    bool rightShoulder = false;
    bool curved = false;
    bool divided = false;
    double dividedGap = 4.0;
    ThreeLaneLayout threeLaneLayout = ThreeLaneLayout::CenterTurn;
    RoadEdge edge = RoadEdge::None;

    bool operator==(const RoadStyle&) const = default;
    int GetLaneCount() const;
    void SetLaneCount(int count);
    bool HasCenterTurnLane() const;
    double LeftWidth() const;
    double RightWidth() const;
    double EdgeWidth() const;

private:
    int lanes = 2; // Total across both directions, including any center turn lane.
};

struct RoadPoint
{
    Vector3 position;
    std::optional<double> heading; // Radians from +X toward +Z; empty follows the neighboring points.
    double pitch = 0.0; // Radians above the XZ plane for a manual tangent.
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
    std::vector<RoadPoint> points;
    std::vector<Triangle> triangles;
    std::vector<std::array<EdgeVertex, 3>> edgeTriangles;
    std::vector<Vector3> outline; // Closed perimeter including shoulders, edge treatment and road ends.
    std::vector<Triangle> whiteStripes;
    std::vector<Triangle> yellowStripes;

    // Includes both endpoints; the span is valid until the road is rebuilt.
    std::span<const Vector3> SegmentSamples(std::size_t segment) const;
    Vector3 Tangent(std::size_t index) const;
    void RotatePoint(std::size_t index, double heading);
    void RotatePoint(std::size_t index, Vector3 unitAxis, double angle);
    void Rebuild();

private:
    void BuildSurface(const std::vector<Vector3>& samples);
    std::vector<Vector3> centerline;
    std::vector<std::size_t> pointSamples; // Centerline index of each control point.
};
