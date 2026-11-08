#pragma once

#include "math.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

enum class ThreeLaneLayout
{
    TwoNorthbound = 0,
    TwoSouthbound = 1,
    CenterTurn = 2
};

struct RoadStyle
{
    static constexpr double laneWidth = 5.0;
    static constexpr double shoulderWidth = 5.0;
    static constexpr int maxLanes = 10;
    bool leftShoulder = false;
    bool rightShoulder = false;
    bool curved = false;
    bool divided = false;
    double dividedGap = 4.0;
    ThreeLaneLayout threeLaneLayout = ThreeLaneLayout::CenterTurn;

    bool operator==(const RoadStyle&) const = default;
    int GetLaneCount() const;
    void SetLaneCount(int count);
    bool HasCenterTurnLane() const;
    double LeftWidth() const;
    double RightWidth() const;

private:
    int lanes = 2; // Total across both directions, including any center turn lane.
};

struct RoadPoint
{
    Vector3 position;
    std::optional<double> heading; // Radians from +X toward +Z; empty follows the neighboring points.
};

struct Road
{
    using Triangle = std::array<Vector3, 3>;

    RoadStyle style;
    std::vector<RoadPoint> points;
    std::vector<Triangle> triangles;
    std::vector<Vector3> outline; // Closed pavement perimeter, including shoulders and end caps.
    std::vector<Triangle> whiteStripes;
    std::vector<Triangle> yellowStripes;

    Vector3 Tangent(std::size_t index) const;
    void RotatePoint(std::size_t index, double heading);
    void Rebuild();

private:
    void BuildStripesAndOutline(const std::vector<Vector3>& samples);
};
