#pragma once

#include "math.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

struct RoadStyle
{
    static constexpr double laneWidth = 5.0;
    static constexpr double shoulderWidth = 5.0;
    int lanes = 2; // Total across both directions.
    bool leftShoulder = false;
    bool rightShoulder = false;
    bool curved = false;

    double LeftWidth() const;
    double RightWidth() const;
};

struct RoadPoint
{
    Vector3 position;
    std::optional<double> heading; // Radians from +X toward +Z; empty follows the neighboring points.
};

struct Road
{
    RoadStyle style;
    std::vector<RoadPoint> points;
    std::vector<std::array<Vector3, 3>> triangles;

    Vector3 Tangent(std::size_t index) const;
    void RotatePoint(std::size_t index, double heading);
    void Rebuild();
};

class RoadMap
{
public:
    std::vector<Road> roads;
    bool Open(const char* documentPath);
    bool Save() const;

private:
    std::string path;
};
