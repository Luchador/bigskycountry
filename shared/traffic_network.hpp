#pragma once

#include "mesh.hpp"
#include <cstdint>
#include <optional>
#include <vector>

class WorldDocument;

// Derived alongside pavement. Points are in authoring order, never paint-dependent.
struct RoadTrafficPath
{
    int direction = 1, ordinal = 0;
    double width = 5;
    std::vector<Vector3> points;
};

struct TrafficLane
{
    std::uint64_t id = 0; // Road ID + local path ID, scoped to a cooked network revision.
    int roadId = 0, direction = 1, ordinal = 0;
    double width = 5, speedLimit = 13.4112; // Initial test limit: 30 mph, in engine m/s.
    std::vector<Vector3> points; // Always in driving order.
    std::vector<double> distances; // Derived arc lengths, not serialized.
    Bounds3 bounds; // Derived once; cheap rejection for picking/debug rendering.
    void Measure();
    double Length() const { return distances.empty() ? 0 : distances.back(); }
    Vector3 At(double distance) const;
    Vector3 Tangent(double distance) const;
    struct Projection { double distance = 0, error = 0; Vector3 position; };
    Projection Project(Vector3 point, double from = 0, double to = 1e30) const;
};

class TrafficNetwork
{
public:
    std::vector<TrafficLane> lanes;
    static TrafficNetwork Build(const WorldDocument& world);
    // Also used to validate cooked data before publishing it to the game.
    bool Validate();
    struct Pick { std::size_t lane; double distance, error; };
    std::optional<Pick> Nearest(Vector3 position, double radius) const;
};
