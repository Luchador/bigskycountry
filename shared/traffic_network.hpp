#pragma once

#include "mesh.hpp"
#include <cstdint>
#include <optional>
#include <vector>
#include <unordered_map>

class WorldDocument;

enum class TrafficControl { Stop, Yield, Priority };
enum class TrafficTurn { Straight, Left, Right };

// Derived alongside pavement. Points are in authoring order, never paint-dependent.
struct RoadTrafficPath
{
    int direction = 1, ordinal = 0;
    double width = 5;
    std::vector<Vector3> points;
    std::vector<double> widths; // Per-point directional space; half the pavement on shared one-lane roads.
};

struct TrafficLane
{
    std::uint64_t id = 0; // Road ID + local path ID, scoped to a cooked network revision.
    int roadId = 0, direction = 1, ordinal = 0;
    double width = 5, speedLimit = 13.4112; // Initial test limit: 30 mph, in engine m/s.
    std::vector<Vector3> points; // Always in driving order.
    std::vector<double> widths; // Empty means uniform width (including older cooked worlds).
    std::vector<double> distances; // Derived arc lengths, not serialized.
    Bounds3 bounds; // Derived once; cheap rejection for picking/debug rendering.
    void Measure();
    double Length() const { return distances.empty() ? 0 : distances.back(); }
    Vector3 At(double distance) const;
    double WidthAt(double distance) const;
    Vector3 Tangent(double distance) const;
    struct Projection { double distance = 0, error = 0; Vector3 position; };
    Projection Project(Vector3 point, double from = 0, double to = 1e30) const;

private:
    // Contiguous segment ranges preserve station-limited queries on loops.
    // Derived by Measure(), alongside arc lengths; never written to the cook.
    struct PathNode
    {
        Bounds3 bounds;
        std::size_t first = 0, last = 0, left = 0, right = 0;
    };
    std::vector<PathNode> pathNodes;
    std::size_t BuildPathNode(std::size_t first, std::size_t last);
};

struct TrafficMovement
{
    std::uint64_t id = 0, from = 0, to = 0;
    int junction = 0, fromSide = 0, toSide = 0;
    TrafficControl control = TrafficControl::Stop;
    TrafficTurn turn = TrafficTurn::Straight;
    TrafficLane path; // Only the section between the two road mouths.
    std::vector<std::size_t> conflicts; // Derived, sorted movement indices.
};

enum class TrafficLinkKind { Continue, Exit, Merge };

// Directed, authored road/ramp connections. Stations may be inside the host lane.
struct TrafficLink
{
    std::uint64_t from = 0, to = 0;
    double fromDistance = 0, toDistance = 0;
    TrafficLinkKind kind = TrafficLinkKind::Continue;
    TrafficLane path;
};

class TrafficNetwork
{
public:
    std::vector<TrafficLane> lanes;
    std::vector<TrafficMovement> movements;
    std::vector<TrafficLink> links;
    const TrafficLane* FindLane(std::uint64_t id) const;
    const std::vector<std::size_t>& Outgoing(std::uint64_t lane) const;
    const std::vector<std::size_t>& Connections(std::uint64_t lane) const;
    const std::vector<std::size_t>& Family(const TrafficLane& lane) const;
    // Only neighboring same-direction lanes on the same road; never across a median.
    bool LaneChange(const TrafficLane& from, double start, const TrafficLane& to,
                    double length, TrafficLane& path, double& destination) const;
    void BuildHighways(const WorldDocument& world);
    void BuildMovements(const WorldDocument& world);
    static TrafficNetwork Build(const WorldDocument& world);
    // Also used to validate cooked data before publishing it to the game.
    bool Validate();
    struct Pick { std::size_t lane; double distance, error; };
    std::optional<Pick> Nearest(Vector3 position, double radius) const;
private:
    std::unordered_map<std::uint64_t, std::size_t> laneIndex;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> outgoing, connections, families;
    static std::uint64_t FamilyKey(const TrafficLane& lane)
    { return (std::uint64_t(lane.roadId) << 1) | (lane.direction > 0 ? 1u : 0u); }
};
