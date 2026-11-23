#include "traffic_network.hpp"
#include "world_document.hpp"
#include "spatial_index.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

void TrafficLane::Measure()
{
    bounds = {};
    for (auto point : points) bounds.Include(point);
    distances.assign(points.size(), 0);
    for (std::size_t i = 1; i < points.size(); ++i)
        distances[i] = distances[i - 1] + Math::Length(points[i] - points[i - 1]);
}
Vector3 TrafficLane::At(double distance) const
{
    if (points.empty()) return {};
    if (points.size() < 2 || distances.size() != points.size()) return points.front();
    distance = std::clamp(distance, 0.0, Length());
    const auto next = std::upper_bound(distances.begin(), distances.end(), distance);
    const auto i = std::clamp<std::size_t>(next - distances.begin(), 1, points.size() - 1);
    const double span = distances[i] - distances[i - 1];
    return points[i - 1] + (points[i] - points[i - 1]) * (span > 0 ? (distance - distances[i - 1]) / span : 0);
}
Vector3 TrafficLane::Tangent(double distance) const
{
    return Math::Normalize(At(distance + 0.5) - At(distance - 0.5));
}
TrafficLane::Projection TrafficLane::Project(Vector3 point, double from, double to) const
{
    Projection best{0, std::numeric_limits<double>::infinity(), {}};
    if (points.size() < 2 || distances.size() != points.size()) return best;
    const auto first = std::upper_bound(distances.begin(), distances.end(), from);
    for (std::size_t i = std::clamp<std::size_t>(first - distances.begin(), 1, points.size() - 1);
         i < points.size() && distances[i - 1] <= to; ++i)
    {
        const double lo = std::max(from, distances[i - 1]), hi = std::min(to, distances[i]);
        const double span = distances[i] - distances[i - 1];
        if (hi < lo || span < 1e-8) continue;
        const Vector3 delta = points[i] - points[i - 1];
        const double t = std::clamp(Math::Dot(point - points[i - 1], delta) / (span * span),
                                    (lo - distances[i - 1]) / span, (hi - distances[i - 1]) / span);
        const Vector3 p = points[i - 1] + delta * t;
        const double error = Math::Length(point - p);
        if (error < best.error) best = {distances[i - 1] + t * span, error, p};
    }
    return best;
}
TrafficNetwork TrafficNetwork::Build(const WorldDocument& world)
{
    TrafficNetwork network;
    for (const auto& road : world.roads)
    {
        if (road.id <= 0 || road.style.railroad) continue;
        std::uint32_t local = 0;
        for (const auto& source : road.trafficPaths)
        {
            ++local;
            if (source.points.size() < 2) continue;
            TrafficLane lane;
            lane.id = (std::uint64_t(road.id) << 32) | local;
            lane.roadId = road.id; lane.direction = source.direction; lane.ordinal = source.ordinal;
            lane.width = source.width; lane.points = source.points;
            if (lane.direction < 0) std::reverse(lane.points.begin(), lane.points.end());
            lane.Measure();
            if (lane.Length() >= 0.05) network.lanes.push_back(std::move(lane));
        }
    }
    return network;
}
bool TrafficNetwork::Validate()
{
    std::set<std::uint64_t> ids;
    for (auto& lane : lanes)
    {
        if (!lane.id || lane.roadId <= 0 || (lane.id >> 32) != unsigned(lane.roadId) ||
            !ids.insert(lane.id).second || (lane.direction != -1 && lane.direction != 1) || lane.ordinal < 0 || lane.ordinal >= 10 ||
            !std::isfinite(lane.width) || lane.width < 2 || lane.width > 100 || !std::isfinite(lane.speedLimit) ||
            lane.speedLimit <= 0 || lane.speedLimit > 100 || lane.points.size() < 2)
            return SDL_SetError("Invalid cooked traffic lane");
        for (std::size_t i = 0; i < lane.points.size(); ++i)
            if (!std::isfinite(Math::Length(lane.points[i])) || (i && Math::Length(lane.points[i] - lane.points[i - 1]) < 1e-8))
                return SDL_SetError("Invalid traffic path samples");
        lane.Measure();
        if (!std::isfinite(lane.Length()) || lane.Length() < 0.05) return SDL_SetError("Invalid traffic path length");
    }
    return true;
}
std::optional<TrafficNetwork::Pick> TrafficNetwork::Nearest(Vector3 position, double radius) const
{
    std::optional<Pick> best;
    for (std::size_t i = 0; i < lanes.size(); ++i)
    {
        if (BoundsDistanceSquared(lanes[i].bounds, position) > radius * radius) continue;
        const auto p = lanes[i].Project(position);
        if (p.error <= radius) { best = Pick{i, p.distance, p.error}; radius = p.error; }
    }
    return best;
}
