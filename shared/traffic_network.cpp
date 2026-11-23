#include "traffic_network.hpp"
#include "world_document.hpp"
#include "spatial_index.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <map>

void TrafficLane::Measure()
{
    bounds = {};
    for (auto point : points) bounds.Include(point);
    distances.assign(points.size(), 0);
    for (std::size_t i = 1; i < points.size(); ++i)
        distances[i] = distances[i - 1] + Math::Length(points[i] - points[i - 1]);
    pathNodes.clear();
    if (points.size() > 1) BuildPathNode(1, points.size() - 1);
}
std::size_t TrafficLane::BuildPathNode(std::size_t first, std::size_t last)
{
    const auto index = pathNodes.size();
    pathNodes.push_back({});
    PathNode node;
    node.first = first; node.last = last;
    if (last - first < 8)
    {
        for (auto i = first - 1; i <= last; ++i) node.bounds.Include(points[i]);
    }
    else
    {
        const auto middle = first + (last - first) / 2;
        node.left = BuildPathNode(first, middle);
        node.right = BuildPathNode(middle + 1, last);
        for (auto child : {node.left, node.right})
        {
            node.bounds.Include(pathNodes[child].bounds.min);
            node.bounds.Include(pathNodes[child].bounds.max);
        }
    }
    pathNodes[index] = node;
    return index;
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
    if (points.size() < 2 || distances.size() != points.size() || pathNodes.empty()) return best;
    const auto first = std::clamp<std::size_t>(std::upper_bound(distances.begin(), distances.end(), from) - distances.begin(),
                                              1, points.size() - 1);
    auto bestSegment = points.size();
    const auto visit = [&](const auto& self, std::size_t index) -> void {
        const auto& node = pathNodes[index];
        if (node.last < first || distances[node.first - 1] > to) return;
        const double squaredError = best.error * best.error;
        // A small tolerance keeps pruning conservative at large world coordinates.
        if (BoundsDistanceSquared(node.bounds, point) > squaredError * (1 + 1e-12) + 1e-12) return;
        if (node.left)
        {
            auto near = node.left, far = node.right;
            if (BoundsDistanceSquared(pathNodes[far].bounds, point) < BoundsDistanceSquared(pathNodes[near].bounds, point))
                std::swap(near, far);
            self(self, near); self(self, far);
            return;
        }
        for (auto i = std::max(first, node.first); i <= node.last && distances[i - 1] <= to; ++i)
        {
            const double lo = std::max(from, distances[i - 1]), hi = std::min(to, distances[i]);
            const double span = distances[i] - distances[i - 1];
            if (hi < lo || span < 1e-8) continue;
            const Vector3 delta = points[i] - points[i - 1];
            const double t = std::clamp(Math::Dot(point - points[i - 1], delta) / (span * span),
                                        (lo - distances[i - 1]) / span, (hi - distances[i - 1]) / span);
            const Vector3 p = points[i - 1] + delta * t;
            const double error = Math::Length(point - p);
            // Match the original forward scan when two segments are equally near.
            if (error < best.error || (error == best.error && std::isfinite(error) && i < bestSegment))
            { best = {distances[i - 1] + t * span, error, p}; bestSegment = i; }
        }
    };
    visit(visit, 0);
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
    network.BuildMovements(world);
    network.BuildHighways(world);
    network.Validate();
    return network;
}
bool TrafficNetwork::Validate()
{
    laneIndex.clear(); outgoing.clear(); connections.clear(); families.clear();
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
        laneIndex.emplace(lane.id, &lane - lanes.data());
        families[FamilyKey(lane)].push_back(&lane - lanes.data());
        lane.Measure();
        if (!std::isfinite(lane.Length()) || lane.Length() < 0.05) return SDL_SetError("Invalid traffic path length");
    }
    for (std::size_t i = 0; i < links.size(); ++i)
    {
        auto& link = links[i];
        const auto* from = FindLane(link.from); const auto* to = FindLane(link.to);
        if (!from || !to || link.from == link.to || static_cast<unsigned>(link.kind) > 2 ||
            !std::isfinite(link.fromDistance) || !std::isfinite(link.toDistance) ||
            link.fromDistance < 0 || link.fromDistance > from->Length() ||
            link.toDistance < 0 || link.toDistance > to->Length() || link.path.points.size() < 2 ||
            !std::isfinite(link.path.speedLimit) || link.path.speedLimit <= 0 || link.path.speedLimit > 100)
            return SDL_SetError("Invalid cooked highway connection");
        for (std::size_t j = 0; j < link.path.points.size(); ++j)
            if (!std::isfinite(Math::Length(link.path.points[j])) ||
                (j && Math::Length(link.path.points[j] - link.path.points[j-1]) < 1e-8))
                return SDL_SetError("Invalid highway connection samples");
        if (Math::Length(link.path.points.front()-from->At(link.fromDistance)) > 0.01 ||
            Math::Length(link.path.points.back()-to->At(link.toDistance)) > 0.01)
            return SDL_SetError("Disconnected highway connection");
        link.path.Measure();
        if (!std::isfinite(link.path.Length()) || link.path.Length() < 0.05 || link.path.Length() > 10000)
            return SDL_SetError("Invalid highway connection length");
        link.path.width = std::min(from->width, to->width);
        connections[link.from].push_back(i);
    }
    ids.clear();
    for (std::size_t i = 0; i < movements.size(); ++i)
    {
        auto& m = movements[i];
        const auto* from = FindLane(m.from); const auto* to = FindLane(m.to);
        // A road can loop back to this intersection at a different anchor. Its
        // incoming and outgoing lane IDs then match; this is not a U-turn.
        if (!m.id || !ids.insert(m.id).second || m.junction <= 0 || (m.id >> 32) != unsigned(m.junction) ||
            !from || !to || m.fromSide < 0 || m.fromSide >= 12 || m.toSide < 0 || m.toSide >= 12 ||
            m.fromSide == m.toSide || static_cast<unsigned>(m.control) > 2 || static_cast<unsigned>(m.turn) > 2 ||
            m.path.points.size() < 2 || !std::isfinite(m.path.speedLimit) || m.path.speedLimit <= 0 || m.path.speedLimit > 100)
            return SDL_SetError("Invalid cooked intersection movement (intersection %d, anchors %d -> %d)",
                m.junction, m.fromSide, m.toSide);
        for (std::size_t j = 0; j < m.path.points.size(); ++j)
            if (!std::isfinite(Math::Length(m.path.points[j])) || (j && Math::Length(m.path.points[j]-m.path.points[j-1]) < 1e-8))
                return SDL_SetError("Invalid intersection path samples");
        if (Math::Length(m.path.points.front()-from->points.back()) > 0.01 || Math::Length(m.path.points.back()-to->points.front()) > 0.01)
            return SDL_SetError("Disconnected intersection movement");
        m.path.Measure();
        if (!std::isfinite(m.path.Length()) || m.path.Length() < 0.05 || m.path.Length() > 10000)
            return SDL_SetError("Invalid intersection path length");
        m.path.width = std::min(from->width,to->width);
        m.conflicts.clear(); outgoing[m.from].push_back(i);
    }
    // Precompute swept-path conflicts once. A 2.8 m corridor includes body width
    // and turning overhang; shared destinations always conflict (merging traffic).
    std::map<int, std::vector<std::size_t>> junctions;
    for (std::size_t i = 0; i < movements.size(); ++i) junctions[movements[i].junction].push_back(i);
    for (const auto& [id, group] : junctions) for (auto i : group) for (auto j : group)
    {
        if (j <= i) continue;
        auto& a = movements[i]; auto& b = movements[j];
        bool conflict = a.from == b.from || a.to == b.to;
        for (double d = 0; !conflict && d <= a.path.Length(); d += 1)
            conflict = b.path.Project(a.path.At(d)).error < 2.8;
        if (conflict) { a.conflicts.push_back(j); b.conflicts.push_back(i); }
    }
    for (auto& m : movements) std::sort(m.conflicts.begin(), m.conflicts.end());
    return true;
}
const TrafficLane* TrafficNetwork::FindLane(std::uint64_t id) const
{
    const auto it = laneIndex.find(id);
    return it == laneIndex.end() ? nullptr : &lanes[it->second];
}
const std::vector<std::size_t>& TrafficNetwork::Outgoing(std::uint64_t lane) const
{
    static const std::vector<std::size_t> empty;
    const auto it = outgoing.find(lane); return it == outgoing.end() ? empty : it->second;
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
