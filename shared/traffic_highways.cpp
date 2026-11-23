#include "traffic_network.hpp"
#include "world_document.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

namespace
{
    const std::vector<std::size_t> emptyIndices;
    double Ease(double t) { return t*t*t*(10+t*(-15+6*t)); }
    void Append(std::vector<Vector3>& points, Vector3 p)
    { if (points.empty() || Math::Length(points.back()-p) > 1e-8) points.push_back(p); }
}

const std::vector<std::size_t>& TrafficNetwork::Connections(std::uint64_t lane) const
{
    const auto i = connections.find(lane);
    return i == connections.end() ? emptyIndices : i->second;
}
const std::vector<std::size_t>& TrafficNetwork::Family(const TrafficLane& lane) const
{
    const auto i = families.find(FamilyKey(lane));
    return i == families.end() ? emptyIndices : i->second;
}
bool TrafficNetwork::LaneChange(const TrafficLane& from, double start, const TrafficLane& to,
                               double length, TrafficLane& path, double& destination) const
{
    if (from.roadId != to.roadId || from.direction != to.direction ||
        std::abs(from.ordinal-to.ordinal) != 1 || start < 0 || length < 16 ||
        start+length+3 > from.Length()) return false;
    const auto begin = to.Project(from.At(start));
    if (begin.distance < 1 || begin.distance+length+8 > to.Length()) return false;
    path = {}; path.width = std::min(from.width,to.width);
    path.speedLimit = std::min(from.speedLimit,to.speedLimit);
    const int count = std::clamp(static_cast<int>(std::ceil(length/2)), 8, 256);
    double last = begin.distance;
    for (int i = 0; i <= count; ++i)
    {
        const double t = double(i)/count, d = start+length*t;
        const auto a = from.At(d);
        const auto b = to.Project(a, std::max(0.0,last-0.1), std::min(to.Length(),last+length/count*2+5));
        const auto tangent = from.Tangent(d), right = Math::Normalize(Math::Cross({0,1,0},tangent));
        const auto delta = b.position-a;
        const double lateral = std::abs(Math::Dot(delta,right));
        // A median, shoulder, nearby hairpin, opposite carriageway or another deck
        // must never be mistaken for an adjacent travel lane.
        if (!std::isfinite(b.error) || lateral < 2.5 || lateral > 7.0 ||
            std::abs(delta.y) > 1.5 || std::abs(Math::Dot(delta,tangent)) > 0.5 ||
            Math::Dot(tangent,to.Tangent(b.distance)) < 0.94) return false;
        Append(path.points, a+delta*Ease(t)); last = b.distance;
    }
    destination = last;
    if (destination+8 > to.Length()) return false;
    path.Measure();
    return path.Length() >= 16;
}

void TrafficNetwork::BuildHighways(const WorldDocument& world)
{
    links.clear();
    std::map<int,std::vector<const TrafficLane*>> roadLanes;
    for (const auto& lane : lanes) roadLanes[lane.roadId].push_back(&lane);
    const auto connectEnds = [&](const TrafficLane& from, const TrafficLane& to) {
        if (from.id == to.id || Math::Length(from.points.back()-to.points.front()) > 0.08 ||
            Math::Dot(from.Tangent(from.Length()),to.Tangent(0)) < 0.94 ||
            from.Length() < 6 || to.Length() < 10) return;
        for (const auto& link : links) if (link.from == from.id && link.to == to.id) return;
        TrafficLink link;
        link.from = from.id; link.to = to.id;
        link.fromDistance = from.Length()-2; link.toDistance = 2;
        link.path.width = std::min(from.width,to.width);
        link.path.speedLimit = std::min(from.speedLimit,to.speedLimit);
        link.path.points = {from.At(link.fromDistance),from.points.back(),to.At(2)};
        link.path.Measure(); links.push_back(std::move(link));
    };
    struct End { const Road* road; bool first; };
    std::map<int,End> sockets;
    // Split roads retain coincident free mouths. Match only exact lane endpoints
    // with aligned travel directions, not arbitrary nearby/crossing road geometry.
    using Key = std::tuple<long long,long long,long long>;
    std::map<Key,std::vector<const TrafficLane*>> starts;
    const auto key = [](Vector3 p) { return Key{std::llround(p.x*10),std::llround(p.y*10),std::llround(p.z*10)}; };
    for (const auto& road : world.roads)
    {
        if (road.style.railroad || road.points.size() < 2) continue;
        for (bool first : {true,false})
        {
            const auto& point = first ? road.points.front() : road.points.back();
            if (point.connectorSocket) sockets.emplace(point.connectorSocket,End{&road,first});
        }
        for (auto* lane : roadLanes[road.id])
        {
            const auto& point = lane->direction > 0 ? road.points.front() : road.points.back();
            if (!point.anchor && !point.ramp && !point.curveJoin) starts[key(lane->points.front())].push_back(lane);
        }
    }
    for (const auto& road : world.roads)
    {
        if (road.style.railroad || road.points.size() < 2) continue;
        for (auto* lane : roadLanes[road.id])
        {
            const auto& point = lane->direction > 0 ? road.points.back() : road.points.front();
            if (point.anchor || point.ramp || point.curveJoin) continue;
            const auto [x,y,z] = key(lane->points.back());
            for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz)
                if (const auto i=starts.find({x+dx,y+dy,z+dz}); i!=starts.end())
                    for (auto* to : i->second) if (to->roadId != road.id) connectEnds(*lane,*to);
        }
        for (bool first : {true,false})
        {
            const auto& point = first ? road.points.front() : road.points.back();
            const auto socket = sockets.find(point.connectorTarget);
            if (!point.connectorTarget || socket == sockets.end()) continue;
            for (auto* a : roadLanes[road.id]) for (auto* b : roadLanes[socket->second.road->id])
            { connectEnds(*a,*b); connectEnds(*b,*a); }
        }
    }
    for (const auto& host : world.roads)
    {
        for (const auto& guide : host.rampTrafficPaths)
        {
            const auto seam = host.rampSeams.find({guide.station,guide.side});
            if (seam == host.rampSeams.end() || seam->second.rampIndex < 0 ||
                static_cast<std::size_t>(seam->second.rampIndex) >= world.roads.size()) continue;
            const auto& ramp = world.roads[seam->second.rampIndex];
            const auto hostPoint = guide.exit ? guide.points.front() : guide.points.back();
            const auto rampPoint = guide.exit ? guide.points.back() : guide.points.front();
            const TrafficLane* main = nullptr; const TrafficLane* branch = nullptr;
            double mainDistance = 0, error = 3.0;
            for (auto* lane : roadLanes[host.id])
            {
                if (lane->direction != host.style.TravelDirection(guide.side)) continue;
                const auto p = lane->Project(hostPoint);
                if (p.error < error) { error = p.error; main = lane; mainDistance = p.distance; }
            }
            error = 0.25;
            for (auto* lane : roadLanes[ramp.id])
            {
                if (lane->direction < 0) continue;
                const auto p = guide.exit ? lane->points.front() : lane->points.back();
                const double distance = Math::Length(p-rampPoint);
                if (distance < error) { error = distance; branch = lane; }
            }
            if (!main || !branch || guide.points.size() < 2) continue;
            TrafficLink link;
            link.from = guide.exit ? main->id : branch->id;
            link.to = guide.exit ? branch->id : main->id;
            link.fromDistance = guide.exit ? mainDistance : branch->Length();
            link.toDistance = guide.exit ? 0 : mainDistance;
            link.kind = guide.exit ? TrafficLinkKind::Exit : TrafficLinkKind::Merge;
            link.path.points = guide.points;
            link.path.width = std::min(main->width,branch->width);
            link.path.speedLimit = std::min(main->speedLimit,branch->speedLimit);
            // Multilane tapers initially share one host lane. Ease the small
            // center offset to that lane without a discontinuity at the tip.
            const auto delta = main->At(mainDistance)-hostPoint;
            TrafficLane sampled; sampled.points = guide.points; sampled.Measure();
            for (std::size_t i=0; i<link.path.points.size(); ++i)
            {
                const double t = sampled.distances[i]/sampled.Length();
                link.path.points[i] = link.path.points[i]+delta*(guide.exit ? 1-Ease(t) : Ease(t));
            }
            link.path.points.front() = guide.exit ? main->At(mainDistance) : branch->points.back();
            link.path.points.back() = guide.exit ? branch->points.front() : main->At(mainDistance);
            // Surface subdivision can emit the same seam twice (or round to it
            // at large world coordinates). Navigation requires nonzero segments.
            link.path.points.erase(std::unique(link.path.points.begin(), link.path.points.end(),
                [](Vector3 a, Vector3 b) { return Math::Length(a-b) < 1e-8; }), link.path.points.end());
            link.path.Measure();
            if (link.path.Length() >= 8) links.push_back(std::move(link));
        }
    }
}
