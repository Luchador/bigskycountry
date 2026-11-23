#include "traffic_network.hpp"
#include "world_document.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace
{
    double Cross2(Vector3 a, Vector3 b) { return a.x * b.z - a.z * b.x; }
    bool OnPavement(const RoadGeometry& surface, Vector3 p)
    {
        for (const auto& t : surface.pavementTriangles)
        {
            const auto a = t[0].position, b = t[1].position, c = t[2].position;
            if (std::abs(Cross2(b-a,c-a)) < 1e-10) continue;
            const double x = Cross2(b-a, p-a), y = Cross2(c-b, p-b), z = Cross2(a-c, p-c);
            if ((x >= -1e-6 && y >= -1e-6 && z >= -1e-6) || (x <= 1e-6 && y <= 1e-6 && z <= 1e-6)) return true;
        }
        return false;
    }
    bool MakeTurn(TrafficLane& path, const TrafficLane& from, const TrafficLane& to, const RoadGeometry& surface)
    {
        const auto a = from.points.back(), d = to.points.front();
        const auto incoming = from.Tangent(from.Length()), outgoing = to.Tangent(0);
        const double distance = Math::Length(d-a);
        if (distance < 1) return false;
        for (double factor : {0.4, 0.3, 0.5, 0.55, 0.6, 0.65})
        {
            const auto b = a + incoming * (distance * factor), c = d - outgoing * (distance * factor);
            path.points.clear();
            const int count = std::clamp(static_cast<int>(std::ceil(distance * 2)), 8, 8192);
            for (int i = 0; i <= count; ++i)
            {
                const double t = double(i) / count, u = 1-t;
                path.points.push_back(a*(u*u*u) + b*(3*u*u*t) + c*(3*u*t*t) + d*(t*t*t));
            }
            path.Measure();
            bool valid = true;
            for (double s = 0.5; s < path.Length()-0.5; s += 0.5)
            {
                const auto p = path.At(s), tangent = path.Tangent(s);
                // Shared one-lane approaches have less spare pavement, but
                // must still fit the 2 m car body with a small edge clearance.
                const double clearance = std::clamp(path.width * .5 - .15, 1.05, 1.25);
                const auto side = Math::Normalize(Math::Cross({0,1,0}, tangent)) * clearance;
                if (!OnPavement(surface, p) || !OnPavement(surface, p-side) || !OnPavement(surface, p+side))
                { valid = false; break; }
                // Reject a hairpin that the test car cannot follow at low speed.
                if (Math::Length(path.Tangent(s+0.5)-path.Tangent(s-0.5)) > 0.23) { valid = false; break; }
            }
            if (valid) return true;
        }
        return false; // Never route across an island or outside the paved junction.
    }
}

void TrafficNetwork::BuildMovements(const WorldDocument& world)
{
    movements.clear();
    std::map<int, std::vector<const TrafficLane*>> roadLanes;
    for (const auto& lane : lanes) roadLanes[lane.roadId].push_back(&lane);
    std::map<int, std::vector<WorldDocument::ConnectedEnd>> connected;
    for (std::size_t i = 0; i < world.roads.size(); ++i)
    {
        const auto& points = world.roads[i].points;
        if (points.size() < 2) continue;
        for (bool first : {true,false})
            if (const auto& anchor = (first ? points.front() : points.back()).anchor; anchor && anchor->Valid())
                connected[anchor->intersectionId].push_back({i,first,*anchor});
    }
    for (const auto& [id, junction] : world.intersections)
    {
        const auto surface = world.IntersectionSurfaces().find(id);
        if (surface == world.IntersectionSurfaces().end()) continue;
        std::array<std::vector<const TrafficLane*>, IntersectionAnchor::sideCount> incoming, outgoing;
        for (const auto& end : connected[id])
        {
            const auto& road = world.roads[end.road];
            if (road.style.railroad || road.points.size() < 2) continue;
            const auto& mouth = road.EndpointMouth(end.first);
            if (!mouth) continue;
            for (const auto* lane : roadLanes[road.id])
            {
                const bool enters = (lane->direction > 0) != end.first;
                const auto p = enters ? lane->points.back() : lane->points.front();
                if (std::abs(Math::Dot(p-mouth->center, mouth->outward)) > 0.25 || std::abs(p.y-mouth->center.y) > 0.5) continue;
                (enters ? incoming : outgoing)[end.anchor.side].push_back(lane);
            }
        }
        for (auto* arms : {&incoming, &outgoing}) for (auto& arm : *arms)
            std::sort(arm.begin(), arm.end(), [](auto a, auto b) { return a->ordinal < b->ordinal; });
        std::uint32_t local = 0;
        for (int a = 0; a < IntersectionAnchor::sideCount; ++a)
            for (int b = 0; b < IntersectionAnchor::sideCount; ++b)
            {
                if (a == b || incoming[a].empty() || outgoing[b].empty()) continue; // No U-turns.
                const auto f = incoming[a].front()->Tangent(incoming[a].front()->Length());
                const auto t = outgoing[b].front()->Tangent(0);
                if (Math::Dot(f,t) < -0.8) continue;
                const auto turn = Math::Dot(f,t) > 0.75 ? TrafficTurn::Straight :
                    Math::Cross(f,t).y > 0 ? TrafficTurn::Right : TrafficTurn::Left;
                for (std::size_t i = 0; i < incoming[a].size(); ++i)
                {
                    if (turn == TrafficTurn::Left && i != 0) continue;
                    if (turn == TrafficTurn::Right && i+1 != incoming[a].size()) continue;
                    if (turn == TrafficTurn::Straight && i >= outgoing[b].size()) continue;
                    const auto& from = *incoming[a][i];
                    const auto& to = *outgoing[b][turn == TrafficTurn::Right ? outgoing[b].size()-1 : turn == TrafficTurn::Left ? 0 : i];
                    if (to.Length() < 10) continue; // Must hold an entire car beyond the junction.
                    TrafficMovement m;
                    m.id = (std::uint64_t(id) << 32) | ++local; m.junction = id;
                    m.from = from.id; m.to = to.id; m.fromSide = a; m.toSide = b;
                    m.control = junction.trafficControls[a]; m.turn = turn;
                    m.path.width = std::min(from.WidthAt(from.Length()), to.WidthAt(0));
                    m.path.speedLimit = std::min({from.speedLimit, to.speedLimit, turn == TrafficTurn::Straight ? 13.4112 : 7.0});
                    if (turn != TrafficTurn::Straight && m.path.width < 3.5)
                        m.path.speedLimit = std::min(m.path.speedLimit, 3.0);
                    if (MakeTurn(m.path, from, to, surface->second.geometry)) movements.push_back(std::move(m));
                }
            }
    }
}
