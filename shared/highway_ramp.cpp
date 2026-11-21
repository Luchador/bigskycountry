#include "highway_ramp.hpp"
#include "railroad_switch.hpp"
#include "world_document.hpp"
#include "mesh.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

RampLane HighwayRampLane(const RoadStyle& style, const RampLink& link, bool exit, bool branchesLeft)
{
    return {link.station, link.side, exit, link.length, style.GetLaneCount() * style.TravelLaneWidth() * 0.5,
            branchesLeft ? style.RightWidth() : style.LeftWidth(), branchesLeft ? style.LeftWidth() : style.RightWidth(),
            style.EdgeWidth(), style.GetLaneCount(), link.offset};
}

void AlignRampEnd(RoadPoint& end, const RampSeam& seam)
{
    end.position = seam.center;
    end.heading = std::atan2(seam.forward.z, seam.forward.x);
    end.pitch = std::atan2(seam.forward.y, std::hypot(seam.forward.x, seam.forward.z));
}

int WorldDocument::NextRampStationId() const
{
    int last = 0;
    for (const Road& road : roads)
        for (const auto& p : road.points)
        {
            last = std::max(last, p.rampStation);
            if (p.ramp) last = std::max(last, p.ramp->station);
            if (p.railSwitch) last = std::max(last, p.railSwitch->station);
            if (p.curveJoin) last = std::max(last, p.curveJoin->station);
        }
    return last < std::numeric_limits<int>::max() ? last + 1 : 0;
}

void WorldDocument::RefreshRampConnections(bool force)
{
    if (!force && rampRevision == RoadGeometry::latestRevision) return;
    AlignRailroadSwitches();
    struct Socket { const Road* road; std::size_t point; };
    std::map<int, Socket> sockets;
    std::map<int, size_t> stations;
    for (size_t i = 0; i < roads.size(); ++i)
    {
        const Road& host = roads[i];
        if (host.points.size() < 2) continue;
        for (std::size_t j : {std::size_t{0}, host.points.size() - 1})
            if (int id = host.points[j].connectorSocket) sockets.emplace(id, Socket{&host, j});
        if (host.MaximumLaneCount() < 2) continue;
        for (const auto& point : host.points)
            if (point.rampStation) stations.emplace(point.rampStation, i);
    }
    std::map<size_t, std::vector<RampLane>> lanes;
    for (int index = 0; index < static_cast<int>(roads.size()); ++index)
    {
        const Road& ramp = roads[index];
        if (!ramp.style.oneWay || ramp.points.size() < 2) continue;
        for (bool exit : {true, false})
        {
            const auto& link = (exit ? ramp.points.front() : ramp.points.back()).ramp;
            if (!link) continue;
            const auto it = stations.find(link->station);
            if (it != stations.end())
            {
                const bool branchesLeft = roads[it->second].style.oneWay && link->side > 0;
                auto lane = HighwayRampLane(ramp.EndpointStyle(exit), *link, exit, branchesLeft);
                lane.rampIndex = index;
                lanes[it->second].push_back(lane);
            }
        }
    }
    for (size_t i = 0; i < roads.size(); ++i)
    {
        Road& host = roads[i];
        auto it = lanes.find(i);
        if (it == lanes.end() && host.rampLanes.empty()) continue;
        std::vector<RampLane> current;
        if (it != lanes.end()) current = std::move(it->second);
        if (current == host.rampLanes) continue;
        host.rampLanes = std::move(current);
        host.Rebuild();
    }
    for (Road& ramp : roads)
    {
        if (!ramp.style.oneWay || ramp.points.size() < 2) continue;
        std::array<std::optional<RoadMouthFrame>, 2> caps;
        bool changed = false;
        for (int e = 0; e < 2; ++e)
        {
            auto& point = e == 0 ? ramp.points.front() : ramp.points.back();
            if (point.connectorTarget || point.curveJoin)
            {
                caps[e] = ramp.rampCaps[e]; // Updated after both source ramps have been resolved.
                continue;
            }
            if (!point.ramp) continue;
            const auto& link = *point.ramp;
            const auto host = stations.find(link.station);
            if (host == stations.end()) continue;
            const auto& seams = roads[host->second].rampSeams;
            const auto seam = seams.find({link.station, link.side});
            if (seam == seams.end()) continue;
            const auto before = point;
            AlignRampEnd(point, seam->second);
            changed |= point != before;
            caps[e] = seam->second.frame;
        }
        if (changed || caps != ramp.rampCaps)
        {
            ramp.rampCaps = caps;
            ramp.Rebuild();
        }
    }
    // Resolve every target position before deriving tangents. A source road can
    // itself have an attachment at its other end, in any world-file order.
    std::set<int> moved;
    for (Road& road : roads)
    {
        if (road.points.size() < 2) continue;
        for (RoadPoint* end : {&road.points.front(), &road.points.back()})
        {
            const auto it = sockets.find(end->connectorTarget);
            if (!end->connectorTarget || it == sockets.end()) continue;
            const auto [source, i] = it->second;
            auto position = source->points[i].position;
            if (road.style.railroad && source->style.railroad)
            {
                const auto frame = RailroadSwitchFrame(*source, i, 1);
                position = frame.origin-frame.y*(road.style.railroad->height+RailroadSettings::tieHeight+RailroadSettings::railHeight);
            }
            if (end->position == position) continue;
            end->position = position;
            moved.insert(road.id);
        }
    }
    // Pair IDs survive reordering and splitting. A direct road join can use
    // either end of each road; equal endpoint roles require opposite tangents.
    for (Road& road : roads)
    {
        if (road.points.size() < 2) continue;
        bool changed = moved.contains(road.id);
        for (int e = 0; e < 2; ++e)
        {
            RoadPoint& end = e == 0 ? road.points.front() : road.points.back();
            if (!end.connectorTarget)
            {
                if (!end.ramp && !end.curveJoin && road.rampCaps[e])
                {
                    road.rampCaps[e].reset();
                    changed = true;
                }
                continue;
            }
            const auto it = sockets.find(end.connectorTarget);
            if (it == sockets.end()) continue;
            const auto [source, i] = it->second;
            const double direction = (e == 0) == (i == 0) ? -1.0 : 1.0;
            const Vector3 tangent = source->Tangent(i) * direction;
            auto position = source->points[i].position;
            if (road.style.railroad && source->style.railroad)
            {
                const auto track = RailroadSwitchFrame(*source, i, 1);
                position = track.origin-track.y*(road.style.railroad->height+RailroadSettings::tieHeight+RailroadSettings::railHeight);
                if (road.rotation != source->rotation) { road.rotation = source->rotation; changed = true; }
            }
            const double heading = std::atan2(tangent.z, tangent.x);
            const double pitch = std::atan2(tangent.y, std::hypot(tangent.x, tangent.z));
            // Match the actual cross-section of the source, including whole-road bank.
            const auto frame = ObjectTransform{{}, source->rotation}.ToMeshTransform();
            const Vector3 local = frame.Unrotate(tangent);
            const Vector3 up = source->points[i].surfaceUp;
            const Vector3 across = Math::Cross(local, up);
            const Vector3 left = Math::Length(across) > 1e-8 ? Math::Normalize(across) : Vector3{0, 0, 1};
            const RoadMouthFrame cap{frame.Direction(left), frame.Direction(up)};
            if (road.rampCaps[e] != cap)
            {
                road.rampCaps[e] = cap;
                changed = true;
            }
            if (end.position == position && end.heading == heading && end.pitch == pitch) continue;
            end.position = position;
            end.heading = heading;
            end.pitch = pitch;
            changed = true;
        }
        if (changed) road.Rebuild();
    }
    // Resolve the branch first: its spline is independent of the highway shoulder.
    // Then trim only the mainline shoulder around the actual branch footprint.
    // Cache authored-derived contours, never repeatedly clip an already cut mesh.
    std::map<size_t, std::vector<RampShoulderCut>> shoulderCuts;
    for (const auto& ramp : roads)
    {
        if (!ramp.style.oneWay || ramp.points.size() < 2) continue;
        for (const auto* end : {&ramp.points.front(), &ramp.points.back()})
        {
            if (!end->ramp) continue;
            const auto& link = *end->ramp;
            const auto host = stations.find(link.station);
            if (host == stations.end() || roads[host->second].MaximumShoulderWidth(link.side > 0) == 0) continue;
            shoulderCuts[host->second].push_back({link.station, link.side, ramp.outline});
        }
    }
    for (size_t i = 0; i < roads.size(); ++i)
    {
        auto& host = roads[i];
        const auto found = shoulderCuts.find(i);
        if (found == shoulderCuts.end() && host.rampShoulderCuts.empty()) continue;
        auto current = found == shoulderCuts.end() ? std::vector<RampShoulderCut>{} : std::move(found->second);
        if (current == host.rampShoulderCuts) continue;
        host.rampShoulderCuts = std::move(current);
        host.Rebuild();
    }
    RefreshCurvedConnectors();
    RefreshRailroadSwitches();
    rampRevision = RoadGeometry::latestRevision;
}
