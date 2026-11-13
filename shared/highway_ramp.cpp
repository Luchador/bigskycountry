#include "highway_ramp.hpp"
#include "world_document.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

RampLane HighwayRampLane(const RoadStyle& style, const RampLink& link, bool exit)
{
    return {link.station, link.side, exit, link.length, style.GetLaneCount() * style.TravelLaneWidth() * 0.5,
            style.LeftWidth(), style.RightWidth(), style.EdgeWidth()};
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
        }
    return last < std::numeric_limits<int>::max() ? last + 1 : 0;
}

void WorldDocument::RefreshRampConnections(bool force)
{
    if (!force && rampRevision == RoadGeometry::latestRevision) return;
    std::map<int, size_t> stations;
    for (size_t i = 0; i < roads.size(); ++i)
    {
        const Road& host = roads[i];
        if (!host.style.IsDivided()) continue;
        for (const auto& point : host.points)
            if (point.rampStation) stations.emplace(point.rampStation, i);
    }
    std::map<size_t, std::vector<RampLane>> lanes;
    for (const Road& ramp : roads)
    {
        if (!ramp.style.oneWay || ramp.points.size() < 2) continue;
        for (bool exit : {true, false})
        {
            const auto& link = (exit ? ramp.points.front() : ramp.points.back()).ramp;
            if (!link) continue;
            const auto it = stations.find(link->station);
            if (it != stations.end())
                lanes[it->second].push_back(HighwayRampLane(ramp.style, *link, exit));
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
    rampRevision = RoadGeometry::latestRevision;
}
