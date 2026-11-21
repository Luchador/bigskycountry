#include "road_io.hpp"

#include "material.hpp"
#include <iomanip>
#include <cmath>
#include <cstdint>
#include <istream>
#include <ostream>

namespace
{
    bool ValidDivision(const std::optional<RoadDivision>& division, int lanes, const RoadStyle& style)
    {
        if (division && (!division->Valid() || style.railroad)) return false;
        const bool divided = division ? division->enabled : style.IsDivided();
        return !divided || (!style.oneWay && !(lanes % 2));
    }

    void WriteDivision(std::ostream& out, const std::optional<RoadDivision>& division)
    {
        out << ' ' << division.has_value();
        if (division) out << ' ' << division->enabled << ' ' << division->separation << ' ' << static_cast<int>(division->median);
    }

    void ReadDivision(std::istream& in, std::optional<RoadDivision>& division)
    {
        bool present = false;
        if (!(in >> present) || !present) return;
        auto& value = division.emplace();
        int median = 0;
        in >> value.enabled >> value.separation >> median;
        value.median = static_cast<RoadMedian>(median);
        if (!value.Valid()) in.setstate(std::ios::failbit);
    }

    bool ValidShoulders(const std::array<int, 2>& values, bool railroad)
    {
        for (int value : values) if (value < 0 || value > 2 || (railroad && value)) return false;
        return true;
    }

    bool ValidStyle(int lanes, int layout, double dividedGap, int edge, int median)
    {
        return lanes >= 1 && lanes <= RoadStyle::maxLanes &&
               layout >= static_cast<int>(ThreeLaneLayout::TwoNorthbound) &&
               layout <= static_cast<int>(ThreeLaneLayout::CenterTurn) &&
               std::isfinite(dividedGap) && dividedGap >= 0.0 &&
               edge >= static_cast<int>(RoadEdge::None) && edge <= static_cast<int>(RoadEdge::Curb) &&
               median >= static_cast<int>(RoadMedian::Concrete) && median <= static_cast<int>(RoadMedian::Grass);
    }

    bool ValidMaterials(const RoadStyle& style)
    {
        const bool base = (style.railroad && style.material.empty()) || Material::ValidAssetPath(style.material);
        return base && (style.transitionMaterial.empty() || Material::ValidAssetPath(style.transitionMaterial));
    }

    bool ValidPoint(Vector3 position, double heading, double pitch)
    {
        return std::isfinite(position.x) && std::isfinite(position.y) &&
               std::isfinite(position.z) && std::isfinite(heading) && std::isfinite(pitch);
    }

    bool ValidSurfaceUp(Vector3 up)
    {
        return std::abs(Math::Length(up) - 1.0) < 1e-6;
    }

    bool ValidHandles(const RoadPoint& point)
    {
        return std::isfinite(point.incomingHandle) && point.incomingHandle >= 0 &&
               std::isfinite(point.outgoingHandle) && point.outgoingHandle >= 0;
    }
}

namespace RoadIO
{
    void WriteStyle(std::ostream& out, const RoadStyle& style)
    {
        const int edge = static_cast<int>(style.edge);
        const int median = static_cast<int>(style.median);
        if (!ValidMaterials(style) ||
            (style.railroad && (!style.railroad->Valid() || style.edge == RoadEdge::Curb)) ||
            !ValidStyle(style.GetLaneCount(), static_cast<int>(style.threeLaneLayout), style.DividedGap(), edge, median))
        {
            out.setstate(std::ios::failbit);
            return;
        }
        out << style.GetLaneCount() << ' ' << style.curved << ' ' << style.leftShoulder << ' '
            << style.rightShoulder << ' ' << static_cast<int>(style.threeLaneLayout) << ' '
            << style.IsDivided() << ' ' << style.DividedGap() << ' ' << edge << ' ' << median
            << ' ' << std::quoted(style.material) << ' ' << style.SingleLaneExtraWidth()
            << ' ' << std::quoted(style.transitionMaterial) << ' ' << style.oneWay << ' ' << style.railroad.has_value();
        if (style.railroad) out << ' ' << style.railroad->width << ' ' << style.railroad->height;
        out << '\n';
    }

    void ReadStyle(std::istream& in, RoadStyle& style, std::uint32_t version)
    {
        int lanes, layout;
        bool divided;
        double gap;
        int median = static_cast<int>(RoadMedian::Concrete);
        int edge = static_cast<int>(RoadEdge::None);
        in >> lanes >> style.curved >> style.leftShoulder >> style.rightShoulder >> layout
           >> divided >> gap;
        if (version >= 2)
        {
            in >> edge;
        }
        if (version >= 5) in >> median;
        style.material = RoadStyle::defaultMaterial;
        if (version >= 6) in >> std::quoted(style.material);
        double extraWidth = 0.0;
        if (version >= 11) in >> extraWidth;
        style.transitionMaterial.clear();
        if (version >= 12) in >> std::quoted(style.transitionMaterial);
        style.oneWay = false;
        if (version >= 13) in >> style.oneWay;
        style.railroad.reset();
        if (version >= 16)
        {
            bool rail = false;
            if (in >> rail && rail)
            {
                auto& track = style.railroad.emplace();
                if (!(in >> track.width >> track.height) || !track.Valid() ||
                    (edge != static_cast<int>(RoadEdge::None) && edge != static_cast<int>(RoadEdge::Transition)))
                    in.setstate(std::ios::failbit);
            }
        }
        if (in && divided && style.oneWay) in.setstate(std::ios::failbit);
        if (!in || !ValidMaterials(style) || !style.SetSingleLaneExtraWidth(extraWidth) ||
            !ValidStyle(lanes, layout, gap, edge, median) ||
            (version >= 5 && (gap < RoadStyle::minSeparation || (divided && lanes % 2 != 0))))
        {
            in.setstate(std::ios::failbit);
            return;
        }
        if (style.railroad && version >= 29 && (lanes < 1 || lanes > RailroadSettings::maxTracks))
            in.setstate(std::ios::failbit);
        style.SetLaneCount(style.railroad && version < 29 ? 1 : lanes);
        // Older files stored unused division fields and allowed odd lanes or sub-minimum gaps.
        style.SetDivided(divided);
        style.SetDividedGap(gap);
        style.median = static_cast<RoadMedian>(median);
        style.threeLaneLayout = static_cast<ThreeLaneLayout>(layout);
        style.edge = static_cast<RoadEdge>(edge);
    }

    void WriteRoad(std::ostream& out, const Road& road)
    {
        WriteStyle(out, road.style);
        if (!std::isfinite(Math::Length(road.rotation)))
        {
            out.setstate(std::ios::failbit);
            return;
        }
        for (int count : road.laneNeighbors)
            if (count < 0 || count > RoadStyle::maxLanes || (road.style.railroad && count > RailroadSettings::maxTracks))
            { out.setstate(std::ios::failbit); return; }
        for (int i = 0; i < 2; ++i)
            if (!ValidDivision(road.divisionNeighbors[i], road.laneNeighbors[i] ? road.laneNeighbors[i] : road.style.GetLaneCount(), road.style))
            { out.setstate(std::ios::failbit); return; }
        for (const auto& end : road.shoulderNeighbors)
            if (!ValidShoulders(end, road.style.railroad.has_value())) { out.setstate(std::ios::failbit); return; }
        for (double length : road.profileNeighborLengths)
            if (!std::isfinite(length) || length < 0) { out.setstate(std::ios::failbit); return; }
        out << road.rotation.x << ' ' << road.rotation.y << ' ' << road.rotation.z
            << ' ' << road.laneNeighbors[0] << ' ' << road.laneNeighbors[1];
        for (const auto& end : road.shoulderNeighbors) out << ' ' << end[0] << ' ' << end[1];
        for (const auto& end : road.divisionNeighbors) WriteDivision(out, end);
        for (double length : road.profileNeighborLengths) out << ' ' << length;
        out << '\n';
        out << road.id << ' ' << road.paintMarkings << ' ' << std::quoted(road.NameList()) << '\n';
        out << road.crossings.size() << '\n';
        for (const auto& crossing : road.crossings)
        {
            if (!crossing.Valid()) { out.setstate(std::ios::failbit); return; }
            out << crossing.railroadId << ' ' << crossing.roadFraction << ' ' << crossing.railFraction << '\n';
        }
        out << road.points.size() << '\n';
        for (const RoadPoint& p : road.points)
        {
            if (!ValidPoint(p.position, p.heading.value_or(0.0), p.pitch) || !std::isfinite(p.roll) || !ValidHandles(p) || !ValidSurfaceUp(p.surfaceUp) ||
                !ValidShoulders(p.segmentShoulders, road.style.railroad.has_value()) ||
                (p.bridge && (!p.bridge->Valid() || p.bridge->lowerRoads.contains(road.id))) ||
                p.segmentLanes < 0 || p.segmentLanes > RoadStyle::maxLanes ||
                (road.style.railroad && p.segmentLanes > RailroadSettings::maxTracks) ||
                !ValidDivision(p.segmentDivision, p.segmentLanes ? p.segmentLanes : road.style.GetLaneCount(), road.style) ||
                !std::isfinite(p.deadEndLength) || p.deadEndLength < 0 ||
                (p.deadEndLength > 0 && !road.CanDeadEnd(static_cast<int>(&p - road.points.data()))))
            {
                out.setstate(std::ios::failbit);
                return;
            }
            out << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' '
                << p.heading.has_value() << ' ' << p.heading.value_or(0.0) << ' ' << p.pitch << ' ' << p.roll;
            if (p.anchor)
            {
                if (!p.anchor->Valid() || (&p != &road.points.front() && &p != &road.points.back()))
                {
                    out.setstate(std::ios::failbit);
                    return;
                }
                out << ' ' << p.anchor->intersectionId << ' ' << p.anchor->side;
            }
            else out << " 0";
            out << ' ' << p.incomingHandle << ' ' << p.outgoingHandle << ' ' << p.rampStation << ' ' << (p.ramp ? p.ramp->station : 0);
            if (p.ramp) out << ' ' << p.ramp->side << ' ' << p.ramp->length << ' ' << p.ramp->offset;
            out << ' ' << p.connectorSocket << ' ' << p.connectorTarget
                << ' ' << p.surfaceUp.x << ' ' << p.surfaceUp.y << ' ' << p.surfaceUp.z << ' ' << p.deadEndLength << ' ' << p.segmentLanes
                << ' ' << p.segmentShoulders[0] << ' ' << p.segmentShoulders[1] << ' ' << p.bridge.has_value();
            if (p.bridge)
            {
                out << ' ' << p.bridge->depth << ' ' << p.bridge->groundHeight << ' ' << p.bridge->lowerRoads.size();
                for (int id : p.bridge->lowerRoads) out << ' ' << id;
                out << ' ' << p.bridge->lowerWaterBodies.size();
                for (auto id : p.bridge->lowerWaterBodies) out << ' ' << id;
            }
            if (p.railSwitch && (!p.railSwitch->Valid() || !road.style.railroad || p.anchor || p.ramp ||
                p.connectorSocket || p.connectorTarget || (&p != &road.points.front() && &p != &road.points.back())))
            { out.setstate(std::ios::failbit); return; }
            WriteDivision(out, p.segmentDivision);
            out << ' ' << p.railSwitch.has_value();
            if (p.railSwitch) out << ' ' << p.railSwitch->station << ' ' << p.railSwitch->direction << ' ' << p.railSwitch->trackSide;
            if (p.curveJoin && (!p.curveJoin->Valid() || road.style.railroad || p.anchor || p.ramp || p.railSwitch ||
                p.connectorSocket || p.connectorTarget || (&p != &road.points.front() && &p != &road.points.back())))
            { out.setstate(std::ios::failbit); return; }
            out << ' ' << p.curveJoin.has_value();
            if (p.curveJoin) out << ' ' << p.curveJoin->station << ' ' << p.curveJoin->side << ' ' << p.curveJoin->direction << ' ' << p.curveJoin->offset;
            out << '\n';
        }
    }

    Road ReadRoad(std::istream& in, std::uint32_t version)
    {
        Road road;
        ReadStyle(in, road.style, version);
        if (version >= 4 && (!(in >> road.rotation.x >> road.rotation.y >> road.rotation.z) ||
                             !std::isfinite(Math::Length(road.rotation))))
        {
            in.setstate(std::ios::failbit);
            return road;
        }
        if (version >= 20)
        {
            in >> road.laneNeighbors[0] >> road.laneNeighbors[1];
            for (int count : road.laneNeighbors)
                if (count < 0 || count > RoadStyle::maxLanes || (count && ((road.style.railroad && (version < 29 || count > RailroadSettings::maxTracks)) || (version < 24 && road.style.IsDivided() && count % 2))))
                    in.setstate(std::ios::failbit);
        }
        if (version >= 21)
        {
            for (auto& end : road.shoulderNeighbors)
            {
                in >> end[0] >> end[1];
                if (!ValidShoulders(end, road.style.railroad.has_value())) in.setstate(std::ios::failbit);
            }
        }
        if (version >= 24)
        {
            for (int i = 0; i < 2; ++i)
            {
                ReadDivision(in, road.divisionNeighbors[i]);
                if (!ValidDivision(road.divisionNeighbors[i], road.laneNeighbors[i] ? road.laneNeighbors[i] : road.style.GetLaneCount(), road.style))
                    in.setstate(std::ios::failbit);
            }
        }
        if (version >= 28)
        {
            for (double& length : road.profileNeighborLengths)
                if (!(in >> length) || !std::isfinite(length) || length < 0)
                { in.setstate(std::ios::failbit); return road; }
        }
        if (version >= 8 && (!(in >> road.id) || road.id < 0))
        {
            in.setstate(std::ios::failbit);
            return road;
        }
        if (version >= 10)
        {
            std::string names;
            if (!(in >> road.paintMarkings >> std::quoted(names)) || !road.SetNameList(names))
            {
                in.setstate(std::ios::failbit);
                return road;
            }
        }
        if (version >= 18)
        {
            std::int64_t count = 0;
            if (!(in >> count) || count < 0 || count > 100000 || (count && road.style.railroad && version < 26))
            { in.setstate(std::ios::failbit); return road; }
            for (std::int64_t i = 0; i < count; ++i)
            {
                RailroadCrossingLink crossing;
                if (!(in >> crossing.railroadId >> crossing.roadFraction >> crossing.railFraction) || !crossing.Valid())
                { in.setstate(std::ios::failbit); return road; }
                road.crossings.push_back(crossing);
            }
        }
        std::int64_t count;
        if (!(in >> count) || count < 0)
        {
            in.setstate(std::ios::failbit);
            return road;
        }
        for (std::int64_t i = 0; i < count; ++i)
        {
            RoadPoint p;
            bool hasHeading;
            double heading;
            in >> p.position.x >> p.position.y >> p.position.z >> hasHeading >> heading;
            if (version >= 3)
            {
                in >> p.pitch;
            }
            if (version >= 4) in >> p.roll;
            if (version >= 7)
            {
                int id;
                if (!(in >> id)) return road;
                if (id != 0)
                {
                    IntersectionAnchor anchor{id, -1};
                    if (!(in >> anchor.side) || !anchor.Valid() || (i != 0 && i != count - 1))
                    {
                        in.setstate(std::ios::failbit);
                        return road;
                    }
                    p.anchor = anchor;
                }
            }
            if (version >= 9) in >> p.incomingHandle >> p.outgoingHandle;
            if (version >= 13)
            {
                int target = 0;
                if (!(in >> p.rampStation >> target) || p.rampStation < 0) in.setstate(std::ios::failbit);
                if (target)
                {
                    RampLink link{target};
                    in >> link.side >> link.length;
                    if (version >= 14) in >> link.offset;
                    if (!in || !link.Valid() || p.anchor || !road.style.oneWay ||
                        (i != 0 && i != count - 1)) in.setstate(std::ios::failbit);
                    p.ramp = link;
                }
            }
            if (version >= 15)
            {
                in >> p.connectorSocket >> p.connectorTarget;
                if (p.connectorSocket < 0 || p.connectorTarget < 0 ||
                    ((p.connectorSocket || p.connectorTarget) && (p.anchor || p.ramp || (i != 0 && i + 1 != count))) ||
                    (p.connectorSocket && p.connectorTarget)) in.setstate(std::ios::failbit);
            }
            if (version >= 17) in >> p.surfaceUp.x >> p.surfaceUp.y >> p.surfaceUp.z;
            if (version >= 19) in >> p.deadEndLength;
            if (version >= 20) in >> p.segmentLanes;
            if (version >= 21) in >> p.segmentShoulders[0] >> p.segmentShoulders[1];
            if (version >= 22)
            {
                bool bridge = false;
                if (!(in >> bridge)) return road;
                if (bridge)
                {
                    auto& span = p.bridge.emplace();
                    std::int64_t sources = 0;
                    if (!(in >> span.depth >> span.groundHeight >> sources) || sources < 0 || sources > 10000 || (road.style.railroad && version < 23))
                    { in.setstate(std::ios::failbit); return road; }
                    for (std::int64_t j = 0; j < sources; ++j)
                    {
                        int id = 0;
                        if (!(in >> id) || id <= 0 || id == road.id || !span.lowerRoads.insert(id).second)
                        { in.setstate(std::ios::failbit); return road; }
                    }
                    if (version >= 25)
                    {
                        if (!(in >> sources) || sources < 0 || sources > 10000)
                        { in.setstate(std::ios::failbit); return road; }
                        for (std::int64_t j = 0; j < sources; ++j)
                        {
                            std::uint64_t id = 0;
                            if (!(in >> id) || id == 0 || !span.lowerWaterBodies.insert(id).second)
                            { in.setstate(std::ios::failbit); return road; }
                        }
                    }
                    if (!span.Valid()) { in.setstate(std::ios::failbit); return road; }
                }
            }
            if (version >= 24) ReadDivision(in, p.segmentDivision);
            if (version >= 27)
            {
                bool linked = false;
                if (!(in >> linked)) return road;
                if (linked)
                {
                    auto& link = p.railSwitch.emplace();
                    in >> link.station >> link.direction;
                    if (version >= 29) in >> link.trackSide;
                    if (!link.Valid() || !road.style.railroad || p.anchor || p.ramp || p.connectorSocket ||
                        p.connectorTarget || (i != 0 && i + 1 != count)) in.setstate(std::ios::failbit);
                }
            }
            if (version >= 30)
            {
                bool linked = false;
                if (!(in >> linked)) return road;
                if (linked)
                {
                    auto& link = p.curveJoin.emplace();
                    in >> link.station >> link.side >> link.direction >> link.offset;
                    if (!link.Valid() || road.style.railroad || p.anchor || p.ramp || p.railSwitch || p.connectorSocket ||
                        p.connectorTarget || (i != 0 && i + 1 != count)) in.setstate(std::ios::failbit);
                }
            }
            if (!ValidDivision(p.segmentDivision, p.segmentLanes ? p.segmentLanes : road.style.GetLaneCount(), road.style)) in.setstate(std::ios::failbit);
            if (!ValidShoulders(p.segmentShoulders, road.style.railroad.has_value())) in.setstate(std::ios::failbit);
            if (p.segmentLanes < 0 || p.segmentLanes > RoadStyle::maxLanes ||
                (road.style.railroad && (p.segmentLanes > RailroadSettings::maxTracks || (version < 29 && p.segmentLanes)))) in.setstate(std::ios::failbit);
            if (!std::isfinite(p.deadEndLength) || p.deadEndLength < 0 ||
                (p.deadEndLength > 0 && (count < 2 || (i != 0 && i + 1 != count) || road.style.railroad || p.Joined() || p.rampStation)))
                in.setstate(std::ios::failbit);
            if (!in || !ValidPoint(p.position, heading, p.pitch) || !std::isfinite(p.roll) || !ValidHandles(p) || !ValidSurfaceUp(p.surfaceUp))
            {
                in.setstate(std::ios::failbit);
                break;
            }
            if (hasHeading)
            {
                p.heading = heading;
            }
            road.points.push_back(p);
        }
        return road;
    }
}
