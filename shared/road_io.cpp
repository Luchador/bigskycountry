#include "road_io.hpp"

#include "material.hpp"
#include <iomanip>
#include <cmath>
#include <cstdint>
#include <istream>
#include <ostream>

namespace
{
    bool ValidStyle(int lanes, int layout, double dividedGap, int edge, int median)
    {
        return lanes >= 1 && lanes <= RoadStyle::maxLanes &&
               layout >= static_cast<int>(ThreeLaneLayout::TwoNorthbound) &&
               layout <= static_cast<int>(ThreeLaneLayout::CenterTurn) &&
               std::isfinite(dividedGap) && dividedGap >= 0.0 &&
               edge >= static_cast<int>(RoadEdge::None) && edge <= static_cast<int>(RoadEdge::Curb) &&
               median >= static_cast<int>(RoadMedian::Concrete) && median <= static_cast<int>(RoadMedian::Grass);
    }

    bool ValidPoint(Vector3 position, double heading, double pitch)
    {
        return std::isfinite(position.x) && std::isfinite(position.y) &&
               std::isfinite(position.z) && std::isfinite(heading) && std::isfinite(pitch);
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
        if ((!style.transitionMaterial.empty() && !Material::ValidAssetPath(style.transitionMaterial)) ||
            !Material::ValidAssetPath(style.material) ||
            !ValidStyle(style.GetLaneCount(), static_cast<int>(style.threeLaneLayout), style.DividedGap(), edge, median))
        {
            out.setstate(std::ios::failbit);
            return;
        }
        out << style.GetLaneCount() << ' ' << style.curved << ' ' << style.leftShoulder << ' '
            << style.rightShoulder << ' ' << static_cast<int>(style.threeLaneLayout) << ' '
            << style.IsDivided() << ' ' << style.DividedGap() << ' ' << edge << ' ' << median
            << ' ' << std::quoted(style.material) << ' ' << style.SingleLaneExtraWidth()
            << ' ' << std::quoted(style.transitionMaterial) << '\n';
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
        if ((!style.transitionMaterial.empty() && !Material::ValidAssetPath(style.transitionMaterial)) ||
            !in || !style.SetSingleLaneExtraWidth(extraWidth) || !Material::ValidAssetPath(style.material) ||
            !ValidStyle(lanes, layout, gap, edge, median) ||
            (version >= 5 && (gap < RoadStyle::minSeparation || (divided && lanes % 2 != 0))))
        {
            in.setstate(std::ios::failbit);
            return;
        }
        style.SetLaneCount(lanes);
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
        out << road.rotation.x << ' ' << road.rotation.y << ' ' << road.rotation.z << '\n';
        out << road.id << ' ' << road.paintMarkings << ' ' << std::quoted(road.NameList()) << '\n';
        out << road.points.size() << '\n';
        for (const RoadPoint& p : road.points)
        {
            if (!ValidPoint(p.position, p.heading.value_or(0.0), p.pitch) || !std::isfinite(p.roll) || !ValidHandles(p))
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
            out << ' ' << p.incomingHandle << ' ' << p.outgoingHandle << '\n';
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
            if (!in || !ValidPoint(p.position, heading, p.pitch) || !std::isfinite(p.roll) || !ValidHandles(p))
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
