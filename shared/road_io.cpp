#include "road_io.hpp"

#include <cmath>
#include <cstdint>
#include <istream>
#include <ostream>

namespace
{
    bool ValidStyle(int lanes, int layout, double dividedGap, int edge)
    {
        return lanes >= 1 && lanes <= RoadStyle::maxLanes &&
               layout >= static_cast<int>(ThreeLaneLayout::TwoNorthbound) &&
               layout <= static_cast<int>(ThreeLaneLayout::CenterTurn) &&
               std::isfinite(dividedGap) && dividedGap >= 0.0 &&
               edge >= static_cast<int>(RoadEdge::None) && edge <= static_cast<int>(RoadEdge::Curb);
    }

    bool ValidPoint(Vector3 position, double heading, double pitch)
    {
        return std::isfinite(position.x) && std::isfinite(position.y) &&
               std::isfinite(position.z) && std::isfinite(heading) && std::isfinite(pitch);
    }
}

namespace RoadIO
{
    void WriteStyle(std::ostream& out, const RoadStyle& style)
    {
        const int edge = static_cast<int>(style.edge);
        if (!ValidStyle(style.GetLaneCount(), static_cast<int>(style.threeLaneLayout), style.dividedGap, edge))
        {
            out.setstate(std::ios::failbit);
            return;
        }
        out << style.GetLaneCount() << ' ' << style.curved << ' ' << style.leftShoulder << ' '
            << style.rightShoulder << ' ' << static_cast<int>(style.threeLaneLayout) << ' '
            << style.divided << ' ' << style.dividedGap << ' ' << edge << '\n';
    }

    void ReadStyle(std::istream& in, RoadStyle& style, std::uint32_t version)
    {
        int lanes, layout;
        int edge = static_cast<int>(RoadEdge::None);
        in >> lanes >> style.curved >> style.leftShoulder >> style.rightShoulder >> layout
           >> style.divided >> style.dividedGap;
        if (version >= 2)
        {
            in >> edge;
        }
        if (!in || !ValidStyle(lanes, layout, style.dividedGap, edge))
        {
            in.setstate(std::ios::failbit);
            return;
        }
        style.SetLaneCount(lanes);
        style.threeLaneLayout = static_cast<ThreeLaneLayout>(layout);
        style.edge = static_cast<RoadEdge>(edge);
    }

    void WriteRoad(std::ostream& out, const Road& road)
    {
        WriteStyle(out, road.style);
        out << road.points.size() << '\n';
        for (const RoadPoint& p : road.points)
        {
            if (!ValidPoint(p.position, p.heading.value_or(0.0), p.pitch))
            {
                out.setstate(std::ios::failbit);
                return;
            }
            out << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' '
                << p.heading.has_value() << ' ' << p.heading.value_or(0.0) << ' ' << p.pitch << '\n';
        }
    }

    Road ReadRoad(std::istream& in, std::uint32_t version)
    {
        Road road;
        ReadStyle(in, road.style, version);
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
            if (!in || !ValidPoint(p.position, heading, p.pitch))
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
