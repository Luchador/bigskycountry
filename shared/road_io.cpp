#include "road_io.hpp"

#include <cmath>
#include <cstdint>
#include <istream>
#include <ostream>

namespace
{
    bool ValidStyle(int lanes, int layout, double dividedGap)
    {
        return lanes >= 1 && lanes <= RoadStyle::maxLanes &&
               layout >= static_cast<int>(ThreeLaneLayout::TwoNorthbound) &&
               layout <= static_cast<int>(ThreeLaneLayout::CenterTurn) &&
               std::isfinite(dividedGap) && dividedGap >= 0.0;
    }

    bool ValidPoint(Vector3 position, double heading)
    {
        return std::isfinite(position.x) && std::isfinite(position.y) &&
               std::isfinite(position.z) && std::isfinite(heading);
    }
}

namespace RoadIO
{
    void WriteStyle(std::ostream& out, const RoadStyle& style)
    {
        if (!ValidStyle(style.GetLaneCount(), static_cast<int>(style.threeLaneLayout), style.dividedGap))
        {
            out.setstate(std::ios::failbit);
            return;
        }
        out << style.GetLaneCount() << ' ' << style.curved << ' ' << style.leftShoulder << ' '
            << style.rightShoulder << ' ' << static_cast<int>(style.threeLaneLayout) << ' '
            << style.divided << ' ' << style.dividedGap << '\n';
    }

    void ReadStyle(std::istream& in, RoadStyle& style)
    {
        int lanes, layout;
        in >> lanes >> style.curved >> style.leftShoulder >> style.rightShoulder >> layout
           >> style.divided >> style.dividedGap;
        if (!in || !ValidStyle(lanes, layout, style.dividedGap))
        {
            in.setstate(std::ios::failbit);
            return;
        }
        style.SetLaneCount(lanes);
        style.threeLaneLayout = static_cast<ThreeLaneLayout>(layout);
    }

    void WriteRoad(std::ostream& out, const Road& road)
    {
        WriteStyle(out, road.style);
        out << road.points.size() << '\n';
        for (const RoadPoint& p : road.points)
        {
            if (!ValidPoint(p.position, p.heading.value_or(0.0)))
            {
                out.setstate(std::ios::failbit);
                return;
            }
            out << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' '
                << p.heading.has_value() << ' ' << p.heading.value_or(0.0) << '\n';
        }
    }

    Road ReadRoad(std::istream& in)
    {
        Road road;
        ReadStyle(in, road.style);
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
            if (!in || !ValidPoint(p.position, heading))
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
