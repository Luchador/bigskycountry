#include "road.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    // Subdivide until the cubic deviates from its linear interpolation by at most 10 cm.
    void SampleCubic(Vector3 a, Vector3 b, Vector3 c, Vector3 d, int depth, std::vector<Vector3>& samples)
    {
        const Vector3 chord = d - a;
        const double error =
            std::max(Math::Length((b - a) - chord * (1.0 / 3.0)), Math::Length((c - a) - chord * (2.0 / 3.0)));
        if (error <= 0.1 || depth == 12)
        {
            samples.push_back(d);
            return;
        }

        const Vector3 ab = (a + b) * 0.5;
        const Vector3 bc = (b + c) * 0.5;
        const Vector3 cd = (c + d) * 0.5;
        const Vector3 abc = (ab + bc) * 0.5;
        const Vector3 bcd = (bc + cd) * 0.5;
        const Vector3 middle = (abc + bcd) * 0.5;
        SampleCubic(a, ab, abc, middle, depth + 1, samples);
        SampleCubic(middle, bcd, cd, d, depth + 1, samples);
    }
}

double RoadStyle::LeftWidth() const
{
    return lanes * laneWidth * 0.5 + (leftShoulder ? shoulderWidth : 0.0);
}

double RoadStyle::RightWidth() const
{
    return lanes * laneWidth * 0.5 + (rightShoulder ? shoulderWidth : 0.0);
}

Vector3 Road::Tangent(std::size_t index) const
{
    const RoadPoint& point = points[index];
    if (style.curved && point.heading)
    {
        return {std::cos(*point.heading), 0.0, std::sin(*point.heading)};
    }

    Vector3 incoming{};
    Vector3 outgoing{};
    if (index > 0)
    {
        incoming = Math::Normalize(point.position - points[index - 1].position);
    }
    if (index + 1 < points.size())
    {
        outgoing = Math::Normalize(points[index + 1].position - point.position);
    }
    const Vector3 sum = incoming + outgoing;
    if (Math::Length(sum) < 1.0e-8)
    {
        return Math::Length(outgoing) > 0.0 ? outgoing : Vector3{1.0, 0.0, 0.0};
    }
    return Math::Normalize(sum);
}

void Road::RotatePoint(std::size_t index, double heading)
{
    if (style.curved)
    {
        points[index].heading = heading;
        return;
    }

    // A straight span's direction is determined by its endpoints. Pivot its
    // neighbors around this point; anchors farther down the road stay fixed.
    const Vector3 axis = Tangent(index);
    const double delta = heading - std::atan2(axis.z, axis.x);
    const double cosine = std::cos(delta);
    const double sine = std::sin(delta);
    const Vector3 pivot = points[index].position;
    const std::size_t first = index == 0 ? 0 : index - 1;
    const std::size_t last = std::min(index + 1, points.size() - 1);
    for (std::size_t i = first; i <= last; ++i)
    {
        const Vector3 offset = points[i].position - pivot;
        points[i].position =
            pivot + Vector3{cosine * offset.x - sine * offset.z, 0.0, sine * offset.x + cosine * offset.z};
        if (points[i].heading)
        {
            *points[i].heading += delta;
        }
    }
}

void Road::Rebuild()
{
    triangles.clear();
    whiteStripes.clear();
    yellowStripes.clear();
    if (points.size() < 2)
    {
        return;
    }

    std::vector<Vector3> samples{points.front().position};
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const Vector3 a = points[i - 1].position;
        const Vector3 b = points[i].position;
        if (style.curved)
        {
            const double arm = Math::Length(b - a) / 3.0;
            SampleCubic(a, a + Tangent(i - 1) * arm, b - Tangent(i) * arm, b, 0, samples);
        }
        else
        {
            samples.push_back(b);
        }
    }

    Vector3 previousLeft{}, previousRight{};
    bool havePrevious = false;
    const double leftWidth = style.LeftWidth();
    const double rightWidth = style.RightWidth();
    for (std::size_t i = 1; i < samples.size(); ++i)
    {
        const Vector3 a = samples[i - 1];
        const Vector3 b = samples[i];
        const Vector3 delta = b - a;
        const double length = std::hypot(delta.x, delta.z);
        if (length == 0.0)
        {
            continue;
        }
        const Vector3 left = {-delta.z / length, 0.0, delta.x / length};
        const Vector3 aLeft = a + left * leftWidth, aRight = a - left * rightWidth;
        const Vector3 bLeft = b + left * leftWidth, bRight = b - left * rightWidth;
        if (havePrevious)
        {
            // Bevel the join without stretching the road at sharp turns.
            triangles.push_back({a, previousLeft, aLeft});
            triangles.push_back({a, previousRight, aRight});
        }
        // Both sides share an explicit center vertex with each bevel. This keeps
        // the joins watertight even when the renderer rounds to pixel centers.
        triangles.push_back({aLeft, a, b});
        triangles.push_back({aLeft, b, bLeft});
        triangles.push_back({a, aRight, bRight});
        triangles.push_back({a, bRight, b});
        previousLeft = bLeft;
        previousRight = bRight;
        havePrevious = true;
    }
    BuildStripes(samples);
}
