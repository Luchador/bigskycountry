#include "road.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    Vector3 ManualTangent(const RoadPoint& point)
    {
        const double horizontal = std::cos(point.pitch);
        return {horizontal * std::cos(*point.heading), std::sin(point.pitch), horizontal * std::sin(*point.heading)};
    }

    void SetTangent(RoadPoint& point, Vector3 tangent)
    {
        point.heading = std::atan2(tangent.z, tangent.x);
        point.pitch = std::atan2(tangent.y, std::hypot(tangent.x, tangent.z));
    }

    // Subdivide until the cubic deviates from its linear interpolation by at most 10 cm.
    void SampleCubic(Vector3 a, Vector3 b, Vector3 c, Vector3 d, int depth, std::vector<Vector3>& samples)
    {
        const Vector3 chord = d - a;
        const double error = std::max(Math::Length((b - a) - chord * (1.0 / 3.0)), Math::Length((c - a) - chord * (2.0 / 3.0)));

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

int RoadStyle::GetLaneCount() const
{
    return lanes;
}

void RoadStyle::SetLaneCount(int count)
{
    lanes = std::clamp(count, 1, maxLanes);
}

bool RoadStyle::HasCenterTurnLane() const
{
    if (lanes == 3)
    {
        return threeLaneLayout == ThreeLaneLayout::CenterTurn;
    }

    return lanes >= 5 && lanes % 2 == 1;
}

double RoadStyle::LeftWidth() const
{
    return lanes * laneWidth * 0.5 + (leftShoulder ? shoulderWidth : 0.0);
}

double RoadStyle::RightWidth() const
{
    return lanes * laneWidth * 0.5 + (rightShoulder ? shoulderWidth : 0.0);
}

double RoadStyle::EdgeWidth() const
{
    switch (edge)
    {
        case RoadEdge::Transition: return stripWidth;
        case RoadEdge::Curb: return stripWidth + curbTopWidth;
        default: return 0.0;
    }
}

Vector3 Road::Tangent(std::size_t index) const
{
    const RoadPoint& point = points[index];

    if (style.curved && point.heading)
    {
        return ManualTangent(point);
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
    const Vector3 tangent = Tangent(index);

    if (style.curved)
    {
        points[index].heading = heading;
        points[index].pitch = std::atan2(tangent.y, std::hypot(tangent.x, tangent.z));
        return;
    }

    // Positive plan-view heading turns +X toward +Z, around negative Y.
    RotatePoint(index, {0, -1, 0}, heading - std::atan2(tangent.z, tangent.x));
}

void Road::RotatePoint(std::size_t index, Vector3 unitAxis, double angle)
{
    const double cosine = std::cos(angle), sine = std::sin(angle);

    const auto rotate = [&](Vector3 v)
    {
        return v * cosine + Math::Cross(unitAxis, v) * sine + unitAxis * (Math::Dot(unitAxis, v) * (1.0 - cosine));
    };

    if (style.curved)
    {
        const Vector3 tangent = Tangent(index);
        const Vector3 rotated = rotate(tangent);
    
        if (Math::Length(rotated - tangent) > 1.0e-12)
        {
            SetTangent(points[index], rotated);
        }
    
        return;
    }

    // A straight span is defined by its endpoints. Pivot only the adjacent spans.
    const Vector3 pivot = points[index].position;
    const std::size_t first = index == 0 ? 0 : index - 1;
    const std::size_t last = std::min(index + 1, points.size() - 1);

    for (std::size_t i = first; i <= last; ++i)
    {
        RoadPoint& point = points[i];
        point.position = pivot + rotate(point.position - pivot);

        if (point.heading)
        {
            SetTangent(point, rotate(ManualTangent(point)));
        }
    }
}

std::span<const Vector3> Road::SegmentSamples(std::size_t segment) const
{
    if (pointSamples.empty() || segment >= pointSamples.size() - 1)
    {
        return {};
    }

    const std::size_t start = pointSamples[segment];
    return std::span<const Vector3>(centerline).subspan(start, pointSamples[segment + 1] - start + 1);
}

void Road::Rebuild()
{
    centerline.clear();
    pointSamples.clear();
    triangles.clear();
    edgeTriangles.clear();
    outline.clear();
    whiteStripes.clear();
    yellowStripes.clear();

    if (points.size() < 2)
    {
        return;
    }

    centerline.push_back(points.front().position);
    pointSamples.push_back(0);

    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const Vector3 a = points[i - 1].position;
        const Vector3 b = points[i].position;

        if (style.curved)
        {
            const double arm = Math::Length(b - a) / 3.0;
            SampleCubic(a, a + Tangent(i - 1) * arm, b - Tangent(i) * arm, b, 0, centerline);
        }
        else
        {
            centerline.push_back(b);
        }

        pointSamples.push_back(centerline.size() - 1);
    }

    BuildSurface(centerline);

    // Bevels can turn either way. Give all top surfaces an upward normal once,
    // when the road changes. Vertical curb risers already face the pavement.
    const auto facesDown = [](Vector3 a, Vector3 b, Vector3 c)
    {
        return (b.z - a.z) * (c.x - a.x) < (b.x - a.x) * (c.z - a.z);
    };

    for (auto* mesh : {&triangles, &whiteStripes, &yellowStripes})
    {
        for (Triangle& triangle : *mesh)
        {
            if (facesDown(triangle[0], triangle[1], triangle[2]))
            {
                std::swap(triangle[1], triangle[2]);
            }
        }
    }

    for (auto& triangle : edgeTriangles)
    {
        if (facesDown(triangle[0].position, triangle[1].position, triangle[2].position))
        {
            std::swap(triangle[1], triangle[2]);
        }
    }
}
