#include "road.hpp"
#include "mesh.hpp"

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
    void SampleCubic(Vector3 a, Vector3 b, Vector3 c, Vector3 d, int depth, std::vector<Vector3>& samples,
                     std::vector<Vector3>& tangents)
    {
        const Vector3 chord = d - a;
        const double error = std::max(Math::Length((b - a) - chord * (1.0 / 3.0)), Math::Length((c - a) - chord * (2.0 / 3.0)));

        if (error <= 0.1 || depth == 12)
        {
            samples.push_back(d);
            // The derivative at this subcurve's endpoint is parallel to its last control-polygon edge.
            tangents.push_back(d - c);
            return;
        }

        const Vector3 ab = (a + b) * 0.5;
        const Vector3 bc = (b + c) * 0.5;
        const Vector3 cd = (c + d) * 0.5;
        const Vector3 abc = (ab + bc) * 0.5;
        const Vector3 bcd = (bc + cd) * 0.5;
        const Vector3 middle = (abc + bcd) * 0.5;
        SampleCubic(a, ab, abc, middle, depth + 1, samples, tangents);
        SampleCubic(middle, bcd, cd, d, depth + 1, samples, tangents);
    }
}

int RoadStyle::GetLaneCount() const
{
    return lanes;
}

void RoadStyle::SetLaneCount(int count)
{
    lanes = std::clamp(count, 1, maxLanes);
    if (!CanDivide()) divided = false;
}

void RoadStyle::SetDividedGap(double meters)
{
    if (std::isfinite(meters)) dividedGap = std::max(minSeparation, meters);
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
    return HalfSeparation() + lanes * laneWidth * 0.5 + (leftShoulder ? shoulderWidth : 0.0);
}

double RoadStyle::RightWidth() const
{
    return HalfSeparation() + lanes * laneWidth * 0.5 + (rightShoulder ? shoulderWidth : 0.0);
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

std::array<Vector3, 3> Road::PointAxes(std::size_t point, bool includeRoll) const
{
    const Vector3 tangent = Tangent(point);
    Vector3 heading = tangent;
    const auto vertical = [](Vector3 v) { return std::hypot(v.x, v.z) < 1.0e-8; };
    if (vertical(tangent))
    {
        const RoadPoint& pivot = points[point];
        if (pivot.heading)
        {
            heading = {std::cos(*pivot.heading), 0, std::sin(*pivot.heading)};
        }
        else
        {
            // Vertical spans inherit the preceding heading. At the start, look ahead.
            for (std::size_t i = point; i > 0 && vertical(heading); --i)
            {
                heading = pivot.position - points[i - 1].position;
            }
            for (std::size_t i = point + 1; i < points.size() && vertical(heading); ++i)
            {
                heading = points[i].position - pivot.position;
            }
        }
    }
    const Vector3 across = vertical(heading) ? Vector3{0, 0, 1} : Vector3{-heading.z, 0, heading.x};
    const Vector3 up = Math::Normalize(Math::Cross(across, tangent));
    const Vector3 side = Math::Cross(tangent, up);
    const double roll = includeRoll ? points[point].roll : 0.0;
    return {tangent, up * std::cos(roll) + side * std::sin(roll),
            side * std::cos(roll) - up * std::sin(roll)};
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
    const MeshTransform turn = MeshTransform::AxisRotation(unitAxis, angle);
    const Vector3 rotatedUp = turn.Direction(PointAxes(index)[1]);
    if (style.curved)
    {
        const Vector3 tangent = Tangent(index);
        const Vector3 rotated = turn.Direction(tangent);

        if (Math::Length(rotated - tangent) > 1.0e-12)
        {
            SetTangent(points[index], rotated);
        }
    }
    else
    {
        // A straight span is defined by its endpoints. Pivot only the adjacent spans.
        const Vector3 pivot = points[index].position;
        const std::size_t first = index == 0 ? 0 : index - 1;
        const std::size_t last = std::min(index + 1, points.size() - 1);

        for (std::size_t i = first; i <= last; ++i)
        {
            RoadPoint& point = points[i];
            point.position = pivot + turn.Direction(point.position - pivot);

            if (point.heading)
            {
                SetTangent(point, turn.Direction(ManualTangent(point)));
            }
        }
    }
    const auto base = PointAxes(index, false);
    points[index].roll = std::atan2(Math::Dot(rotatedUp, base[2]), Math::Dot(rotatedUp, base[1]));
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

Road Road::Section(std::size_t first, std::size_t last) const
{
    Road result;
    result.style = style;
    result.rotation = rotation;
    result.points.assign(points.begin() + first, points.begin() + last + 1);
    // An automatic tangent at a cut may depend on the neighbor we just removed.
    // Keep the surviving Bézier span's original shape at that new endpoint.
    const auto preserveTangent = [&](std::size_t source, std::size_t endpoint)
    {
        RoadPoint& p = result.points[endpoint];
        const Vector3 tangent = Tangent(source);
        if (!p.heading && Math::Length(tangent - result.Tangent(endpoint)) > 1.0e-12)
        {
            SetTangent(p, tangent);
            if (tangent.x == 0.0 && tangent.z == 0.0)
            {
                const auto axes = PointAxes(source, false);
                p.heading = std::atan2(-axes[2].x, axes[2].z);
            }
        }
    };
    if (style.curved)
    {
        if (first > 0) preserveTangent(first, 0);
        if (last + 1 < points.size()) preserveTangent(last, last - first);
    }
    result.Rebuild();
    return result;
}

bool Road::SetPose(Vector3 position, Vector3 angles)
{
    if (points.empty() || !std::isfinite(Math::Length(angles)) || !std::isfinite(Math::Length(position)))
    {
        return false;
    }
    const Vector3 pivot = points.front().position;
    const MeshTransform from = ObjectTransform{{}, rotation}.ToMeshTransform();
    const MeshTransform to = ObjectTransform{{}, angles}.ToMeshTransform();
    const auto rotate = [&](Vector3 v) { return to.Direction(from.Unrotate(v)); };
    auto moved = points;
    for (RoadPoint& point : moved)
    {
        const Vector3 offset = point.position - pivot;
        point.position = angles == rotation ? point.position + (position - pivot) : position + rotate(offset);
        if (!std::isfinite(Math::Length(point.position)))
        {
            return false;
        }
        if (point.heading && angles != rotation)
        {
            SetTangent(point, rotate(ManualTangent(point)));
        }
    }
    moved.front().position = position;
    std::vector<Vector3> up;
    if (angles != rotation)
    {
        up.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) up.push_back(rotate(PointAxes(i)[1]));
    }
    points = std::move(moved);
    for (std::size_t i = 0; i < up.size(); ++i)
    {
        const auto base = PointAxes(i, false);
        points[i].roll = std::atan2(Math::Dot(up[i], base[2]), Math::Dot(up[i], base[1]));
    }
    rotation = angles;
    Rebuild();
    return true;
}

void Road::Rebuild()
{
    centerline.clear();
    pointSamples.clear();
    pavementTriangles.clear();
    edgeTriangles.clear();
    outline.clear();
    whiteStripes.clear();
    yellowStripes.clear();

    if (points.size() < 2)
    {
        return;
    }

    std::vector<Vector3> tangents;
    if (style.curved) tangents.push_back(Tangent(0));
    centerline.push_back(points.front().position);
    pointSamples.push_back(0);

    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const Vector3 a = points[i - 1].position;
        const Vector3 b = points[i].position;

        if (style.curved)
        {
            const double arm = Math::Length(b - a) / 3.0;
            const Vector3 endTangent = Tangent(i);
            SampleCubic(a, a + Tangent(i - 1) * arm, b - endTangent * arm, b, 0, centerline, tangents);
            // Preserve the authored direction even when a segment has zero length.
            tangents.back() = endTangent;
        }
        else
        {
            centerline.push_back(b);
        }

        pointSamples.push_back(centerline.size() - 1);
    }

    // Sweep and wind the cross section in the road's own frame, including curbs and paint.
    const bool rotated = rotation != Vector3{};
    const MeshTransform frame = ObjectTransform{points.front().position, rotation}.ToMeshTransform();
    if (rotated)
    {
        std::vector<Vector3> local;
        local.reserve(centerline.size());
        for (Vector3 sample : centerline)
        {
            local.push_back(frame.Unrotate(sample - frame.origin));
        }
        for (Vector3& tangent : tangents) tangent = frame.Unrotate(tangent);
        BuildSurface(local, tangents);
    }
    else
    {
        BuildSurface(centerline, tangents);
    }

    // Bevels can turn either way. Give all top surfaces an upward normal once,
    // when the road changes. Vertical curb risers already face the pavement.
    const auto facesDown = [](Vector3 a, Vector3 b, Vector3 c)
    {
        return (b.z - a.z) * (c.x - a.x) < (b.x - a.x) * (c.z - a.z);
    };

    for (auto* mesh : {&whiteStripes, &yellowStripes})
    {
        for (Triangle& triangle : *mesh)
        {
            if (facesDown(triangle[0], triangle[1], triangle[2]))
            {
                std::swap(triangle[1], triangle[2]);
            }
        }
    }

    const auto windSurface = [&](auto& surface)
    {
        for (auto& triangle : surface)
        {
            if (facesDown(triangle[0].position, triangle[1].position, triangle[2].position))
            {
                std::swap(triangle[1], triangle[2]);
            }
        }
    };
    windSurface(pavementTriangles);
    windSurface(edgeTriangles);

    if (rotated)
    {
        for (auto* mesh : {&whiteStripes, &yellowStripes})
        {
            for (Triangle& triangle : *mesh)
            {
                for (Vector3& vertex : triangle)
                {
                    vertex = frame.Point(vertex);
                }
            }
        }
        const auto moveSurface = [&](auto& surface)
        {
            for (auto& triangle : surface)
            {
                for (auto& vertex : triangle) vertex.position = frame.Point(vertex.position);
            }
        };
        moveSurface(pavementTriangles);
        moveSurface(edgeTriangles);
        for (Vector3& vertex : outline)
        {
            vertex = frame.Point(vertex);
        }
    }
}
