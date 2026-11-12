#include "road.hpp"
#include "road_curve.hpp"
#include "mesh.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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

    using namespace RoadSpline;

    double HalfLengthParameter(const Cubic& curve)
    {
        struct Sample { double distance, t; };
        std::vector<Sample> samples{{0, 0}};
        Vector3 previous = curve.a;
        SampleCubic(curve, 0.001, 0, 0, 1, [&](Vector3 position, Vector3, double t)
        {
            samples.push_back({samples.back().distance + Math::Length(position - previous), t});
            previous = position;
        });
        const double half = samples.back().distance * 0.5;
        if (!(half > 0) || !std::isfinite(half)) return 0.5;
        const auto next = std::ranges::lower_bound(samples, half, {}, &Sample::distance);
        const auto& last = *(next - 1);
        return std::lerp(last.t, next->t, (half - last.distance) / (next->distance - last.distance));
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

double Road::ThreeLaneDividerOffset() const
{
    // Use the authored road frame so trimming its ends at junctions cannot swap lane directions.
    const auto frame = ObjectTransform{{}, rotation}.ToMeshTransform();
    const Vector3 travel = frame.Unrotate(points.back().position - points.front().position);
    // Whole-road rotations can leave a rounding residual on an east-west road.
    const bool eastWest = std::abs(travel.z) <= 1e-12 * std::hypot(travel.x, travel.z);
    const bool northFollowsPoints = eastWest ? travel.x >= 0.0 : travel.z > 0.0;
    const bool twoNorthbound = style.threeLaneLayout == ThreeLaneLayout::TwoNorthbound;
    constexpr double halfLane = RoadStyle::laneWidth * 0.5;
    return twoNorthbound == northFollowsPoints ? halfLane : -halfLane;
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

bool Road::SubdivideSegments(const std::set<int>& segments)
{
    if (segments.empty() || *segments.begin() < 0 || static_cast<std::size_t>(*segments.rbegin()) + 1 >= points.size())
        return false;

    std::vector<RoadPoint> result;
    result.reserve(points.size() + segments.size());
    result.push_back(points.front());
    // Automatic tangents can change at the original ends when their neighbors change.
    std::vector<std::pair<std::size_t, Vector3>> originalTangents;
    for (std::size_t i = 0; i + 1 < points.size(); ++i)
    {
        RoadPoint end = points[i + 1];
        if (!segments.contains(static_cast<int>(i)))
        {
            result.push_back(end);
            continue;
        }
        RoadPoint& start = result.back();
        RoadPoint middle{};
        middle.position = start.position + (end.position - start.position) * 0.5;
        middle.roll = start.roll + 0.5 * std::remainder(end.roll - start.roll, 2 * std::numbers::pi);
        if (style.curved)
        {
            const Vector3 startTangent = Tangent(i), endTangent = Tangent(i + 1);
            if (!start.heading && !segments.contains(static_cast<int>(i) - 1))
                originalTangents.emplace_back(result.size() - 1, startTangent);
            if (!end.heading)
                originalTangents.emplace_back(result.size() + 1, endTangent);
            const Cubic curve = SegmentCurve(start, end, startTangent, endTangent);
            const auto [left, right] = Split(curve, HalfLengthParameter(curve));
            const double leftChord = Math::Length(left.d - left.a), rightChord = Math::Length(right.d - right.a);
            if (leftChord == 0 || rightChord == 0 || !std::isfinite(leftChord) || !std::isfinite(rightChord)) return false;
            middle.position = left.d;
            SetTangent(middle, right.b - left.c);
            start.outgoingHandle = Math::Length(left.b - left.a) / leftChord;
            middle.incomingHandle = Math::Length(left.d - left.c) / leftChord;
            middle.outgoingHandle = Math::Length(right.b - right.a) / rightChord;
            end.incomingHandle = Math::Length(right.d - right.c) / rightChord;
        }
        else if (middle.position == start.position || middle.position == end.position) return false;
        result.push_back(middle);
        result.push_back(end);
    }
    points = std::move(result);
    for (const auto& [i, tangent] : originalTangents)
        if (Math::Length(Tangent(i) - tangent) > 1e-12) SetTangent(points[i], tangent);
    Rebuild();
    return true;
}

bool Road::SetPose(Vector3 position, Vector3 angles)
{
    if (!TransformControlPoints(position, angles)) return false;
    Rebuild();
    return true;
}

bool Road::TransformControlPoints(Vector3 position, Vector3 angles)
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
    return true;
}

void Road::SetMouths(const std::array<std::optional<RoadMouth>, 2>& value, bool rebuild)
{
    if (!rebuild && mouths == value) return;
    mouths = value;
    Rebuild();
}

void Road::Rebuild()
{
    MarkRebuilt();
    centerline.clear();
    splineLengthMeters = 0.0;
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
        const Vector3 b = points[i].position;

        if (style.curved)
        {
            const Vector3 endTangent = Tangent(i);
            const Cubic curve = SegmentCurve(points[i - 1], points[i], tangents.back(), endTangent);
            SampleCubic(curve, 0.1, 0, 0, 1, [&](Vector3 position, Vector3 tangent, double)
            {
                centerline.push_back(position);
                tangents.push_back(tangent);
            });
            // Preserve the authored direction even when a segment has zero length.
            tangents.back() = endTangent;
        }
        else
        {
            centerline.push_back(b);
        }

        pointSamples.push_back(centerline.size() - 1);
    }

    // Measure the authored centerline before intersection mouths trim the pavement.
    for (std::size_t i = 1; i < centerline.size(); ++i)
    {
        splineLengthMeters += Math::Length(centerline[i] - centerline[i - 1]);
    }

    // Sweep and wind the cross section in the road's own frame, including curbs and paint.
    const bool rotated = rotation != Vector3{};
    const MeshTransform frame = ObjectTransform{points.front().position, rotation}.ToMeshTransform();
    std::array<std::optional<RoadMouthFrame>, 2> caps{};
    std::vector<Vector3> trimmed;
    const std::vector<Vector3>* surfacePath = &centerline;
    std::size_t first = 0, last = centerline.size() - 1;
    for (int end = 0; end < 2; ++end)
    {
        const auto& mouth = mouths[end];
        const auto& point = end == 0 ? points.front() : points.back();
        if (!mouth || !point.anchor || point.position != mouth->anchor) continue;
        // Only trim the incident end. A later section of the same road may loop back.
        // The whole cross section must clear the cut. On an oblique approach the
        // inside shoulder can still cross the junction after the centerline has left it.
        const double width = std::max(style.LeftWidth(), style.RightWidth()) + style.EdgeWidth();
        const Vector3 outward = frame.Unrotate(mouth->outward);
        const auto clearsMouth = [&](std::size_t i)
        {
            const double run = Math::Dot(centerline[i] - mouth->center, mouth->outward);
            if (run <= 1e-7) return false;
            const auto leftAxis = [&](Vector3 direction)
            {
                const Vector3 local = frame.Unrotate(direction);
                return Math::Normalize(Vector3{-local.z, 0, local.x});
            };
            double extent;
            if (!tangents.empty())
            {
                extent = std::abs(Math::Dot(leftAxis(tangents[i]), outward));
            }
            else
            {
                // Bound both the bevel and the miter at a straight-road control point.
                const Vector3 a = leftAxis(i == 0 ? centerline[1] - centerline[0] : centerline[i] - centerline[i - 1]);
                const Vector3 b = i + 1 < centerline.size() ? leftAxis(centerline[i + 1] - centerline[i]) : a;
                extent = std::max(std::abs(Math::Dot(a, outward)), std::abs(Math::Dot(b, outward)));
                const double divisor = 1.0 + Math::Dot(a, b);
                if (divisor > 1e-6) extent = std::max(extent, std::abs(Math::Dot(a + b, outward)) / divisor);
            }
            const double height = style.edge == RoadEdge::Curb ? RoadStyle::curbHeight : RoadStyle::stripeHeight;
            return run > width * extent + std::abs(outward.y) * height + 1e-7;
        };
        if (end == 0)
        {
            while (first < last && !clearsMouth(first + 1)) ++first;
        }
        else
        {
            while (last > first && !clearsMouth(last - 1)) --last;
        }
        if (first == last) return; // No remaining cross section clears the junction mouth.
        const Vector3 heading = mouth->outward * (end == 0 ? 1.0 : -1.0);
        const Vector3 left = Math::Cross(heading, {0,1,0});
        caps[end] = RoadMouthFrame{frame.Unrotate(left), frame.Unrotate({0,1,0})};
    }
    if (caps[0] || caps[1])
    {
        trimmed.assign(centerline.begin() + first, centerline.begin() + last + 1);
        if (!tangents.empty())
            tangents = std::vector<Vector3>(tangents.begin() + first, tangents.begin() + last + 1);
        if (caps[0]) trimmed.front() = mouths[0]->center;
        if (caps[1]) trimmed.back() = mouths[1]->center;
        if (trimmed.size() == 2 && caps[0] && caps[1] &&
            (Math::Dot(trimmed.back() - trimmed.front(), mouths[0]->outward) <= 0 ||
             Math::Dot(trimmed.front() - trimmed.back(), mouths[1]->outward) <= 0)) return;
        surfacePath = &trimmed;
    }
    if (rotated)
    {
        std::vector<Vector3> local;
        local.reserve(surfacePath->size());
        for (Vector3 sample : *surfacePath) local.push_back(frame.Unrotate(sample - frame.origin));
        for (Vector3& tangent : tangents) tangent = frame.Unrotate(tangent);
        BuildSurface(local, tangents, caps);
    }
    else BuildSurface(*surfacePath, tangents, caps);

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
