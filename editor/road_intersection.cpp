#include "road_intersection.hpp"
#include "road_curve.hpp"
#include "transform_inspector.hpp"

#include <cmath>
#include <limits>

namespace
{
    using namespace RoadSpline;
    struct Location { int segment; double t; };

    Cubic Curve(const Road& road, int i)
    {
        const auto& a = road.points[i];
        const auto& b = road.points[i + 1];
        if (road.style.curved) return SegmentCurve(a, b, road.Tangent(i), road.Tangent(i + 1));
        const auto step = (b.position - a.position) * (1.0 / 3.0);
        return {a.position, a.position + step, b.position - step, b.position};
    }

    Location Closest(const Road& road, RoadPick hit, Vector3 position)
    {
        const int count = static_cast<int>(road.points.size()) - 1;
        const int first = hit.kind == RoadPickKind::Segment ? hit.index : 0;
        const int end = hit.kind == RoadPickKind::Segment ? first + 1 : count;
        Location best{-1, 0};
        double distance = std::numeric_limits<double>::infinity();
        for (int i = first; i < end; ++i)
        {
            const Cubic curve = Curve(road, i);
            Vector3 prev = curve.a;
            double prevT = 0;
            SampleCubic(curve, 0.01, 0, 0, 1, [&](Vector3 next, Vector3, double t)
            {
                const auto chord = next - prev;
                const double length2 = Math::Dot(chord, chord);
                const double u = length2 > 0 ? std::clamp(Math::Dot(position - prev, chord) / length2, 0.0, 1.0) : 0;
                const auto delta = position - (prev + chord * u);
                const double d = Math::Dot(delta, delta);
                if (d < distance) { distance = d; best = {i, std::lerp(prevT, t, u)}; }
                prev = next;
                prevT = t;
            });
        }
        return best;
    }

    // Find the first crossing in travel order, including curves whose far endpoint
    // bends back behind the cut plane. Refine against the cubic inside that bracket.
    template<typename Station>
    std::optional<double> FirstCrossing(const Cubic& curve, const Station& station, double distance)
    {
        if (station(curve.a) >= distance) return 0;
        if (std::max({station(curve.b), station(curve.c), station(curve.d)}) < distance) return {};
        double low = 0;
        std::optional<double> high;
        SampleCubic(curve, 0.01, 0, 0, 1, [&](Vector3 p, Vector3, double t)
        {
            if (high) return;
            if (station(p) >= distance) high = t;
            else low = t;
        });
        if (!high) return {};
        for (int n = 0; n < 44; ++n)
        {
            const double t = (low + *high) * 0.5;
            if (station(Split(curve, t).first.d) < distance) low = t;
            else *high = t;
        }
        return high;
    }

    // Walk outward in original spline order on each side. Points inside the footprint are
    // replaced by the anchor. Beyond a short approach the original Bézier polygon is retained.
    bool Approach(const Road& road, Location at, const Intersection& junction, bool forward,
                  std::vector<RoadPoint>& points)
    {
        const int side = forward ? 0 : 2, step = forward ? 1 : -1;
        const auto axis = Intersection::Directions(junction.yaw)[side];
        const auto station = [&](Vector3 p) { return Math::Dot(p - junction.position, axis); };
        const double radius = Intersection::anchorDistance;
        const int end = static_cast<int>(road.points.size()) - 1;
        int i = at.segment;
        double start = at.t;
        if (forward && start == 1) { ++i; start = 0; }
        if (!forward && start == 0) { --i; start = 1; }
        for (; i >= 0 && i < end; i += step, start = forward ? 0.0 : 1.0)
        {
            const int far = forward ? i + 1 : i;
            const auto& neighbor = road.points[far];
            const double run = station(neighbor.position);
            const auto [left, right] = Split(Curve(road, i), start);
            const Cubic outward = forward ? right : Cubic{left.d, left.c, left.b, left.a};
            const auto onAxis = [&](Vector3 p) {
                return Math::Length(p - (junction.position + axis * station(p))) < 1e-6;
            };
            // A short transition keeps the original spline beyond the flat junction.
            std::optional<double> blend;
            if (!(onAxis(outward.a) && onAxis(outward.b) && onAxis(outward.c) && onAxis(outward.d)))
            {
                blend = FirstCrossing(outward, station, 2 * radius);
                if (blend && Math::Length(neighbor.position - Split(outward, *blend).first.d) < Road::minPointSpacing)
                    blend.reset();
            }
            if (!blend && run < radius + Road::minPointSpacing)
            {
                const Vector3 offset = neighbor.position - junction.position;
                const double lateral = Math::Cross(axis, offset).y * step;
                if (run < -Road::minPointSpacing || lateral > road.style.RightWidth() || -lateral > road.style.LeftWidth()) return false;
                continue;
            }

            const Vector3 anchorPosition = junction.AnchorPosition(side);
            auto cut = blend;
            if (!cut) cut = FirstCrossing(outward, station, radius);
            if (!cut) return false;
            const auto kept = Split(outward, *cut).second;
            RoadPoint anchor{anchorPosition, -junction.yaw};
            RoadPoint join = anchor;
            if (blend)
            {
                join.position = kept.a;
                const auto tangent = (kept.b - kept.a) * step;
                join.heading = std::atan2(tangent.z, tangent.x);
                join.pitch = std::atan2(tangent.y, std::hypot(tangent.x, tangent.z));
                const double t = std::lerp(start, forward ? 1.0 : 0.0, *cut);
                join.roll = road.points[i].roll + t * std::remainder(road.points[i + 1].roll - road.points[i].roll, 2 * std::numbers::pi);
            }
            if (!ValidRoadSpan(road, join.position, neighbor.position) ||
                (blend && !ValidRoadSpan(road, anchor.position, join.position))) return false;

            RoadPoint retained = neighbor;
            if (road.style.curved)
            {
                const double chord = Math::Length(retained.position - join.position);
                (forward ? retained.incomingHandle : retained.outgoingHandle) = Math::Length(kept.d - kept.c) / chord;
                (forward ? join.outgoingHandle : join.incomingHandle) = Math::Length(kept.b - kept.a) / chord;
                const auto previousDirection = Math::Normalize(retained.position - road.points[far - step].position);
                const auto trimmedDirection = Math::Normalize(retained.position - join.position);
                if (!retained.heading && Math::Length(trimmedDirection - previousDirection) > 1e-12)
                {
                    const auto tangent = road.Tangent(far);
                    retained.heading = std::atan2(tangent.z, tangent.x);
                    retained.pitch = std::atan2(tangent.y, std::hypot(tangent.x, tangent.z));
                }
            }
            if (forward)
            {
                if (blend) points.push_back(anchor);
                points.push_back(join);
                points.push_back(retained);
                points.insert(points.end(), road.points.begin() + far + 1, road.points.end());
            }
            else
            {
                points.assign(road.points.begin(), road.points.begin() + far);
                points.push_back(retained);
                points.push_back(join);
                if (blend) points.push_back(anchor);
            }
            return true;
        }
        return false;
    }
}

std::optional<RoadIntersectionInsertion> PlanRoadIntersection(const Road& road, RoadPick hit, Vector3 position)
{
    const int last = static_cast<int>(road.points.size()) - 1;
    if (last < 1 || road.style.railroad) return {};
    Location at{};
    if (hit.kind == RoadPickKind::Point)
    {
        if (hit.index < 0 || hit.index > last) return {};
        at = hit.index == last ? Location{last - 1, 1} : Location{hit.index, 0};
    }
    else
    {
        if (hit.kind == RoadPickKind::Segment && (hit.index < 0 || hit.index >= last)) return {};
        at = Closest(road, hit, position);
        if (at.segment < 0) return {};
    }
    const auto [left, right] = Split(Curve(road, at.segment), at.t);
    const bool atStart = at.segment == 0 && at.t == 0;
    const bool atEnd = at.segment == last - 1 && at.t == 1;
    if ((atStart && road.points.front().Joined()) ||
        (atEnd && road.points.back().Joined())) return {};
    const bool knot = at.t == 0 || at.t == 1;
    const int point = at.segment + (at.t == 1);
    const auto tangent = knot ? road.Tangent(point) : Math::Normalize(right.b - left.c);
    if (std::hypot(tangent.x, tangent.z) < 1e-8) return {};
    RoadIntersectionInsertion plan;
    plan.junction.position = knot ? road.points[point].position : left.d;
    plan.junction.yaw = -std::atan2(tangent.z, tangent.x);
    plan.junction.material = road.style.material;
    plan.junction.pavedThrough = true;
    for (int i = 0; i < 2; ++i)
    {
        const bool forward = i == 1;
        if ((atStart && !forward) || (atEnd && forward)) continue;
        Road& piece = plan.approaches.emplace_back(road.NewSection());
        if (!Approach(road, at, plan.junction, forward, piece.points)) return {};

        // Use the renderer's own trimming rules. The full cross section, including
        // shoulders and curbs, must still clear both this junction and an existing far mouth.
        const int side = forward ? 0 : 2;
        auto& end = forward ? piece.points.front() : piece.points.back();
        end.anchor = IntersectionAnchor{0, side}; // The world assigns the new ID on commit.
        std::array<std::optional<RoadMouth>, 2> mouths;
        mouths[i] = road.EndpointMouth(!forward);
        mouths[1 - i] = RoadMouth{end.position, end.position, Intersection::Directions(plan.junction.yaw)[side]};
        piece.SetMouths(mouths);
        if (piece.pavementTriangles.empty()) return {};
    }
    return plan;
}
