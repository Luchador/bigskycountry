#include "intersection.hpp"
#include "view_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr int sides = IntersectionAnchor::sideCount;
    constexpr Vector3 up{0, 1, 0};
    constexpr int cornerIntervals = 12;

    struct EdgeProfile
    {
        double apron, rise, cap;
        SDL_FColor inner, outer;
        double Width() const { return apron + cap; }
    };

    EdgeProfile Profile(RoadEdge edge)
    {
        if (edge == RoadEdge::Curb)
            return {RoadStyle::stripWidth, RoadStyle::curbHeight, RoadStyle::curbTopWidth, RoadEdgeColor::apron, RoadEdgeColor::apron};
        if (edge == RoadEdge::Transition) return {RoadStyle::stripWidth, 0, 0, RoadEdgeColor::transitionInner, RoadEdgeColor::transitionOuter};
        return {};
    }

    SDL_FColor Mix(SDL_FColor a, SDL_FColor b, double t)
    {
        const float f = static_cast<float>(t);
        return {std::lerp(a.r, b.r, f), std::lerp(a.g, b.g, f), std::lerp(a.b, b.b, f), 1};
    }

    struct Rim
    {
        std::array<Vector3, 4> p; // Pavement edge, apron edge, top of riser, outer rim.
        SDL_FColor inner, outer;
    };

    // The outer rim follows the corner curve. Insetting it builds a concentric
    // pavement edge, even when the radius is smaller than the curb width.
    Rim Section(Vector3 outer, Vector3 normal, EdgeProfile a, EdgeProfile b, double t)
    {
        const double cap = std::lerp(a.cap, b.cap, t);
        const double height = std::lerp(a.rise, b.rise, t);
        const Vector3 apron = outer - normal * cap;
        if (a.Width() == 0) { a.inner = b.inner; a.outer = b.outer; }
        if (b.Width() == 0) { b.inner = a.inner; b.outer = a.outer; }
        return {{apron - normal * std::lerp(a.apron, b.apron, t), apron, apron + up * height, outer + up * height},
                Mix(a.inner, b.inner, t), Mix(a.outer, b.outer, t)};
    }

    void Pavement(IntersectionSurface& surface, const std::array<Vector3, sides>& axes, Vector3 a, Vector3 b, Vector3 c)
    {
        const auto vertex = [&](Vector3 p)
        {
            const Vector3 d = p - surface.layout.center;
            return RoadGeometry::PavementVertex{p, {static_cast<float>(Math::Dot(d, axes[0]) / RoadStyle::laneWidth), static_cast<float>(Math::Dot(d, axes[1]) / RoadStyle::laneWidth)}};
        };
        const double area = Math::Cross(b - a, c - a).y;
        if (std::abs(area) < 1e-12) return;
        if (area < 0) std::swap(b, c);
        surface.geometry.pavementTriangles.push_back({vertex(a), vertex(b), vertex(c)});
    }

    void Join(IntersectionSurface& surface, const Rim& a, const Rim& b)
    {
        for (int i = 0; i < 3; ++i)
        {
            const SDL_FColor curbColor = i == 1 ? RoadEdgeColor::riser : RoadEdgeColor::top;
            using Vertex = RoadGeometry::EdgeVertex;
            const Vertex v0{a.p[i], i == 0 ? a.inner : curbColor};
            const Vertex v1{b.p[i], i == 0 ? b.inner : curbColor};
            const Vertex v2{b.p[i + 1], i == 0 ? b.outer : curbColor};
            const Vertex v3{a.p[i + 1], i == 0 ? a.outer : curbColor};
            if (v0.position != v1.position && v1.position != v2.position)
                surface.geometry.edgeTriangles.push_back({v0, v1, v2});
            if (v0.position != v3.position && v2.position != v3.position)
                surface.geometry.edgeTriangles.push_back({v0, v2, v3});
        }
    }

    double OuterRadius(double pavementRadius, double edgeA, double edgeB)
    {
        return std::max(0.0, pavementRadius - std::max(edgeA, edgeB));
    }

    void PaintStopLine(RoadGeometry& geometry, Vector3 mouth, Vector3 outward, Vector3 left,
                       const IntersectionApproach::LaneSpan& lanes)
    {
        // Clip the paint rectangle to the pavement. The back edge of an asymmetric
        // T can taper inside the lane edge between the two ends of the rectangle.
        const std::array normals{left, left * -1, outward, outward * -1};
        const double station = Math::Dot(mouth, outward);
        const double lateral = Math::Dot(mouth, left);
        const std::array offsets{lateral + lanes.inner, -lateral - lanes.outer,
                                 station - Intersection::stopLineWidth, -station};
        for (const auto& triangle : geometry.pavementTriangles)
        {
            ViewGeometry::Polygon paint{{triangle[0].position, triangle[1].position, triangle[2].position}, 3};
            for (std::size_t i = 0; i < normals.size() && paint.count >= 3; ++i)
                paint = ViewGeometry::Clip(paint, normals[i], offsets[i]);

            const Vector3 lift = up * RoadStyle::stripeHeight;
            for (int i = 2; i < paint.count; ++i)
            {
                const Vector3 a = paint.points[0], b = paint.points[i - 1], c = paint.points[i];
                if (Math::Cross(b - a, c - a).y > 0)
                    geometry.whiteStripes.push_back({a + lift, b + lift, c + lift});
            }
        }
    }

    int NextArm(const IntersectionLayout& layout, int side)
    {
        do { side = (side + 1) % sides; } while (!layout.arms[side]);
        return side;
    }
}

std::array<Vector3, IntersectionAnchor::sideCount> Intersection::Directions(double yaw)
{
    if (yaw == 0) return IntersectionAnchor::directions;
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {{{c, 0, -s}, {s, 0, c}, {-c, 0, s}, {-s, 0, -c}}};
}

IntersectionApproach IntersectionApproach::FromRoad(const Road& road, bool first, bool paintStopLine)
{
    const auto& style = road.style;
    IntersectionApproach arm{first ? style.LeftWidth() : style.RightWidth(),
                             first ? style.RightWidth() : style.LeftWidth(), style.edge, {}};
    if (!paintStopLine) return arm;

    const int lanes = style.GetLaneCount();
    const double halfLane = RoadStyle::laneWidth * 0.5;
    const double outer = style.HalfSeparation() + lanes * halfLane;
    double inner = style.HalfSeparation();
    if (lanes == 1) inner = -halfLane; // Shared, unstriped lane.
    else if (style.HasCenterTurnLane()) inner = halfLane;
    else if (lanes == 3) inner = road.ThreeLaneDividerOffset() * (first ? 1 : -1);
    arm.stopLine = LaneSpan{inner, outer};
    return arm;
}

void IntersectionSurface::Build(const IntersectionLayout& input)
{
    layout = input;
    const auto directions = Intersection::Directions(layout.yaw);
    geometry = {};
    geometry.MarkRebuilt();
    setbacks.fill(Intersection::anchorDistance);
    // Provide space for the full facing shoulder of the cross street and the fillet.
    // Only the rendered mouths move. The four authored connection anchors stay put.
    for (int i = 0; i < sides; ++i)
    {
        const int j = (i + 1) % sides;
        if (!layout.arms[i] || !layout.arms[j]) continue;
        const auto& a = *layout.arms[i];
        const auto& b = *layout.arms[j];
        const double wa = Profile(a.edge).Width(), wb = Profile(b.edge).Width();
        const double r = OuterRadius(layout.radius, wa, wb);
        setbacks[i] = std::max(setbacks[i], b.right + wb + r);
        setbacks[j] = std::max(setbacks[j], a.left + wa + r);
    }

    for (int i = 0; i < sides; ++i)
    {
        if (!layout.arms[i]) continue;
        const int j = NextArm(layout, i);
        const auto& a = *layout.arms[i];
        const auto& b = *layout.arms[j];
        const Vector3 d = directions[i], e = directions[j], n = directions[(i + 1) % sides];
        const EdgeProfile pa = Profile(a.edge), pb = Profile(b.edge);
        const double wa = a.left + pa.Width(), wb = b.right + pb.Width();
        const Vector3 center = layout.center;
        const Vector3 mouth = center + d * setbacks[i];
        const bool adjacent = j == (i + 1) % sides;
        const Vector3 nextMouth = center + e * setbacks[j];
        if (adjacent)
        {
            // Split each mouth at the centerline, including across a median tip.
            Pavement(*this, directions, center, mouth, mouth + n * a.left);
            Pavement(*this, directions, center, nextMouth + d * b.right, nextMouth);
        }
        Vector3 lastBase = mouth;
        std::optional<Rim> previous;
        auto emit = [&](const Rim& rim, double t)
        {
            if (previous)
            {
                if (adjacent) Pavement(*this, directions, center, previous->p[0], rim.p[0]);
                else
                {
                    // A widening T back may be concave relative to the junction center.
                    const Vector3 base = mouth + (nextMouth - mouth) * t;
                    Pavement(*this, directions, lastBase, base, rim.p[0]);
                    Pavement(*this, directions, lastBase, rim.p[0], previous->p[0]);
                    lastBase = base;
                }
                Join(*this, *previous, rim);
            }
            geometry.outline.push_back(rim.p.back());
            previous = rim;
        };
        const auto add = [&](Vector3 p, Vector3 normal, double t)
        {
            emit(Section(p, normal, pa, pb, t), t);
        };
        add(mouth + n * wa, n, 0);
        if (adjacent)
        {
            const double r = OuterRadius(layout.radius, pa.Width(), pb.Width());
            const auto corner = center + d * wb + e * wa;
            const Vector3 arcCenter = corner + (d + e) * r;
            // Offset each axis by its own edge dimensions. This gives concentric
            // arcs for matching curbs and tangent elliptical arcs for unlike edges.
            const auto cornerSection = [&](double sine, double cosine, double t)
            {
                const Vector3 outer = arcCenter - (d * sine + e * cosine) * r;
                Rim rim = Section(outer, {}, pa, pb, t);
                const Vector3 apron = outer - d * (sine * pb.cap) - e * (cosine * pa.cap);
                rim.p[0] = apron - d * (sine * pb.apron) - e * (cosine * pa.apron);
                rim.p[1] = apron;
                rim.p[2] = apron + up * std::lerp(pa.rise, pb.rise, t);
                emit(rim, t);
            };
            if (r == 0) cornerSection(1, 1, 0.5); // Miter both profiles at the square corner.
            else
            {
                for (int k = 0; k <= cornerIntervals; ++k)
                {
                    const double t = static_cast<double>(k) / cornerIntervals, angle = t * std::numbers::pi / 2;
                    cornerSection(k == cornerIntervals ? 1 : std::sin(angle), k == cornerIntervals ? 0 : std::cos(angle), t);
                }
            }
            add(center + e * setbacks[j] + d * wb, d, 1);
        }
        else
        {
            // Across the closed side of a T there is one continuous boundary.
            // Horizontal end tangents make asymmetric shoulders meet it smoothly.
            const double length = setbacks[i] + setbacks[j];
            const int intervals = wa == wb && a.edge == b.edge ? 1 : cornerIntervals;
            for (int k = 1; k <= intervals; ++k)
            {
                const double t = static_cast<double>(k) / intervals, smooth = t * t * (3 - 2 * t);
                const double slope = (wb - wa) * 6 * t * (1 - t) / length;
                const Vector3 normal = Math::Normalize(n + d * slope);
                add(center + d * (setbacks[i] - length * t) + n * std::lerp(wa, wb, smooth), normal, t);
            }
        }
    }

    for (int i = 0; i < sides; ++i)
    {
        const auto& arm = layout.arms[i];
        if (arm && arm->stopLine)
            PaintStopLine(geometry, layout.center + directions[i] * setbacks[i], directions[i],
                          directions[(i + 1) % sides], *arm->stopLine);
    }
}
