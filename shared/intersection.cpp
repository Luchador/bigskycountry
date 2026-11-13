#include "intersection.hpp"
#include "view_geometry.hpp"

#include <CDT.h>
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

    SDL_FPoint SurfaceUV(Vector3 center, const std::array<Vector3, sides>& axes, Vector3 p)
    {
        const Vector3 d = p - center;
        return {float(Math::Dot(d, axes[0]) / RoadStyle::laneWidth), float(Math::Dot(d, axes[1]) / RoadStyle::laneWidth)};
    }

    void Pavement(IntersectionSurface& surface, const std::array<Vector3, sides>& axes, Vector3 a, Vector3 b, Vector3 c)
    {
        const auto vertex = [&](Vector3 p)
        {
            return RoadGeometry::PavementVertex{p, SurfaceUV(surface.layout.center, axes, p)};
        };
        const double area = Math::Cross(b - a, c - a).y;
        if (std::abs(area) < 1e-12) return;
        if (area < 0) std::swap(b, c);
        surface.geometry.pavementTriangles.push_back({vertex(a), vertex(b), vertex(c)});
    }

    void Join(IntersectionSurface& surface, const Rim& a, const Rim& b,
              const IntersectionApproach& approach, const std::array<Vector3, sides>& axes)
    {
        for (int i = 0; i < 3; ++i)
        {
            const SDL_FColor curbColor = i == 1 ? RoadEdgeColor::riser : RoadEdgeColor::top;
            const auto append = [&](auto& mesh, auto v0, auto v1, auto v2, auto v3)
            {
                if (v0.position != v1.position && v1.position != v2.position) mesh.push_back({v0, v1, v2});
                if (v0.position != v3.position && v2.position != v3.position) mesh.push_back({v0, v2, v3});
            };
            if (i == 0 && !approach.transitionMaterial.empty() && approach.edge == RoadEdge::Transition)
            {
                const auto vertex = [&](Vector3 p, SDL_FColor mask)
                {
                    return RoadGeometry::TransitionVertex{p, mask, SurfaceUV(surface.layout.center, axes, p)};
                };
                append(surface.geometry.transitionStrips[approach.transitionMaterial],
                       vertex(a.p[0], RoadEdgeColor::transitionInner), vertex(b.p[0], RoadEdgeColor::transitionInner),
                       vertex(b.p[1], RoadEdgeColor::transitionOuter), vertex(a.p[1], RoadEdgeColor::transitionOuter));
            }
            else
            {
                using Vertex = RoadGeometry::EdgeVertex;
                append(surface.geometry.edgeTriangles,
                       Vertex{a.p[i], i == 0 ? a.inner : curbColor}, Vertex{b.p[i], i == 0 ? b.inner : curbColor},
                       Vertex{b.p[i + 1], i == 0 ? b.outer : curbColor}, Vertex{a.p[i + 1], i == 0 ? a.outer : curbColor});
            }
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

    // The skewed mouths can put the junction center outside a corner's
    // visibility region. Triangulate the entire inner curb boundary in XZ.
    void FillPavement(IntersectionSurface& surface, const std::vector<Vector3>& rim,
                      const std::array<Vector3, sides>& axes)
    {
        std::vector<CDT::V2d<double>> vertices;
        std::vector<CDT::Edge> edges;
        std::vector<Vector3> positions;
        for (Vector3 p : rim)
        {
            if (!positions.empty() && Math::Length(p - positions.back()) < 1e-7) continue;
            positions.push_back(p);
        }
        if (positions.size() > 1 && Math::Length(positions.front() - positions.back()) < 1e-7)
            positions.pop_back();
        for (CDT::VertInd i = 0; i < positions.size(); ++i)
        {
            const auto local = positions[i] - surface.layout.center;
            vertices.push_back({local.x, local.z});
            edges.emplace_back(i, (i + 1) % positions.size());
        }
        if (positions.size() < 3) return;
        CDT::RemoveDuplicatesAndRemapEdges(vertices, edges);
        CDT::Triangulation<double> mesh(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::TryResolve, 0.0);
        mesh.insertVertices(vertices);
        mesh.insertEdges(edges);
        mesh.eraseOuterTrianglesAndHoles();
        for (const auto& triangle : mesh.triangles)
        {
            const auto point = [&](int i)
            {
                const auto& p = mesh.vertices[triangle.vertices[i]];
                return surface.layout.center + Vector3{p.x, 0, p.y};
            };
            Pavement(surface, axes, point(0), point(1), point(2));
        }
    }

    // A skewed road crosses the side's original cut plane obliquely. Cut the
    // true-width road ribbon on that plane, keeping the four authoring pivots fixed.
    struct ApproachCut
    {
        Vector3 axis, lateral;
        double scale, drift;
    };

    ApproachCut Cut(const IntersectionLayout& layout, int side)
    {
        const Vector3 axis = Intersection::Directions(layout.yaw)[side];
        const double angle = layout.skew[side];
        return {axis, Math::Cross(axis, up), 1.0 / std::cos(angle), -std::tan(angle)};
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

std::array<Vector3, IntersectionAnchor::sideCount> Intersection::AnchorPositions() const
{
    auto anchors = Directions(yaw);
    for (auto& anchor : anchors) anchor = position + anchor * anchorDistance;
    return anchors;
}

IntersectionApproach IntersectionApproach::FromRoad(const Road& road, bool first, bool paintStopLine)
{
    const auto& style = road.style;
    IntersectionApproach arm{first ? style.LeftWidth() : style.RightWidth(),
                             first ? style.RightWidth() : style.LeftWidth(), style.edge, {},
                             style.edge == RoadEdge::Transition ? style.transitionMaterial : std::string{}};
    if (!paintStopLine || !road.paintMarkings || (style.oneWay && first)) return arm;
    if (style.oneWay)
    {
        const double half = style.GetLaneCount() * style.TravelLaneWidth() * 0.5;
        arm.stopLine = LaneSpan{-half, half};
        return arm;
    }

    const int lanes = style.GetLaneCount();
    const double halfLane = style.TravelLaneWidth() * 0.5;
    const double outer = style.HalfSeparation() + lanes * halfLane;
    double inner = style.HalfSeparation();
    if (lanes == 1) inner = -halfLane; // Shared, unstriped lane.
    else if (style.HasCenterTurnLane()) inner = halfLane;
    else if (lanes == 3) inner = road.ThreeLaneDividerOffset() * (first ? 1 : -1);
    arm.stopLine = LaneSpan{inner, outer};
    return arm;
}

RoadMouth IntersectionSurface::Mouth(int side) const
{
    const auto cut = Cut(layout, side);
    return {layout.center + cut.axis * Intersection::anchorDistance,
            layout.center + cut.axis * setbacks[side] + cut.lateral * lateralShifts[side], cut.axis, cut.scale};
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
    std::array<ApproachCut, sides> cuts;
    for (int i = 0; i < sides; ++i) cuts[i] = Cut(layout, i);

    // Each corner needs room for both mouth profiles at their actual lateral
    // offsets. At opposing 45-degree limits two incident edges can be parallel;
    // in that case keep the openings apart laterally on their respective sides.
    lateralShifts.fill(0);
    for (int i = 0; i < sides; ++i)
    {
        const int j = (i + 1) % sides;
        if (!layout.arms[i] || !layout.arms[j]) continue;
        const auto& a = *layout.arms[i];
        const auto& b = *layout.arms[j];
        const double edgeA = Profile(a.edge).Width(), edgeB = Profile(b.edge).Width();
        const double radius = OuterRadius(layout.radius, edgeA, edgeB);
        setbacks[i] = std::max(setbacks[i], (b.right + edgeB) * cuts[j].scale + radius);
        setbacks[j] = std::max(setbacks[j], (a.left + edgeA) * cuts[i].scale + radius);
    }
    // An oblique approach can run toward one adjacent mouth. Its lateral
    // position is bounded by the two corner tangencies at the chosen setback.
    for (int i = 0; i < sides; ++i)
    {
        if (!layout.arms[i]) continue;
        const auto& a = *layout.arms[i];
        const double edge = Profile(a.edge).Width();
        const double desired = (setbacks[i] - Intersection::anchorDistance) * cuts[i].drift;
        // The center must stay inside the mouth's projected pavement span.
        // This also prevents opposite corners crossing on a narrow branch.
        double lo = -a.left * cuts[i].scale, hi = a.right * cuts[i].scale;
        const int prev = (i + sides - 1) % sides, next = (i + 1) % sides;
        if (layout.arms[next])
            hi = std::min(hi, setbacks[next] - (a.left + edge) * cuts[i].scale);
        if (layout.arms[prev])
            lo = std::max(lo, (a.right + edge) * cuts[i].scale - setbacks[prev]);
        lateralShifts[i] = std::clamp(desired, lo, hi);
    }

    std::vector<Vector3> pavementRim;
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
        const Vector3 mouth = Mouth(i).center;
        const bool adjacent = j == (i + 1) % sides;
        const Vector3 nextMouth = Mouth(j).center;
        const double scaleA = cuts[i].scale, scaleB = cuts[j].scale;
        pavementRim.push_back(mouth);
        std::optional<Rim> previous;
        auto emit = [&](const Rim& rim, double t)
        {
            if (previous)
            {
                // Carry a strip through a taper to a bare edge. Otherwise each arm owns its half corner.
                const bool useA = a.edge != RoadEdge::None && (b.edge == RoadEdge::None || t <= .5);
                const auto& approach = useA ? a : b;
                Join(*this, *previous, rim, approach, directions);
            }
            pavementRim.push_back(rim.p[0]);
            geometry.outline.push_back(rim.p.back());
            previous = rim;
        };
        const auto add = [&](Vector3 p, Vector3 normal, double t)
        {
            emit(Section(p, normal, pa, pb, t), t);
        };
        add(mouth + n * (wa * scaleA), n * scaleA, 0);
        if (adjacent)
        {
            const Vector3 end = nextMouth + d * (wb * scaleB);
            const Vector3 corner = center + d * (wb * scaleB - lateralShifts[j]) +
                                            e * (wa * scaleA + lateralShifts[i]);
            const double r = std::max(0.0, std::min({OuterRadius(layout.radius, pa.Width(), pb.Width()),
                                      Math::Dot(mouth - corner, d), Math::Dot(nextMouth - corner, e)}));
            const Vector3 arcCenter = corner + (d + e) * r;
            // Offset each axis by its own edge dimensions. This gives concentric
            // arcs for matching curbs and tangent elliptical arcs for unlike edges.
            const auto cornerSection = [&](double sine, double cosine, double t)
            {
                const Vector3 outer = arcCenter - (d * sine + e * cosine) * r;
                Rim rim = Section(outer, {}, pa, pb, t);
                const Vector3 apron = outer - d * (sine * pb.cap * scaleB) - e * (cosine * pa.cap * scaleA);
                rim.p[0] = apron - d * (sine * pb.apron * scaleB) - e * (cosine * pa.apron * scaleA);
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
            add(end, d * scaleB, 1);
        }
        else
        {
            // Across the closed side of a T there is one continuous boundary.
            // Horizontal end tangents make asymmetric shoulders meet it smoothly.
            const double length = setbacks[i] + setbacks[j];
            const double start = lateralShifts[i] + wa * scaleA;
            const double finish = -lateralShifts[j] + wb * scaleB;
            const int intervals = start == finish && a.edge == b.edge ?
                (a.transitionMaterial == b.transitionMaterial ? 1 : 2) : cornerIntervals;
            for (int k = 1; k <= intervals; ++k)
            {
                const double t = static_cast<double>(k) / intervals, smooth = t * t * (3 - 2 * t);
                const double slope = (finish - start) * 6 * t * (1 - t) / length;
                const Vector3 normal = Math::Normalize(n + d * slope) * std::lerp(scaleA, scaleB, t);
                add(center + d * (setbacks[i] - length * t) + n * std::lerp(start, finish, smooth), normal, t);
            }
        }
    }

    FillPavement(*this, pavementRim, directions);
    for (int i = 0; i < sides; ++i)
    {
        const auto& arm = layout.arms[i];
        if (arm && arm->stopLine)
        {
            const auto mouth = Mouth(i);
            PaintStopLine(geometry, mouth.center, mouth.outward, cuts[i].lateral,
                          {arm->stopLine->inner * cuts[i].scale, arm->stopLine->outer * cuts[i].scale});
        }
    }
}
