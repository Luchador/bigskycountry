#include "intersection.hpp"
#include "view_geometry.hpp"

#include <CDT.h>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

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

    void ContinuePaint(IntersectionSurface& surface)
    {
        const auto& layout = surface.layout;
        if (layout.paint == IntersectionPaint::Standard) return;
        const int a = layout.paint == IntersectionPaint::NorthSouth ? 1 : 0;
        const int b = a + 2;
        if (!layout.arms[a] || !layout.arms[b]) return;
        RoadGeometry paint;
        BuildConnectingRoadPaint(paint, surface.Mouth(a), surface.Mouth(b),
                                 layout.arms[a]->paint, layout.arms[b]->paint);
        // Skewed/asymmetric junctions may have a concave pavement outline.
        // Clip to its triangulation so paint never floats across a corner or curb.
        const auto append = [&](const auto& source, auto& destination)
        {
            for (const auto& stripe : source)
            {
                for (const auto& pavement : surface.geometry.pavementTriangles)
                {
                    ViewGeometry::Polygon polygon{{stripe[0], stripe[1], stripe[2]}, 3};
                    for (int i = 0; i < 3 && polygon.count >= 3; ++i)
                    {
                        const Vector3 start = pavement[i].position;
                        const Vector3 normal = Math::Cross(Vector3{0, 1, 0}, pavement[(i + 1) % 3].position - start);
                        polygon = ViewGeometry::Clip(polygon, normal, Math::Dot(start, normal));
                    }
                    for (int i = 2; i < polygon.count; ++i)
                    {
                        const auto a = polygon.points[0], b = polygon.points[i - 1], c = polygon.points[i];
                        if (Math::Cross(b - a, c - a).y > 1e-10) destination.push_back({a, b, c});
                    }
                }
            }
        };
        append(paint.whiteStripes, surface.geometry.whiteStripes);
        append(paint.yellowStripes, surface.geometry.yellowStripes);
    }

    // The skewed mouths can put the junction center outside a corner's
    // visibility region. Triangulate the entire inner curb boundary in XZ.
    void FillPavement(IntersectionSurface& surface, const std::vector<Vector3>& rim,
                      const std::array<Vector3, sides>& axes, const std::vector<Vector3>& hole = {})
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
        if (positions.size() < 3 || !std::in_range<CDT::VertInd>(positions.size())) return;
        const auto vertexCount = static_cast<CDT::VertInd>(positions.size());
        for (CDT::VertInd i = 0; i < vertexCount; ++i)
        {
            const auto local = positions[i] - surface.layout.center;
            vertices.push_back({local.x, local.z});
            edges.emplace_back(i, (i + 1) % vertexCount);
        }
        const auto offset = static_cast<CDT::VertInd>(vertices.size());
        const auto holeCount = static_cast<CDT::VertInd>(hole.size());
        for (CDT::VertInd i = 0; i < holeCount; ++i)
        {
            const auto local = hole[i] - surface.layout.center;
            vertices.push_back({local.x, local.z});
            edges.emplace_back(offset + i, offset + (i + 1) % holeCount);
        }
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

    struct ApproachFrame
    {
        Vector3 anchor, forward, left;
    };

    ApproachFrame Frame(const IntersectionLayout& layout, int side)
    {
        const Vector3 nominal = Intersection::Directions(layout.yaw, layout.shape)[side];
        const Vector3 forward = Intersection::Directions(layout.yaw + layout.skew[side], layout.shape)[side];
        return {layout.center + nominal * layout.AnchorDistance(), forward, Math::Cross(forward, up)};
    }

    struct Corner
    {
        Vector3 start, end, center;
        double radius, turn;
    };

    std::optional<Corner> Fillet(const ApproachFrame& a, const ApproachFrame& b,
                                 double widthA, double widthB, double radius, double angle)
    {
        const Vector3 start = a.anchor + a.left * widthA, end = b.anchor - b.left * widthB;
        if (angle < 1e-6)
        {
            // Two parallel approaches can share a rounded nose. Its radius is
            // limited by the actual space between their facing edges.
            const double advance = Math::Dot(end - start, a.forward);
            const Vector3 p = start + a.forward * std::max(0.0, advance);
            const Vector3 q = end - b.forward * std::min(0.0, advance);
            const double gap = Math::Dot(q - p, a.left);
            if (gap <= 1e-6) return {};
            return Corner{p, q, (p + q) * 0.5, gap * 0.5, std::numbers::pi};
        }
        if (angle >= std::numbers::pi - 1e-6) return {};
        const double sine = -Math::Cross(a.forward, b.forward).y;
        const double alongA = -Math::Cross(end - start, b.forward).y / sine;
        const double alongB = -Math::Cross(end - start, a.forward).y / sine;
        const double cotangent = 1.0 / std::tan(angle * 0.5);
        // Near-parallel roads must not send their mouths arbitrarily far down
        // the spline. Tighten the corner to the available approach space.
        const double reach = std::max(Intersection::anchorDistance, 2 * (std::max(widthA, widthB) + radius));
        const double furthest = std::max(alongA, alongB);
        if (furthest > reach) return {};
        radius = std::min(radius, std::max(0.0, (reach - furthest) / cotangent));
        const Vector3 p = start + a.forward * (alongA + radius * cotangent);
        const Vector3 q = end + b.forward * (alongB + radius * cotangent);
        return Corner{p, q, p + a.left * radius, radius, std::numbers::pi - angle};
    }

    int NextArm(const IntersectionLayout& layout, int side)
    {
        do { side = (side + 1) % sides; } while (!layout.arms[side]);
        return side;
    }

    // Fit a triangular island inside the junction, reserving a full approach
    // width around it. Test finite boundary segments: extending the tangent
    // lines of a concave Y would incorrectly eliminate the center of the island.
    std::vector<Vector3> BuildIsland(IntersectionSurface& surface, const std::vector<Vector3>& pavement,
                                     const std::array<Vector3, sides>& axes)
    {
        const auto& layout = surface.layout;
        if (layout.shape != IntersectionShape::Y || !layout.island || pavement.size() < 3) return {};
        double clearance = RoadStyle::laneWidth;
        const IntersectionApproach* approach = nullptr;
        for (const auto& arm : layout.arms)
        {
            if (!arm) continue;
            clearance = std::max(clearance, arm->left + arm->right);
            if (!approach || arm->edge == RoadEdge::Curb || (approach->edge == RoadEdge::None && arm->edge != RoadEdge::None))
                approach = &*arm;
        }
        if (!approach) return {};
        const auto profile = Profile(approach->edge);
        const double distance = clearance + profile.Width();
        const Vector3 center = layout.center;
        std::vector<Vector3> boundary;
        for (auto p : pavement) boundary.push_back(p - center);
        const auto inside = [&](Vector3 p)
        {
            bool result = false;
            auto a = boundary.back();
            for (auto b : boundary)
            {
                if ((a.z > p.z) != (b.z > p.z) && p.x < (b.x - a.x) * (p.z - a.z) / (b.z - a.z) + a.x) result = !result;
                a = b;
            }
            return result;
        };
        const auto pointDistance = [](Vector3 p, Vector3 a, Vector3 b)
        {
            const auto d = b - a;
            const double length = Math::Dot(d, d);
            return Math::Length(p - (a + d * (length > 1e-16 ? std::clamp(Math::Dot(p - a, d) / length, 0.0, 1.0) : 0)));
        };
        const auto segmentDistance = [&](Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            const auto turn = [](Vector3 p, Vector3 q, Vector3 r) { return Math::Cross(q - p, r - p).y; };
            if (turn(a,b,c) * turn(a,b,d) < 0 && turn(c,d,a) * turn(c,d,b) < 0) return 0.0;
            return std::min({pointDistance(a,c,d), pointDistance(b,c,d), pointDistance(c,a,b), pointDistance(d,a,b)});
        };
        const auto nominal = Intersection::Directions(layout.yaw, layout.shape);
        std::vector<Vector3> polygon{nominal[0] * layout.size, nominal[2] * layout.size, nominal[3] * layout.size};
        const auto fits = [&](double scale)
        {
            for (std::size_t i = 0; i < polygon.size(); ++i)
            {
                const auto a = polygon[i] * scale, b = polygon[(i + 1) % polygon.size()] * scale;
                if (!inside(a)) return false;
                for (std::size_t j = 0; j < boundary.size(); ++j)
                    if (segmentDistance(a,b,boundary[j],boundary[(j + 1) % boundary.size()]) < distance) return false;
            }
            return true;
        };
        if (!fits(0)) return {};
        double low = 0, high = 1;
        for (int i = 0; i < 32; ++i)
        {
            const double middle = (low + high) * .5;
            if (fits(middle)) low = middle;
            else high = middle;
        }
        if (low * layout.size < 1.0) return {};
        for (auto& point : polygon) point = point * low;

        // Round the convex land boundary, then reverse it to keep pavement on
        // the left, matching the winding of the outer curb/transition strips.
        std::vector<Vector3> rounded;
        for (std::size_t i = 0; i < polygon.size(); ++i)
        {
            const auto a = polygon[(i + polygon.size() - 1) % polygon.size()], b = polygon[i], c = polygon[(i + 1) % polygon.size()];
            const double ab = Math::Length(a - b), cb = Math::Length(c - b);
            if (ab < 1e-6 || cb < 1e-6) continue;
            const double cut = std::min({layout.radius, ab * .25, cb * .25});
            const auto start = b + (a - b) * (cut / ab), end = b + (c - b) * (cut / cb);
            const int intervals = cut > 1e-4 ? 6 : 1;
            for (int k = 0; k <= intervals; ++k)
            {
                const double t = double(k) / intervals, u = 1 - t;
                const auto point = center + start * (u * u) + b * (2 * u * t) + end * (t * t);
                if (rounded.empty() || Math::Length(point - rounded.back()) > 1e-7) rounded.push_back(point);
            }
        }
        if (rounded.size() < 3) return {};
        std::reverse(rounded.begin(), rounded.end());
        std::vector<Rim> rims;
        std::vector<Vector3> hole, land;
        for (std::size_t i = 0; i < rounded.size(); ++i)
        {
            const auto previous = rounded[(i + rounded.size() - 1) % rounded.size()], next = rounded[(i + 1) % rounded.size()];
            const auto normal = Math::Normalize(Math::Cross(up, next - previous));
            rims.push_back(Section(rounded[i], normal, profile, profile, 0));
            hole.push_back(rims.back().p[0]);
            land.push_back(rims.back().p.back());
        }
        for (std::size_t i = 0; i < rims.size(); ++i)
            Join(surface, rims[i], rims[(i + 1) % rims.size()], *approach, axes);
        surface.geometry.holes.push_back(std::move(land));
        return hole;
    }
}

std::array<Vector3, IntersectionAnchor::sideCount> Intersection::Directions(double yaw, IntersectionShape shape)
{
    if (shape == IntersectionShape::Y)
    {
        auto axes = Directions(yaw);
        axes[0] = Directions(yaw - std::numbers::pi / 6)[0];
        axes[2] = Directions(yaw + std::numbers::pi / 6)[2];
        axes[1] = {}; // The fourth socket is deliberately unavailable.
        return axes;
    }
    if (yaw == 0) return IntersectionAnchor::directions;
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {{{c, 0, -s}, {s, 0, c}, {-c, 0, s}, {-s, 0, -c}}};
}

std::array<Vector3, IntersectionAnchor::sideCount> Intersection::AnchorPositions() const
{
    auto anchors = Directions(yaw, shape);
    for (auto& anchor : anchors) anchor = position + anchor * AnchorDistance();
    return anchors;
}

IntersectionApproach IntersectionApproach::FromRoad(const Road& road, bool first, bool paintStopLine)
{
    const auto style = road.EndpointStyle(first);
    IntersectionApproach arm{first ? style.LeftWidth() : style.RightWidth(),
                             first ? style.RightWidth() : style.LeftWidth(), style.edge, {},
                             style.edge == RoadEdge::Transition ? style.transitionMaterial : std::string{}};
    arm.paint = {style.GetLaneCount(), style.TravelLaneWidth(), style.HalfSeparation(),
                 style.GetLaneCount() == 3 && !style.HasCenterTurnLane() ? road.ThreeLaneDividerOffset() * (first ? 1 : -1) : 0,
                 style.HasCenterTurnLane(), style.oneWay ? (first ? 1 : -1) : 0, road.paintMarkings};
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
    const auto frame = Frame(layout, side);
    return {frame.anchor, frame.anchor + frame.forward * (setbacks[side] - layout.AnchorDistance()), frame.forward};
}

void IntersectionSurface::Build(const IntersectionLayout& input)
{
    layout = input;
    const auto directions = Intersection::Directions(layout.yaw);
    geometry = {};
    geometry.MarkRebuilt();
    setbacks.fill(layout.AnchorDistance());
    if (layout.shape == IntersectionShape::Y) layout.arms[1].reset();
    std::array<ApproachFrame, sides> frames;
    std::array<std::optional<Corner>, sides> corners;
    for (int i = 0; i < sides; ++i) frames[i] = Frame(layout, i);

    // Intersect the true-width edge lines, then find the two tangencies of
    // their circular fillet. Each mouth clears both of its adjoining corners.
    // Authoring anchors stay fixed; rendered cuts are normal to the approach.
    for (int i = 0; i < sides; ++i)
    {
        if (!layout.arms[i] || layout.shape == IntersectionShape::Y) continue;
        const int j = NextArm(layout, i);
        if (j == i) continue;
        const auto& a = *layout.arms[i];
        const auto& b = *layout.arms[j];
        const double edgeA = Profile(a.edge).Width(), edgeB = Profile(b.edge).Width();
        const double angle = ((j - i + sides) % sides) * std::numbers::pi / 2 + layout.skew[i] - layout.skew[j];
        corners[i] = Fillet(frames[i], frames[j], a.left + edgeA, b.right + edgeB,
                            OuterRadius(layout.radius, edgeA, edgeB), angle);
        if (const auto& corner = corners[i])
        {
            setbacks[i] = std::max(setbacks[i], Intersection::anchorDistance + Math::Dot(corner->start - frames[i].anchor, frames[i].forward));
            setbacks[j] = std::max(setbacks[j], Intersection::anchorDistance + Math::Dot(corner->end - frames[j].anchor, frames[j].forward));
        }
    }

    std::vector<Vector3> pavementRim;
    for (int i = 0; i < sides; ++i)
    {
        if (!layout.arms[i]) continue;
        const int j = NextArm(layout, i);
        const auto& a = *layout.arms[i];
        const auto& b = *layout.arms[j];
        const auto& frameA = frames[i];
        const auto& frameB = frames[j];
        const EdgeProfile pa = Profile(a.edge), pb = Profile(b.edge);
        const double wa = a.left + pa.Width(), wb = b.right + pb.Width();
        const Vector3 mouth = Mouth(i).center;
        const Vector3 nextMouth = Mouth(j).center;
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
        const Vector3 start = mouth + frameA.left * wa;
        const Vector3 end = nextMouth - frameB.left * wb;
        add(start, frameA.left, 0);
        if (const auto& corner = corners[i])
        {
            const int intervals = std::max(1, static_cast<int>(std::ceil(cornerIntervals * corner->turn / (std::numbers::pi / 2))));
            for (int k = 0; k <= intervals; ++k)
            {
                const double t = static_cast<double>(k) / intervals;
                const double angle = corner->turn * t;
                const Vector3 normal = k == 0 ? frameA.left : k == intervals ? frameB.left * -1 :
                    frameA.left * std::cos(angle) + frameA.forward * std::sin(angle);
                const Vector3 outer = k == 0 ? corner->start : k == intervals ? corner->end :
                    corner->center - normal * corner->radius;
                // Ease unlike edge profiles so their pavement and curb edges
                // also meet the straight road tangentially.
                add(outer, normal, t * t * (3 - 2 * t));
            }
            add(end, frameB.left * -1, 1);
        }
        else
        {
            // Close the back of a T (or a parallel/overlapping pair) with a
            // tangent cubic. Equal, opposite approaches reduce to a straight edge.
            const double handle = Math::Length(end - start) / 3;
            const Vector3 p = start - frameA.forward * handle, q = end - frameB.forward * handle;
            const bool straight = Math::Dot(frameA.forward, frameB.forward) < -1 + 1e-12 &&
                                  std::abs(Math::Dot(end - start, frameA.left)) < 1e-8;
            const int curveIntervals = layout.shape == IntersectionShape::Y ?
                std::clamp(static_cast<int>(std::ceil(Math::Length(end - start) / 3)), cornerIntervals, 256) : cornerIntervals;
            const int intervals = straight && a.edge == b.edge ?
                (a.transitionMaterial == b.transitionMaterial ? 1 : 2) : curveIntervals;
            for (int k = 1; k <= intervals; ++k)
            {
                const double t = static_cast<double>(k) / intervals, u = 1 - t;
                const Vector3 point = start * (u * u * u) + p * (3 * u * u * t) + q * (3 * u * t * t) + end * (t * t * t);
                const Vector3 tangent = (p - start) * (u * u) + (q - p) * (2 * u * t) + (end - q) * (t * t);
                const Vector3 normal = k == intervals ? frameB.left * -1 :
                    Math::Length(tangent) > 1e-8 ? Math::Normalize(Math::Cross(up, tangent)) : frameA.left;
                add(k == intervals ? end : point, normal, t * t * (3 - 2 * t));
            }
        }
    }

    const auto hole = BuildIsland(*this, pavementRim, directions);
    FillPavement(*this, pavementRim, directions, hole);
    ContinuePaint(*this);
    for (int i = 0; i < sides; ++i)
    {
        const auto& arm = layout.arms[i];
        if (arm && arm->stopLine)
        {
            const auto mouth = Mouth(i);
            PaintStopLine(geometry, mouth.center, mouth.outward, frames[i].left, *arm->stopLine);
        }
    }
    geometry.UpdateBounds();
}
