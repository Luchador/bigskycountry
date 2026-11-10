#include "road.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace
{
    constexpr double stripeWidth = 0.16;
    constexpr double yellowGap = 0.16; // Clear space between the two yellow stripes.
    constexpr double yellowPairOffset = (yellowGap + stripeWidth) * 0.5;
    constexpr double stripeHeight = 0.02;
    constexpr double dashLength = 3.0;
    constexpr double gapLength = RoadStyle::dashPeriod - dashLength;

    struct Span
    {
        Vector3 start;
        Vector3 end;
        Vector3 left;
        double length;
    };

    constexpr Vector3 defaultLeft{0, 0, 1}; // A wholly vertical road has no horizontal heading.

    template<typename Visit>
    void VisitSpans(const std::vector<Vector3>& path, std::optional<Vector3> initialLeft, Visit visit)
    {
        // Use the preceding heading for vertical spans. If the whole path
        // begins vertically, look ahead for its first horizontal heading.
        Vector3 left = initialLeft.value_or(defaultLeft);
        if (!initialLeft)
        {
            for (std::size_t i = 1; i < path.size(); ++i)
            {
                const Vector3 delta = path[i] - path[i - 1];
                const double run = std::hypot(delta.x, delta.z);
                if (run > 0.0)
                {
                    left = {-delta.z / run, 0, delta.x / run};
                    break;
                }
            }
        }

        for (std::size_t i = 1; i < path.size(); ++i)
        {
            const Vector3 delta = path[i] - path[i - 1];
            const double run = std::hypot(delta.x, delta.z);
            if (run > 0.0)
            {
                left = {-delta.z / run, 0, delta.x / run};
            }
            else if (delta.y == 0.0)
            {
                continue;
            }
            visit(Span{path[i - 1], path[i], left, run});
        }
    }

    // A positive offset is left of the road in the direction its points were placed.
    template<typename Visit>
    void VisitOffsetStations(const std::vector<Span>& spans, double offset, Visit visit)
    {
        // On an inside bend the offset lines intersect before/after the corner.
        // Record that trim distance; a negative entry requests a pavement-style bevel.
        std::vector<double> trim(spans.size() + 1, 0.0);

        for (std::size_t i = 1; i < spans.size(); ++i)
        {
            const Vector3 a = spans[i - 1].left;
            const Vector3 b = spans[i].left;
            const double turn = a.x * b.z - a.z * b.x;
            const double divisor = 1.0 + Math::Dot(a, b);
            trim[i] = divisor > 0.0 ? offset * turn / divisor : -1.0;
        }

        visit(spans.front().start, spans.front().left);

        for (std::size_t i = 1; i < spans.size(); ++i)
        {
            const Span& a = spans[i - 1];
            const Span& b = spans[i];

            // Keep a miter only if the trims from both ends fit on each span.
            if (trim[i] >= 0.0 && trim[i] + std::max(0.0, trim[i - 1]) <= a.length && trim[i] + std::max(0.0, trim[i + 1]) <= b.length)
            {
                const double divisor = 1.0 + Math::Dot(a.left, b.left);
                visit(b.start, (a.left + b.left) * (1.0 / divisor));
            }
            else
            {
                visit(a.end, a.left);
                visit(b.start, b.left);
            }
        }

        visit(spans.back().end, spans.back().left);
    }

    std::vector<Vector3> OffsetPath(const std::vector<Span>& spans, double offset)
    {
        std::vector<Vector3> path;
        VisitOffsetStations(spans, offset, [&](Vector3 center, Vector3 normal)
        {
            path.push_back(center + normal * offset);
        });
        return path;
    }

    void AppendBoundary(Road& road, int side, const std::vector<Vector3>& boundary)
    {
        if (side < 0)
        {
            road.outline.insert(road.outline.end(), boundary.begin(), boundary.end());
        }
        else
        {
            road.outline.insert(road.outline.end(), boundary.rbegin(), boundary.rend());
        }
    }

    using EdgeProfile = std::array<Vector3, 4>;

    void AddEdgeFace(Road& road, int side, const EdgeProfile& previous, const EdgeProfile& current,
                     int face, SDL_FColor inner, SDL_FColor outer)
    {
        const Road::EdgeVertex a{previous[face], inner}, b{previous[face + 1], outer};
        const Road::EdgeVertex c{current[face + 1], outer}, d{current[face], inner};
        if (side > 0)
        {
            road.edgeTriangles.push_back({a, b, c});
            road.edgeTriangles.push_back({a, c, d});
        }
        else
        {
            road.edgeTriangles.push_back({a, c, b});
            road.edgeTriangles.push_back({a, d, c});
        }
    }

    void AddPavementBand(Road& road, int side, Vector3 previousInner, Vector3 inner,
                         Vector3 previousOuter, Vector3 outer)
    {
        if (previousInner != inner)
        {
            road.triangles.push_back(side > 0 ? Road::Triangle{previousOuter, inner, previousInner}
                                              : Road::Triangle{previousInner, inner, previousOuter});
        }
        road.triangles.push_back(side > 0 ? Road::Triangle{previousOuter, outer, inner}
                                          : Road::Triangle{previousOuter, inner, outer});
    }

    // Use the same cross sections for pavement, its outline and any edge treatment.
    // Adjacent spans then meet at the same elevation along the entire join.
    void BuildRoadSide(Road& road, const std::vector<Span>& spans, int side, bool hasGradedBend)
    {
        constexpr SDL_FColor red{1, 0, 0, 1}, black{0, 0, 0, 1};
        constexpr SDL_FColor apron{0.69f, 0.69f, 0.69f, 1}, riser{0.56f, 0.56f, 0.56f, 1}, top{0.78f, 0.78f, 0.78f, 1};

        const RoadEdge edge = road.style.edge;
        const int faces = edge == RoadEdge::Curb ? 3 : (edge == RoadEdge::Transition ? 1 : 0);
        const double pavement = side < 0 ? road.style.RightWidth() : road.style.LeftWidth();
        // On a graded bend, a wide pavement triangle can cut through a stripe.
        // Add longitudinal edges at lane boundaries, including the shoulder line.
        std::vector<double> laneFractions;
        if (hasGradedBend)
        {
            const double firstLane = road.style.GetLaneCount() % 2 ? RoadStyle::laneWidth * 0.5 : RoadStyle::laneWidth;
            for (double offset = firstLane; offset < pavement; offset += RoadStyle::laneWidth)
            {
                laneFractions.push_back(offset / pavement);
            }
        }
        Vector3 previousCenter{};
        EdgeProfile previous{};
        std::vector<Vector3> boundary;

        VisitOffsetStations(spans, side * (pavement + road.style.EdgeWidth()), [&](Vector3 center, Vector3 normal)
        {
            const Vector3 outward = normal * side;
            EdgeProfile profile{};
            profile[0] = center + outward * pavement;
            if (faces > 0)
            {
                profile[1] = profile[0] + outward * RoadStyle::stripWidth;
            }

            if (edge == RoadEdge::Curb)
            {
                profile[2] = profile[1] + Vector3{0, RoadStyle::curbHeight, 0};
                profile[3] = profile[2] + outward * RoadStyle::curbTopWidth;
            }

            if (boundary.empty())
            {
                // Follow the end profile up to the outer rim.
                boundary.insert(boundary.end(), profile.begin(), profile.begin() + faces);
            }
            else
            {
                Vector3 previousInner = previousCenter, inner = center;
                for (double fraction : laneFractions)
                {
                    const Vector3 previousOuter = previousCenter + (previous[0] - previousCenter) * fraction;
                    const Vector3 outer = center + (profile[0] - center) * fraction;
                    AddPavementBand(road, side, previousInner, inner, previousOuter, outer);
                    previousInner = previousOuter;
                    inner = outer;
                }
                AddPavementBand(road, side, previousInner, inner, previous[0], profile[0]);

                if (edge == RoadEdge::Transition)
                {
                    AddEdgeFace(road, side, previous, profile, 0, red, black);
                }
                else if (edge == RoadEdge::Curb)
                {
                    AddEdgeFace(road, side, previous, profile, 0, apron, apron);
                    AddEdgeFace(road, side, previous, profile, 1, riser, riser);
                    AddEdgeFace(road, side, previous, profile, 2, top, top);
                }
            }

            boundary.push_back(profile[faces]);
            previous = profile;
            previousCenter = center;
        });

        for (int i = faces - 1; i >= 0; --i)
        {
            boundary.push_back(previous[i]);
        }

        AppendBoundary(road, side, boundary);
    }

    // lo and hi are signed distances perpendicular to each span, in meters.
    void PaintStrip(const std::vector<Vector3>& path, double lo, double hi, std::vector<Road::Triangle>& mesh,
                    std::optional<Vector3> initialLeft = {})
    {
        Vector3 previousLo{}, previousHi{};
        bool havePrevious = false;

        VisitSpans(path, initialLeft, [&](const Span& span)
        {
            const Vector3 left = span.left;
            // A vertical stripe needs a horizontal depth offset from its pavement.
            const Vector3 up = span.length > 0.0 ? Vector3{0, 1, 0}
                : Math::Cross(left, {0, span.end.y > span.start.y ? 1.0 : -1.0, 0});
            const Vector3 a = span.start + up * stripeHeight;
            const Vector3 b = span.end + up * stripeHeight;
            const Vector3 aLo = a + left * lo, aHi = a + left * hi;
            const Vector3 bLo = b + left * lo, bHi = b + left * hi;

            if (havePrevious)
            {
                if (lo != 0.0)
                {
                    mesh.push_back({a, previousLo, aLo});
                }

                if (hi != 0.0)
                {
                    mesh.push_back({a, previousHi, aHi});
                }
            }

            mesh.push_back({aLo, aHi, bHi});
            mesh.push_back({aLo, bHi, bLo});
            previousLo = bLo;
            previousHi = bHi;
            havePrevious = true;
        });
    }

    void PaintDashes(const std::vector<Vector3>& path, std::vector<Road::Triangle>& mesh)
    {
        if (path.empty())
        {
            return;
        }

        // Carry the unfinished dash or gap into the next span of this divider.
        bool painting = true;
        double remaining = dashLength;
        std::vector<Vector3> dash{path.front()};

        Vector3 dashLeft = defaultLeft;
        VisitSpans(path, {}, [&](const Span& span)
        {
            const Vector3 start = span.start;
            const Vector3 delta = span.end - start;
            if (dash.size() == 1)
            {
                dashLeft = span.left;
            }
            const double length = Math::Length(delta);
            double used = 0.0;

            while (used < length)
            {
                const double available = length - used;

                if (remaining > available)
                {
                    if (painting)
                    {
                        dash.push_back(span.end);
                    }
                    remaining -= available;
                    break;
                }

                used += remaining;
                const Vector3 boundary = start + delta * (used / length);

                if (painting)
                {
                    dash.push_back(boundary);
                    PaintStrip(dash, -stripeWidth * 0.5, stripeWidth * 0.5, mesh, dashLeft);
                    dash.clear();
                }
                else
                {
                    dashLeft = span.left;
                    dash.push_back(boundary);
                }

                painting = !painting;
                remaining = painting ? dashLength : gapLength;
            }
        });
        PaintStrip(dash, -stripeWidth * 0.5, stripeWidth * 0.5, mesh, dashLeft);
    }

    void PaintDoubleYellow(const std::vector<Span>& spans, double boundary, std::vector<Road::Triangle>& mesh)
    {
        for (int side : {-1, 1})
        {
            const auto stripe = OffsetPath(spans, boundary + side * yellowPairOffset);
            PaintStrip(stripe, -stripeWidth * 0.5, stripeWidth * 0.5, mesh);
        }
    }
}

void Road::BuildSurface(const std::vector<Vector3>& samples)
{
    std::vector<Span> spans;
    VisitSpans(samples, {}, [&](const Span& span) { spans.push_back(span); });

    if (spans.empty())
    {
        return;
    }

    const int laneCount = style.GetLaneCount();
    const bool hasGradedBend = std::adjacent_find(spans.begin(), spans.end(), [](const Span& a, const Span& b)
    {
        return a.left != b.left && (a.start.y != a.end.y || b.start.y != b.end.y);
    }) != spans.end();
    const double halfTravelWidth = laneCount * RoadStyle::laneWidth * 0.5;

    for (int side : {-1, 1})
    {
        // Place the white stripe wholly inside the travel edge, beside any shoulder.
        const auto edge = OffsetPath(spans, side * halfTravelWidth);
        PaintStrip(edge, side > 0 ? -stripeWidth : 0.0, side > 0 ? 0.0 : stripeWidth, whiteStripes);

        BuildRoadSide(*this, spans, side, hasGradedBend);
    }

    constexpr double halfLane = RoadStyle::laneWidth * 0.5;
    const bool centerTurn = style.HasCenterTurnLane();

    if (centerTurn)
    {
        for (int side : {-1, 1})
        {
            const auto solid = OffsetPath(spans, side * (halfLane + yellowPairOffset));
            PaintStrip(solid, -stripeWidth * 0.5, stripeWidth * 0.5, yellowStripes);
            const auto dashed = OffsetPath(spans, side * (halfLane - yellowPairOffset));
            PaintDashes(dashed, yellowStripes);
        }
    }
    else if (laneCount == 3)
    {
        // North is +Z. On an east-west road, use +X as the northbound direction.
        const Vector3 travel = spans.back().end - spans.front().start;
        const bool northFollowsPoints = travel.z == 0.0 ? travel.x >= 0.0 : travel.z > 0.0;
        const bool twoNorthbound = style.threeLaneLayout == ThreeLaneLayout::TwoNorthbound;
        // Traffic keeps right. Positive offsets are left of the authored path.
        const double boundary = (twoNorthbound == northFollowsPoints) ? halfLane : -halfLane;
        PaintDoubleYellow(spans, boundary, yellowStripes);
        PaintDashes(OffsetPath(spans, -boundary), whiteStripes);
        return;
    }
    else
    {
        if(laneCount != 1)
        {
            PaintDoubleYellow(spans, 0.0, yellowStripes);
        }
    }

    // Start counting through lanes at the edge of the center turn lane, if present.
    const double throughLaneStart = centerTurn ? halfLane : 0.0;
    const int lanesPerDirection = laneCount / 2;

    for (int side : {-1, 1})
    {
        for (int lane = 1; lane < lanesPerDirection; ++lane)
        {
            const auto divider = OffsetPath(spans, side * (throughLaneStart + lane * RoadStyle::laneWidth));
            PaintDashes(divider, whiteStripes);
        }
    }
}
