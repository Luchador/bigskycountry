#include "road.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double stripeWidth = 0.16;
    constexpr double yellowGap = 0.16; // Clear space between the two yellow stripes.
    constexpr double yellowPairOffset = (yellowGap + stripeWidth) * 0.5;
    constexpr double stripeHeight = 0.02;
    constexpr double dashLength = 3.0;
    constexpr double gapLength = 9.0;

    struct Span
    {
        Vector3 start;
        Vector3 end;
        Vector3 left;
        double length;
    };

    std::vector<Span> MakeSpans(const std::vector<Vector3>& samples)
    {
        std::vector<Span> spans;

        for (std::size_t i = 1; i < samples.size(); ++i)
        {
            const Vector3 delta = samples[i] - samples[i - 1];
            const double length = std::hypot(delta.x, delta.z);

            if (length > 0.0)
            {
                spans.push_back({samples[i - 1], samples[i], {-delta.z / length, 0.0, delta.x / length}, length});
            }
        }

        return spans;
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

    void BuildPlainPavement(Road& road, const std::vector<Span>& spans)
    {
        Vector3 previousLeft{}, previousRight{};
        bool havePrevious = false;
        const double leftWidth = road.style.LeftWidth();
        const double rightWidth = road.style.RightWidth();

        for (const Span& span : spans)
        {
            const Vector3 a = span.start, b = span.end, left = span.left;
            const Vector3 aLeft = a + left * leftWidth, aRight = a - left * rightWidth;
            const Vector3 bLeft = b + left * leftWidth, bRight = b - left * rightWidth;

            if (havePrevious)
            {
                // Bevel the join without stretching the road at sharp turns.
                road.triangles.push_back({a, previousLeft, aLeft});
                road.triangles.push_back({a, previousRight, aRight});
            }

            // Both sides share an explicit center vertex with each bevel. This keeps
            // the joins watertight even when the renderer rounds to pixel centers.
            road.triangles.push_back({aLeft, a, b});
            road.triangles.push_back({aLeft, b, bLeft});
            road.triangles.push_back({a, aRight, bRight});
            road.triangles.push_back({a, bRight, b});
            previousLeft = bLeft;
            previousRight = bRight;
            havePrevious = true;
        }
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

    // Sweep half the pavement and its edge along the same stations. A bevel adds
    // a second station at the corner, so all three curb faces turn together.
    void BuildRoadSide(Road& road, const std::vector<Span>& spans, int side)
    {
        constexpr SDL_FColor red{1, 0, 0, 1}, black{0, 0, 0, 1};
        constexpr SDL_FColor apron{0.69f, 0.69f, 0.69f, 1}, riser{0.56f, 0.56f, 0.56f, 1}, top{0.78f, 0.78f, 0.78f, 1};

        const RoadEdge edge = road.style.edge;
        const int faces = edge == RoadEdge::Curb ? 3 : 1;
        const double pavement = side < 0 ? road.style.RightWidth() : road.style.LeftWidth();
        Vector3 previousCenter{};
        EdgeProfile previous{};
        std::vector<Vector3> boundary;

        VisitOffsetStations(spans, side * (pavement + road.style.EdgeWidth()), [&](Vector3 center, Vector3 normal)
        {
            const Vector3 outward = normal * side;
            EdgeProfile profile{};
            profile[0] = center + outward * pavement;
            profile[1] = profile[0] + outward * RoadStyle::stripWidth;

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
                if (previousCenter.x != center.x || previousCenter.z != center.z)
                {
                    road.triangles.push_back(side > 0 ? Road::Triangle{previous[0], previousCenter, center} : Road::Triangle{previousCenter, previous[0], center});
                }

                road.triangles.push_back(side > 0 ? Road::Triangle{previous[0], center, profile[0]} : Road::Triangle{previous[0], profile[0], center});

                if (edge == RoadEdge::Transition)
                {
                    AddEdgeFace(road, side, previous, profile, 0, red, black);
                }
                else
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
    void PaintStrip(const std::vector<Vector3>& path, double lo, double hi, std::vector<Road::Triangle>& mesh)
    {
        const Vector3 lift{0.0, stripeHeight, 0.0};
        Vector3 previousLo{}, previousHi{};
        bool havePrevious = false;

        for (std::size_t i = 1; i < path.size(); ++i)
        {
            const Vector3 a = path[i - 1] + lift;
            const Vector3 b = path[i] + lift;
            const Vector3 delta = b - a;
            const double length = std::hypot(delta.x, delta.z);

            if (length == 0.0)
            {
                continue;
            }

            const Vector3 left{-delta.z / length, 0.0, delta.x / length};
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
        }
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

        for (std::size_t i = 1; i < path.size(); ++i)
        {
            const Vector3 start = path[i - 1];
            const Vector3 delta = path[i] - start;
            const double length = Math::Length(delta);
            double used = 0.0;

            while (used < length)
            {
                const double available = length - used;

                if (remaining > available)
                {
                    if (painting)
                    {
                        dash.push_back(path[i]);
                    }
                    remaining -= available;
                    break;
                }

                used += remaining;
                const Vector3 boundary = start + delta * (used / length);

                if (painting)
                {
                    dash.push_back(boundary);
                    PaintStrip(dash, -stripeWidth * 0.5, stripeWidth * 0.5, mesh);
                    dash.clear();
                }
                else
                {
                    dash.push_back(boundary);
                }

                painting = !painting;
                remaining = painting ? dashLength : gapLength;
            }
        }
        PaintStrip(dash, -stripeWidth * 0.5, stripeWidth * 0.5, mesh);
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
    const auto spans = MakeSpans(samples);

    if (spans.empty())
    {
        return;
    }

    const int laneCount = style.GetLaneCount();
    const double halfTravelWidth = laneCount * RoadStyle::laneWidth * 0.5;

    if (style.edge == RoadEdge::None)
    {
        BuildPlainPavement(*this, spans);
    }

    for (int side : {-1, 1})
    {
        // Place the white stripe wholly inside the travel edge, beside any shoulder.
        auto edge = OffsetPath(spans, side * halfTravelWidth);
        PaintStrip(edge, side > 0 ? -stripeWidth : 0.0, side > 0 ? 0.0 : stripeWidth, whiteStripes);

        if (style.edge == RoadEdge::None)
        {
            const double width = side < 0 ? style.RightWidth() : style.LeftWidth();
            if (width != halfTravelWidth)
            {
                edge = OffsetPath(spans, side * width);
            }
            AppendBoundary(*this, side, edge);
        }
        else
        {
            BuildRoadSide(*this, spans, side);
        }
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
