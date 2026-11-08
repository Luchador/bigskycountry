#include "road.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double stripeWidth = 0.16;
    constexpr double yellowGap = 0.16; // Clear space between the two yellow stripes.
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
    std::vector<Vector3> OffsetPath(const std::vector<Span>& spans, double offset)
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

        std::vector<Vector3> path{spans.front().start + spans.front().left * offset};

        for (std::size_t i = 1; i < spans.size(); ++i)
        {
            const Span& a = spans[i - 1];
            const Span& b = spans[i];

            // Keep a miter only if the trims from both ends fit on each span.
            if (trim[i] >= 0.0 && trim[i] + std::max(0.0, trim[i - 1]) <= a.length && trim[i] + std::max(0.0, trim[i + 1]) <= b.length)
            {
                const double divisor = 1.0 + Math::Dot(a.left, b.left);
                path.push_back(b.start + (a.left + b.left) * (offset / divisor));
            }
            else
            {
                path.push_back(a.end + a.left * offset);
                path.push_back(b.start + b.left * offset);
            }
        }

        path.push_back(spans.back().end + spans.back().left * offset);

        return path;
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
}

void Road::BuildStripes(const std::vector<Vector3>& samples)
{
    const auto spans = MakeSpans(samples);

    if (spans.empty())
    {
        return;
    }

    const double halfTravelWidth = style.lanes * RoadStyle::laneWidth * 0.5;
    const int lanesPerDirection = style.lanes / 2;

    for (int side : {-1, 1})
    {
        // Place the white stripe wholly inside the travel edge, beside any shoulder.
        const auto edge = OffsetPath(spans, side * halfTravelWidth);
        PaintStrip(edge, side > 0 ? -stripeWidth : 0.0, side > 0 ? 0.0 : stripeWidth, whiteStripes);

        const auto yellow = OffsetPath(spans, side * (yellowGap + stripeWidth) * 0.5);
        PaintStrip(yellow, -stripeWidth * 0.5, stripeWidth * 0.5, yellowStripes);

        for (int lane = 1; lane < lanesPerDirection; ++lane)
        {
            const auto divider = OffsetPath(spans, side * lane * RoadStyle::laneWidth);
            PaintDashes(divider, whiteStripes);
        }
    }
}
