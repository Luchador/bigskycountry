#pragma once

#include "road.hpp"
#include <algorithm>
#include <utility>

namespace RoadSpline
{
    struct Cubic
    {
        Vector3 a, b, c, d;
    };

    inline Cubic SegmentCurve(const RoadPoint& start, const RoadPoint& end, Vector3 startTangent, Vector3 endTangent)
    {
        const double chord = Math::Length(end.position - start.position);
        return {start.position, start.position + startTangent * (chord * start.outgoingHandle),
                end.position - endTangent * (chord * end.incomingHandle), end.position};
    }

    // De Casteljau subdivision gives the control polygons of both exact child curves.
    inline std::pair<Cubic, Cubic> Split(const Cubic& curve, double t)
    {
        const auto lerp = [t](Vector3 a, Vector3 b) { return a + (b - a) * t; };
        const auto [a, b, c, d] = curve;
        const Vector3 ab = lerp(a, b), bc = lerp(b, c), cd = lerp(c, d);
        const Vector3 abc = lerp(ab, bc), bcd = lerp(bc, cd);
        const Vector3 middle = lerp(abc, bcd);
        return {{a, ab, abc, middle}, {middle, bcd, cd, d}};
    }

    struct EndSampling
    {
        double start = 0.0, end = 0.0; // Meters of closely spaced sections needed at either cap.
        double spacing = 1.0;
    };

    template<typename Visit>
    void SampleCubic(const Cubic& curve, double tolerance, int depth, double t0, double t1, const Visit& visit,
                     EndSampling ends = {})
    {
        const auto& [a, b, c, d] = curve;
        const Vector3 chord = d - a;
        const double error = std::max(Math::Length((b - a) - chord * (1.0 / 3.0)), Math::Length((c - a) - chord * (2.0 / 3.0)));

        const bool refineEnd = (ends.start > 0 || ends.end > 0) && Math::Dot(chord, chord) > ends.spacing * ends.spacing;
        if ((error <= tolerance && !refineEnd) || depth == 12)
        {
            // The derivative at this subcurve's endpoint is parallel to its last control-polygon edge.
            visit(d, d - c, t1);
            return;
        }

        const auto [left, right] = Split(curve, 0.5);
        const double middle = (t0 + t1) * 0.5;
        EndSampling first = ends, last = ends;
        if (ends.start > 0) last.start -= Math::Length(left.d - left.a);
        if (ends.end > 0) first.end -= Math::Length(right.d - right.a);
        SampleCubic(left, tolerance, depth + 1, t0, middle, visit, first);
        SampleCubic(right, tolerance, depth + 1, middle, t1, visit, last);
    }
}
