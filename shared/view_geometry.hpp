#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace ViewGeometry
{
    template<typename Vertex>
    struct VertexPolygon
    {
        // A triangle clipped against five planes or a quad against four needs at most eight vertices.
        std::array<Vertex, 8> points;
        int count = 0;
    };

    using Polygon = VertexPolygon<Vector3>;

    // Keep the portion on or in front of the clipping plane.
    template<typename Vertex, typename Distance, typename Mix>
    VertexPolygon<Vertex> ClipPolygon(const VertexPolygon<Vertex>& input, Distance distanceToPlane, Mix mix)
    {
        VertexPolygon<Vertex> result{};
        Vertex previous = input.points[input.count - 1];

        double previousDistance = distanceToPlane(previous);

        for (int i = 0; i < input.count; ++i)
        {
            const Vertex current = input.points[i];
            const double distance = distanceToPlane(current);

            if ((distance < 0.0 && previousDistance > 0.0) || (distance > 0.0 && previousDistance < 0.0))
            {
                const double t = previousDistance / (previousDistance - distance);
                result.points[result.count++] = mix(previous, current, t);
            }

            if (distance >= 0.0)
            {
                result.points[result.count++] = current;
            }

            previous = current;
            previousDistance = distance;
        }

        return result;
    }

    inline Polygon Clip(const Polygon& input, Vector3 normal, double offset)
    {
        return ClipPolygon(input, [&](Vector3 p) { return Math::Dot(p, normal) - offset; },
                           [](Vector3 a, Vector3 b, double t) { return a + (b - a) * t; });
    }

    inline std::array<Vector3, 4> SidePlanes(double focal, SDL_Rect viewport)
    {
        const double w = viewport.w * 0.5, h = viewport.h * 0.5;
        return {{{focal, 0, w}, {-focal, 0, w}, {0, focal, h}, {0, -focal, h}}};
    }

    inline SDL_FPoint ScreenPoint(Vector3 v, double focal, SDL_Rect viewport)
    {
        return {static_cast<float>(viewport.x + viewport.w * 0.5 + v.x * focal / v.z),
                static_cast<float>(viewport.y + viewport.h * 0.5 - v.y * focal / v.z)};
    }

    inline bool ClipLine(Vector3& a, Vector3& b, Vector3 normal, double offset)
    {
        const double da = Math::Dot(a, normal) - offset;
        const double db = Math::Dot(b, normal) - offset;
        if (da < 0.0 && db < 0.0)
        {
            return false;
        }
        if ((da < 0.0) != (db < 0.0))
        {
            const Vector3 cut = a + (b - a) * (da / (da - db));
            if (da < 0.0)
            {
                a = cut;
            }
            else
            {
                b = cut;
            }
        }
        return true;
    }

    struct ProjectedStroke
    {
        std::array<SDL_FPoint, 4> corners;
        double startDepth, endDepth;
    };

    inline std::optional<ProjectedStroke> ProjectStroke(Vector3 a, Vector3 b, const std::array<Vector3, 4>& sides, double focal, SDL_Rect viewport, double nearDepth, float halfWidth)
    {
        if (!ClipLine(a, b, {0, 0, 1}, nearDepth) || !std::all_of(sides.begin(), sides.end(), [&](Vector3 side) { return ClipLine(a, b, side, 0.0); }))
        {
            return {};
        }

        const SDL_FPoint p = ScreenPoint(a, focal, viewport), q = ScreenPoint(b, focal, viewport);
        const float dx = q.x - p.x, dy = q.y - p.y;
        const float length = std::hypot(dx, dy);

        if (length == 0.0f)
        {
            return {};
        }

        // Square caps overlap at the joins. The width stays constant on screen.
        const float tx = halfWidth * dx / length, ty = halfWidth * dy / length;
        ProjectedStroke stroke;
        stroke.corners = {{{p.x - tx - ty, p.y - ty + tx}, {p.x - tx + ty, p.y - ty - tx}, {q.x + tx + ty, q.y + ty - tx}, {q.x + tx - ty, q.y + ty + tx}}};
        stroke.startDepth = a.z;
        stroke.endDepth = b.z;
        return stroke;
    }
}
