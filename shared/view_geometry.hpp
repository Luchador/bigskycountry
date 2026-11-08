#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>
#include <array>

namespace ViewGeometry
{
    struct Polygon
    {
        // A triangle clipped against five planes or a quad against four needs at most eight vertices.
        std::array<Vector3, 8> points;
        int count = 0;
    };

    // Retain the half-space dot(point, normal) >= offset.
    inline Polygon Clip(const Polygon& input, Vector3 normal, double offset)
    {
        Polygon result{};
        Vector3 previous = input.points[input.count - 1];

        double previousDistance = Math::Dot(previous, normal) - offset;

        for (int i = 0; i < input.count; ++i)
        {
            const Vector3 current = input.points[i];
            const double distance = Math::Dot(current, normal) - offset;

            if ((distance < 0.0 && previousDistance > 0.0) || (distance > 0.0 && previousDistance < 0.0))
            {
                const double t = previousDistance / (previousDistance - distance);
                result.points[result.count++] = previous + (current - previous) * t;
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
}
