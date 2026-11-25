#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>
#include <array>
#include <cmath>

// Shared by terrain, transition outer layers and the world cooker. The
// projection is anchored to the world, never a road, chunk or camera origin.
namespace SurfaceProjection
{
    inline constexpr double tileMeters = 4.0;

    inline std::array<SDL_FPoint, 3> UVs(const std::array<Vector3, 3>& points, Vector3 origin)
    {
        const Vector3 n = Math::Cross(points[1] - points[0], points[2] - points[0]);
        Vector3 u{1, 0, 0}, v{0, 0, 1};
        // Retain a useful projection on steep banks and vertical faces.
        if (std::abs(n.x) > std::abs(n.y) && std::abs(n.x) >= std::abs(n.z)) u = {0, 1, 0};
        else if (std::abs(n.z) > std::abs(n.y)) v = {0, 1, 0};
        // Remove whole repeats in double precision before converting to float.
        // All corners share this offset, so interpolation and mip derivatives
        // remain continuous even at tile boundaries and far from world origin.
        const double originU = std::floor(Math::Dot(origin, u) / tileMeters) * tileMeters;
        const double originV = std::floor(Math::Dot(origin, v) / tileMeters) * tileMeters;
        std::array<SDL_FPoint, 3> uv;
        for (size_t i = 0; i < points.size(); ++i)
            uv[i] = {float((Math::Dot(points[i], u) - originU) / tileMeters),
                     float((Math::Dot(points[i], v) - originV) / tileMeters)};
        return uv;
    }

    inline std::array<SDL_FPoint, 3> UVs(const std::array<Vector3, 3>& points)
    {
        return UVs(points, points[0]);
    }

    struct Frame
    {
        Vector3 tangent;
        float handedness;
    };

    inline Frame TangentFrame(const std::array<Vector3, 3>& points, const std::array<SDL_FPoint, 3>& uv)
    {
        const Vector3 e1 = points[1] - points[0], e2 = points[2] - points[0];
        const double du1 = double(uv[1].x) - uv[0].x, dv1 = double(uv[1].y) - uv[0].y;
        const double du2 = double(uv[2].x) - uv[0].x, dv2 = double(uv[2].y) - uv[0].y;
        const double determinant = du1 * dv2 - du2 * dv1;
        const float handedness = determinant < 0.0 ? -1.0f : 1.0f;
        return {Math::Normalize(determinant == 0.0 ? e1 : (e1 * dv2 - e2 * dv1) * handedness), handedness};
    }
}
