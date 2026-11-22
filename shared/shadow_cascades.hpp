#pragma once

#include "camera_view.hpp"
#include "mesh.hpp"
#include <array>

// Camera-relative projections built in double precision. Each frustum slice is
// enclosed by a rotation-invariant sphere and snapped to the light's texel grid.
struct ShadowCascade
{
    Vector3 right, up, forward, center;
    double radius = 0, depthRadius = 0, farDepth = 0, texelMeters = 0;
    bool Intersects(const Bounds3& bounds) const;
    std::array<SDL_FColor, 3> Rows(Vector3 eye) const;
};

struct ShadowCascades
{
    static constexpr int count = 4, resolution = 2048;
    std::array<ShadowCascade, count> cascades;
    bool active = false, moon = false;
    double distance = 0;
    void Build(const CameraView& camera, Vector3 lightDirection, double scaleX, double scaleY, double meters);
    bool Intersects(const Bounds3& bounds) const;
};
