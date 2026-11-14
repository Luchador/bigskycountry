#pragma once

#include "math.hpp"
#include <optional>

// Shared camera geometry for gameplay, editor navigation, sky and world rendering.
class CameraView
{
public:
    void LookAt(Vector3 eye, Vector3 target);
    Vector3 Right() const { return right; }
    Vector3 Up() const { return up; }
    Vector3 Forward() const { return forward; }
    Vector3 GetPosition() const { return position; }
    Vector3 WorldToView(Vector3 point) const;
    Vector3 WorldDirectionToView(Vector3 direction) const;
    // World direction corresponding to the view-space ray (x, y, 1), without normalization.
    Vector3 ViewRay(double x, double y) const;
    // Intersect the view-space ray (x, y, 1) with the world ground plane.
    std::optional<Vector3> PickGround(double x, double y, double nearDepth) const
    {
        return PickGround(ViewRay(x, y), nearDepth);
    }
    std::optional<Vector3> PickGround(Vector3 worldRay, double nearDepth) const;

protected:
    void SetOrientation(double yaw, double pitch);
    Vector3 forward{0, 0, 1}, right{1, 0, 0}, up{0, 1, 0};
    Vector3 position{};
    double yaw = 0, pitch = 0;
};
