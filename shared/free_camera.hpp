#pragma once

#include "math.hpp"
#include <optional>

struct CameraInput;

class FreeCamera
{
    public:
        void LookAt(Vector3 eye, Vector3 target);
        void Update(const CameraInput& input, double elapsedSeconds);
        Vector3 WorldToView(Vector3 point) const;
        // World direction corresponding to the view-space ray (x, y, 1), without normalization.
        Vector3 ViewRay(double x, double y) const;
        // Intersect the view-space ray (x, y, 1) with the world ground plane.
        std::optional<Vector3> PickGround(double x, double y, double nearDepth) const;

    private:
        Vector3 Forward() const;
        Vector3 Right() const;

        Vector3 position;
        double yaw = 0.0;
        double pitch = 0.0;
};
