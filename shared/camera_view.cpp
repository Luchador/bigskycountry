#include "camera_view.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double pitchLimit = Math::DegreesToRadians(89.0);
}

void CameraView::SetOrientation(double newYaw, double newPitch)
{
    if (yaw == newYaw && pitch == newPitch) return;
    yaw = newYaw;
    pitch = newPitch;
    const double sinYaw = std::sin(yaw), cosYaw = std::cos(yaw), cosPitch = std::cos(pitch);
    forward = {sinYaw * cosPitch, std::sin(pitch), cosYaw * cosPitch};
    right = {cosYaw, 0.0, -sinYaw};
    up = Math::Cross(forward, right);
}

void CameraView::LookAt(Vector3 eye, Vector3 target)
{
    const Vector3 direction = target - eye;
    position = eye;
    SetOrientation(std::atan2(direction.x, direction.z),
                   std::clamp(std::atan2(direction.y, std::hypot(direction.x, direction.z)), -pitchLimit, pitchLimit));
}

Vector3 CameraView::WorldToView(Vector3 point) const
{
    return WorldDirectionToView(point - position);
}

Vector3 CameraView::WorldDirectionToView(Vector3 direction) const
{
    return { Math::Dot(direction, right), Math::Dot(direction, up), Math::Dot(direction, forward) };
}

Vector3 CameraView::ViewRay(double x, double y) const
{
    return forward + right * x + up * y;
}

std::optional<Vector3> CameraView::PickGround(Vector3 direction, double nearDepth) const
{
    if (direction.y == 0.0)
    {
        return std::nullopt;
    }
    const double depth = -position.y / direction.y;
    if (depth < nearDepth)
    {
        return std::nullopt;
    }
    return position + direction * depth;
}
