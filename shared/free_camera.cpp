#include "free_camera.hpp"
#include "input.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr double moveSpeed = 3.0; // meters per second
    constexpr double lookSensitivity = 0.0025; // radians per mouse unit
    constexpr double pitchLimit = Math::DegreesToRadians(89.0);
}

Vector3 FreeCamera::Forward() const
{
    const double cosPitch = std::cos(pitch);

    return { std::sin(yaw) * cosPitch, std::sin(pitch), std::cos(yaw) * cosPitch };
}

Vector3 FreeCamera::Right() const
{
    return { std::cos(yaw), 0.0, -std::sin(yaw) };
}

void FreeCamera::LookAt(Vector3 eye, Vector3 target)
{
    const Vector3 direction = target - eye;
    position = eye;
    yaw = std::atan2(direction.x, direction.z);
    pitch = std::clamp(std::atan2(direction.y, std::hypot(direction.x, direction.z)), -pitchLimit, pitchLimit);
}

void FreeCamera::Update(const CameraInput& input, double elapsedSeconds)
{
    yaw = std::remainder(yaw + input.lookX * lookSensitivity, 2.0 * std::numbers::pi);
    pitch = std::clamp(pitch - input.lookY * lookSensitivity, -pitchLimit, pitchLimit);

    const Vector3 forward = Forward();
    const Vector3 right = Right();

    const Vector3 worldUp = { 0.0, 1.0, 0.0 };
    const Vector3 move = forward * input.forward + right * input.right + worldUp * input.up;
    position = position + Math::Normalize(move) * (moveSpeed * elapsedSeconds);
}

Vector3 FreeCamera::WorldToView(Vector3 point) const
{
    return WorldDirectionToView(point - position);
}

Vector3 FreeCamera::WorldDirectionToView(Vector3 direction) const
{
    const Vector3 forward = Forward();
    const Vector3 right = Right();

    const Vector3 up = Math::Cross(forward, right);

    return { Math::Dot(direction, right), Math::Dot(direction, up), Math::Dot(direction, forward) };
}

Vector3 FreeCamera::ViewRay(double x, double y) const
{
    const Vector3 forward = Forward();
    const Vector3 right = Right();
    return forward + right * x + Math::Cross(forward, right) * y;
}

std::optional<Vector3> FreeCamera::PickGround(double x, double y, double nearDepth) const
{
    const Vector3 direction = ViewRay(x, y);
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
