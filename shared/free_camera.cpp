#include "free_camera.hpp"
#include "input.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr double moveSpeed = 3.0; // meters per second
    constexpr double lookSensitivity = 0.0025; // radians per mouse unit
    constexpr double pitchLimit = 89.0 * std::numbers::pi / 180.0;

    double Dot(Vector3 a, Vector3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
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

void FreeCamera::Update(const CameraInput& input, double elapsedSeconds)
{
    yaw = std::remainder(yaw + input.lookX * lookSensitivity, 2.0 * std::numbers::pi);
    pitch = std::clamp(pitch - input.lookY * lookSensitivity, -pitchLimit, pitchLimit);

    const Vector3 forward = Forward();
    const Vector3 right = Right();

    const Vector3 move =
    {
        forward.x * input.forward + right.x * input.right,
        forward.y * input.forward + input.up,
        forward.z * input.forward + right.z * input.right
    };

    const double length = std::sqrt(Dot(move, move));

    if (length > 0.0)
    {
        const double distance = moveSpeed * elapsedSeconds / length;
        position.x += move.x * distance;
        position.y += move.y * distance;
        position.z += move.z * distance;
    }
}

Vector3 FreeCamera::WorldToView(Vector3 point) const
{
    const Vector3 offset = { point.x - position.x, point.y - position.y, point.z - position.z };
    const Vector3 forward = Forward();
    const Vector3 right = Right();

    // Camera up is forward crossed with right. Right always lies in the XZ plane.
    const Vector3 up = { forward.y * right.z, forward.z * right.x - forward.x * right.z, -forward.y * right.x };

    return { Dot(offset, right), Dot(offset, up), Dot(offset, forward) };
}
