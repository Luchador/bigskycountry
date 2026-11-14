#include "free_camera.hpp"
#include "input.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr double minMoveSpeed = 0.1; // meters per second
    constexpr double maxMoveSpeed = 50'000.0;
    const double speedLogRange = std::log(maxMoveSpeed / minMoveSpeed);
    constexpr double lookSensitivity = 0.0025; // radians per mouse unit
    constexpr double pitchLimit = Math::DegreesToRadians(89.0);
}

double FreeCamera::GetSpeedFraction() const
{
    return std::log(moveSpeed / minMoveSpeed) / speedLogRange;
}

void FreeCamera::SetSpeedFraction(double fraction)
{
    if (fraction <= 0.0)
    {
        moveSpeed = minMoveSpeed;
    }
    else if (fraction >= 1.0)
    {
        moveSpeed = maxMoveSpeed;
    }
    else
    {
        moveSpeed = minMoveSpeed * std::exp(fraction * speedLogRange);
    }
}

void FreeCamera::ScrollSpeed(double wheelSteps)
{
    // Sixty wheel notches cover the slider. Trackpads can send fractional steps.
    if (wheelSteps != 0.0)
    {
        SetSpeedFraction(GetSpeedFraction() + wheelSteps / 60.0);
    }
}

void FreeCamera::SetOrientation(double newYaw, double newPitch)
{
    if (yaw == newYaw && pitch == newPitch) return;
    yaw = newYaw;
    pitch = newPitch;
    const double sinYaw = std::sin(yaw), cosYaw = std::cos(yaw), cosPitch = std::cos(pitch);
    forward = {sinYaw * cosPitch, std::sin(pitch), cosYaw * cosPitch};
    right = {cosYaw, 0.0, -sinYaw};
    up = Math::Cross(forward, right);
}

void FreeCamera::LookAt(Vector3 eye, Vector3 target)
{
    const Vector3 direction = target - eye;
    position = eye;
    SetOrientation(std::atan2(direction.x, direction.z),
                   std::clamp(std::atan2(direction.y, std::hypot(direction.x, direction.z)), -pitchLimit, pitchLimit));
}

void FreeCamera::Update(const CameraInput& input, double elapsedSeconds)
{
    SetOrientation(std::remainder(yaw + input.lookX * lookSensitivity, 2.0 * std::numbers::pi),
                   std::clamp(pitch - input.lookY * lookSensitivity, -pitchLimit, pitchLimit));

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
    return { Math::Dot(direction, right), Math::Dot(direction, up), Math::Dot(direction, forward) };
}

Vector3 FreeCamera::ViewRay(double x, double y) const
{
    return forward + right * x + up * y;
}

std::optional<Vector3> FreeCamera::PickGround(Vector3 direction, double nearDepth) const
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
