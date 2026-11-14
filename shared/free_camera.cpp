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

void FreeCamera::Update(const CameraInput& input, double elapsedSeconds)
{
    SetOrientation(std::remainder(yaw + input.lookX * lookSensitivity, 2.0 * std::numbers::pi),
                   std::clamp(pitch - input.lookY * lookSensitivity, -pitchLimit, pitchLimit));

    const Vector3 worldUp = { 0.0, 1.0, 0.0 };
    const Vector3 move = forward * input.forward + right * input.right + worldUp * input.up;
    position = position + Math::Normalize(move) * (moveSpeed * elapsedSeconds);
}
