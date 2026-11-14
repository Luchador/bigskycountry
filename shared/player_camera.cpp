#include "player_camera.hpp"
#include "world_collision.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

void PlayerCamera::Possess(const Pawn& pawn)
{
    transitionStart = view;
    blend = initialized ? 0.0 : 1.0;
    yaw = std::atan2(pawn.Forward().x, pawn.Forward().z);
    lastPawnYaw = yaw;
    pitch = pawn.CameraSettings().mode == PawnCameraMode::Chase ? -0.24 : 0.0;
}

void PlayerCamera::Look(double x, double y)
{
    constexpr double sensitivity = 0.0025;
    yaw = std::remainder(yaw + x * sensitivity, 2 * std::numbers::pi);
    pitch = std::clamp(pitch - y * sensitivity, -1.45, 1.45);
}

void PlayerCamera::Update(const Pawn& pawn, double fraction, double seconds, const WorldCollision& collision)
{
    const auto settings = pawn.CameraSettings();
    const auto transform = pawn.InterpolatedTransform(fraction);
    const auto frame = transform.ToMeshTransform();
    if (settings.mode == PawnCameraMode::Chase && std::hypot(frame.z.x, frame.z.z) > 0.2)
    {
        const double heading = std::atan2(frame.z.x, frame.z.z);
        // Follow chassis yaw smoothly through jumps and rollovers. A nearly
        // vertical forward axis has no useful horizontal heading.
        const double turn = std::remainder(heading - lastPawnYaw, 2 * std::numbers::pi) *
                            (1.0 - std::exp(-8.0 * seconds));
        yaw += turn;
        lastPawnYaw += turn;
    }
    Vector3 direction{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
    const Vector3 pivot = frame.Point(settings.pivot);
    Vector3 eye = pivot - direction * settings.distance;
    blend = std::min(1.0, blend + seconds / 0.25);
    const double t = blend * blend * (3.0 - 2.0 * blend);
    if (initialized && t < 1)
    {
        eye = transitionStart.GetPosition() + (eye - transitionStart.GetPosition()) * t;
        const Vector3 mixed = transitionStart.Forward() * (1 - t) + direction * t;
        if (Math::Length(mixed) > 1e-6) direction = Math::Normalize(mixed);
    }
    if (settings.mode == PawnCameraMode::Chase)
    {
        const Vector3 offset = eye - pivot;
        double distance = Math::Length(offset);
        const Vector3 ray = Math::Normalize(offset);
        // Parallel probes keep the camera's near plane clear of walls and ground.
        for (Vector3 shift : {Vector3{}, Vector3{0.2,0,0}, Vector3{-0.2,0,0}, Vector3{0,0.2,0}, Vector3{0,-0.2,0}})
            if (auto hit = collision.Raycast(pivot + shift, ray, distance))
                distance = std::max(0.0, hit->distance - 0.25);
        eye = pivot + ray * distance;
    }
    view.LookAt(eye, eye + direction);
    initialized = true;
}
