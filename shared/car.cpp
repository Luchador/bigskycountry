#include "car.hpp"
#include "game_world.hpp"
#include "test_pawn_meshes.hpp"

#include <algorithm>
#include <array>
#include <cmath>

Car::Car()
{
    acceleration = 6.0f;
    SetMesh(TestPawnMeshes::Car());
}

void Car::Teleport(const ObjectTransform& transform)
{
    Pawn::Teleport(transform);
    const auto frame = Instance().transform;
    heading = std::atan2(frame.z.x, frame.z.z);
    speed = fallingSpeed = 0;
}

bool Car::BlocksSphere(Vector3 center, double radius) const
{
    const auto frame = GetTransform().ToMeshTransform();
    const Vector3 p = frame.Unrotate(center - frame.origin);
    const Vector3 closest{std::clamp(p.x, -1.0, 1.0), std::clamp(p.y, 0.15, 1.6), std::clamp(p.z, -2.2, 2.2)};
    return Math::Length(p - closest) < radius;
}

bool Car::PlaceOnGround(GameWorld& world, ObjectTransform& transform, double drop) const
{
    ObjectTransform horizontal = transform;
    horizontal.rotation.x = horizontal.rotation.z = 0;
    const auto frame = horizontal.ToMeshTransform();
    std::array<Vector3, 4> contact;
    constexpr Vector3 wheels[] = {{-0.85, 0, -1.35}, {0.85, 0, -1.35}, {-0.85, 0, 1.35}, {0.85, 0, 1.35}};
    for (size_t i = 0; i < contact.size(); ++i)
    {
        const auto hit = world.collision.Ground(frame.Point(wheels[i]), 0.4, drop);
        if (!hit) return false;
        contact[i] = hit->position;
    }
    const double rear = (contact[0].y + contact[1].y) * 0.5;
    const double front = (contact[2].y + contact[3].y) * 0.5;
    const double left = (contact[0].y + contact[2].y) * 0.5;
    const double right = (contact[1].y + contact[3].y) * 0.5;
    transform.position.y = (front + rear) * 0.5;
    const Vector3 normal = Math::Normalize(Vector3{0,1,0} - frame.z * ((front - rear) / 2.7)
                                            - frame.x * ((right - left) / 1.7));
    transform.rotation = horizontal.rotation;
    const Vector3 tiltAxis = Math::Cross({0,1,0}, normal);
    if (Math::Length(tiltAxis) > 1e-10)
        transform.Rotate(Math::Normalize(tiltAxis), std::acos(std::clamp(normal.y, -1.0, 1.0)));
    return true;
}

void Car::Simulate(double seconds, GameWorld& world)
{
    const double throttle = Commands().forward;
    const bool braking = throttle * speed < -0.01;
    const double deceleration = Commands().handbrake ? 16.0 : braking ? 10.0 : 1.2;
    if (Commands().handbrake || braking || throttle == 0)
    {
        const double amount = deceleration * seconds;
        speed = std::copysign(std::max(0.0, std::abs(speed) - amount), speed);
    }
    else speed += throttle * acceleration * seconds;
    speed = std::clamp(speed, -10.0, 40.0);

    auto transform = GetTransform();
    const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(speed) * seconds / 0.15)));
    const double dt = seconds / steps;
    for (int i = 0; i < steps; ++i)
    {
        auto candidate = transform;
        const double steer = Commands().right * 0.55 / (1.0 + std::abs(speed) * 0.04);
        const double candidateHeading = heading + speed / 2.7 * std::tan(steer) * dt;
        candidate.rotation = {0,candidateHeading,0};
        const Vector3 direction{std::sin(candidateHeading), 0, std::cos(candidateHeading)};
        candidate.position = candidate.position + direction * (speed * dt);
        const bool supported = PlaceOnGround(world, candidate, 0.65);
        bool blocked = false;
        const auto frame = candidate.ToMeshTransform();
        for (double z : {-1.25, 0.0, 1.25})
            blocked = blocked || world.SphereBlocked(frame.Point({0, 1.05, z}), 0.9, Handle());
        if (blocked)
        {
            speed = 0;
            break;
        }
        heading = candidateHeading;
        transform = candidate;
        if (supported) fallingSpeed = 0;
        else
        {
            fallingSpeed -= 9.81 * dt;
            transform.position.y += fallingSpeed * dt;
            if (PlaceOnGround(world, transform, std::max(0.65, -fallingSpeed * dt))) fallingSpeed = 0;
        }
    }
    SetTransform(transform);
}
