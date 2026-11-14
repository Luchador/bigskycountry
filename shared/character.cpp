#include "character.hpp"
#include "game_world.hpp"

#include <algorithm>
#include <cmath>

void Character::SetSeated(bool value)
{
    seated = value;
    velocity = {};
    grounded = false;
}

bool Character::BlocksSphere(Vector3 center, double radius) const
{
    if (seated) return false;
    const auto feet = GetTransform().position;
    const Vector3 nearest{feet.x, std::clamp(center.y, feet.y + 0.35, feet.y + 1.45), feet.z};
    return Math::Length(center - nearest) < radius + 0.35;
}

void Character::Simulate(double seconds, GameWorld& world)
{
    if (seated) return;
    auto transform = GetTransform();
    transform.rotation = {0, Commands().yaw, 0};
    const Vector3 forward{std::sin(Commands().yaw), 0, std::cos(Commands().yaw)};
    const Vector3 right{forward.z, 0, -forward.x};
    Vector3 direction = forward * Commands().forward + right * Commands().right;
    if (Math::Length(direction) > 1) direction = Math::Normalize(direction);
    const double speed = Commands().sprint ? 7.0 : 4.0;
    velocity.x = direction.x * speed;
    velocity.z = direction.z * speed;
    if (grounded && Commands().jump)
    {
        velocity.y = 5.0;
        grounded = false;
    }

    // Short movements keep the swept body from stepping through thin obstacles.
    const Vector3 movement{velocity.x * seconds, 0, velocity.z * seconds};
    const int steps = std::max(1, static_cast<int>(std::ceil(Math::Length(movement) / 0.15)));
    const Vector3 step = movement * (1.0 / steps);
    const auto move = [&](Vector3 delta) {
        Vector3 candidate = transform.position + delta;
        if (grounded)
        {
            if (const auto ground = world.collision.Ground(candidate, 0.35, 0.45))
                candidate.y = ground->position.y;
        }
        if (!world.CharacterFits(candidate, Handle())) return false;
        transform.position = candidate;
        return true;
    };
    for (int i = 0; i < steps; ++i)
    {
        if (!move(step))
        {
            move({step.x, 0, 0});
            move({0, 0, step.z});
        }
    }

    velocity.y -= 9.81 * seconds;
    const double fall = velocity.y * seconds;
    const auto ground = world.collision.Ground(transform.position, 0.04, std::max(0.08, -fall + 0.04));
    grounded = velocity.y <= 0 && ground && transform.position.y + fall <= ground->position.y + 0.04;
    if (grounded)
    {
        transform.position.y = ground->position.y;
        velocity.y = 0;
    }
    else
    {
        const auto candidate = transform.position + Vector3{0, fall, 0};
        if (world.CharacterFits(candidate, Handle())) transform.position = candidate;
        else velocity.y = 0;
    }
    SetTransform(transform);
}
