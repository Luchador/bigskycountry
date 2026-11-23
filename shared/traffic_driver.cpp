#include "traffic_driver.hpp"
#include "car.hpp"
#include "game_world.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>

TrafficDriver::TrafficDriver(GameWorld& world, const TrafficLane& lane, double start)
    : PawnController(world), lane(lane), progress(start), lookAhead(lane.At(start)) {}

void TrafficDriver::Update(double seconds)
{
    auto* car = dynamic_cast<Car*>(Controlled());
    if (!car || seconds <= 0) return;
    PawnInput input;
    const auto frame = car->GetTransform().ToMeshTransform();
    const auto projection = lane.Project(frame.origin, std::max(0.0, progress - 5), progress + 25);
    progress = std::max(progress, projection.distance);
    targetSpeed = 0;
    if (suspended || projection.error > std::max(1.5, lane.width * 0.5 - 0.35) || frame.y.y < 0.6)
    {
        reason = suspended ? "Waiting for collision" : "Off lane - stopped";
        input.brake = 1; Command(input); return;
    }
    const double speed = std::max(0.0, car->Speed());
    const double look = std::clamp(5.0 + speed * 0.85, 5.0, 20.0);
    lookAhead = lane.At(progress + look);
    const Vector3 local = frame.Unrotate(lookAhead - frame.origin);
    const double steering = std::atan2(2.0 * 2.7 * local.x, std::max(1.0, local.x * local.x + local.z * local.z));
    input.right = std::clamp(steering / std::max(0.01, car->SteeringLimit()), -1.0, 1.0);

    constexpr double braking = 3.0, bumperMargin = 3.5;
    double stopAt = lane.Length() - bumperMargin;
    targetSpeed = lane.speedLimit;
    reason = "Following lane";
    // Plan braking for curves, not just the curvature under the car.
    for (double d = 0; d <= 60; d += 3)
    {
        const double at = std::clamp(progress + d, 1.0, std::max(1.0, lane.Length() - 1));
        const Vector3 a = Math::Normalize(lane.At(at) - lane.At(at - 1));
        const Vector3 b = Math::Normalize(lane.At(at + 1) - lane.At(at));
        const double curvature = Math::Length(b - a);
        const double cornerSpeed = std::sqrt(2.0 / std::max(1e-5, curvature));
        const double allowed = std::sqrt(cornerSpeed * cornerSpeed + 2 * braking * std::max(0.0, d - 5));
        if (allowed < targetSpeed) { targetSpeed = allowed; reason = "Slowing for curve"; }
    }
    const double scan = std::min(lane.Length(), progress + std::max(18.0, speed * speed / (2 * braking) + speed + 10));
    for (double d = progress; d <= scan; d += 2)
    {
        const Vector3 p = lane.At(d);
        if (collisionReady && !collisionReady(p))
        {
            stopAt = std::min(stopAt, d - bumperMargin - 4); reason = "Waiting for collision"; break;
        }
        // A small safety check for the initial single-car test. Later milestones
        // replace this with vehicle following and swept-footprint prediction.
        if (world.SphereBlocked(p + Vector3{0, 1.0, 0}, 0.85, car->Handle()))
        {
            stopAt = std::min(stopAt, d - bumperMargin - 1); reason = "Obstacle ahead"; break;
        }
    }
    const double remaining = stopAt - progress;
    const double stoppingSpeed = std::sqrt(2 * braking * std::max(0.0, remaining - 0.5));
    if (stoppingSpeed < targetSpeed)
    {
        targetSpeed = stoppingSpeed;
        if (reason == std::string_view("Following lane") || reason == std::string_view("Slowing for curve")) reason = "Stopping at lane end";
    }
    if (remaining <= 0.75 && speed < 0.5) targetSpeed = 0;
    const double error = targetSpeed - car->Speed();
    if (targetSpeed <= 0.05) input.brake = 1;
    else if (error < -0.15) input.brake = std::clamp(-error * 0.3, 0.0, 1.0);
    else if (error > 0.1) input.forward = std::clamp(error * 0.35, 0.0, 1.0);
    Command(input);
}
