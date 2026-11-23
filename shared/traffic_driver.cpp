#include "traffic_driver.hpp"
#include "car.hpp"
#include "game_world.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>

TrafficDriver::TrafficDriver(GameWorld& world, const TrafficLane& lane, double start, TrafficFollowing following)
    : PawnController(world), lane(lane), following(following), progress(start), lookAhead(lane.At(start)) {}

double TrafficDriver::FollowingSpeed(const Car& car, double speed, double scan, bool& emergency)
{
    Bounds3 corridor;
    for (double d = progress; d < scan; d += 3) corridor.Include(lane.At(d));
    corridor.Include(lane.At(scan));
    corridor.min = corridor.min - Vector3{3,1,3}; corridor.max = corridor.max + Vector3{3,3,3};
    double limit = lane.speedLimit;
    desiredGap = following.standstillGap + following.timeHeadway * speed;
    // The grid uses velocity-swept bounds. Crossing players are candidates before
    // their current body reaches the lane; exact heights reject other decks.
    for (const auto* other : world.NearbyPawns(corridor, true))
    {
        if (other->Handle() == car.Handle() || !other->hasCollision) continue;
        const auto* otherCar = dynamic_cast<const Car*>(other);
        const auto* driver = dynamic_cast<const TrafficDriver*>(other->Controller());
        const bool frozen = other->Controller() && !other->Controller()->SimulationReady();
        const auto velocity = frozen ? Vector3{} : other->Velocity();
        const auto pose = other->GetTransform().ToMeshTransform();
        const auto box = other->CollisionBounds();
        const auto current = lane.Project(pose.origin, std::max(0.0, progress - 6), scan + 8);
        if (!std::isfinite(current.error)) continue;
        const bool sameLane = driver && driver->Lane().id == lane.id;
        const int predictions = sameLane || frozen ? 0 :
            std::clamp(static_cast<int>(std::ceil(Math::Length(velocity) * PawnIndex::predictionSeconds)), 1, 128);
        double otherStation = 0;
        if (driver) otherStation = driver->Lane().Project(pose.origin,
            std::max(0.0, driver->Progress() - 8), driver->Progress() + 25).distance;
        for (int sample = 0; sample <= predictions; ++sample)
        {
            const double time = predictions ? PawnIndex::predictionSeconds * sample / predictions : 0;
            Vector3 position = pose.origin + velocity * time;
            if (sample && driver && !frozen)
                position = driver->Lane().At(otherStation + std::max(0.0, otherCar ? otherCar->Speed() : 0.0) * time);
            const auto p = sample ? lane.Project(position, std::max(0.0, progress - 6), scan + 8) : current;
            const auto forward = lane.Tangent(p.distance);
            const auto right = Math::Normalize(Math::Cross({0,1,0}, forward));
            const double height = position.y - pose.origin.y;
            if (box.max.y + height < p.position.y - 0.2 || box.min.y + height > p.position.y + 1.9) continue;
            const double halfLength = otherCar ? std::abs(Math::Dot(pose.z, forward)) * 2.2 +
                std::abs(Math::Dot(pose.x, forward)) + std::abs(Math::Dot(pose.y, forward)) * 0.85 : 0.35;
            const double halfWidth = otherCar ? std::abs(Math::Dot(pose.z, right)) * 2.2 +
                std::abs(Math::Dot(pose.x, right)) + std::abs(Math::Dot(pose.y, right)) * 0.85 : 0.35;
            const double lateral = std::abs(Math::Dot(position - p.position, right));
            if (lateral > 1.0 + halfWidth + 0.25) continue;
            const double along = p.distance - progress + Math::Dot(position - p.position, forward);
            if (along < 0 || along > scan - progress + 6) continue;
            // Do not stop for a predicted crossing we will already have passed.
            if (sample && along + halfLength < speed * time - 2.2 - following.standstillGap) continue;
            const double clearance = along - 2.2 - halfLength;
            const double otherSpeed = Math::Dot(velocity, forward);
            const double closing = std::max(0.0, speed - otherSpeed);
            const bool urgent = clearance < 0.5 || (closing > 0.2 && clearance < 1 + speed * 0.2 + closing * closing / 16.0);
            const double leadSpeed = std::max(0.0, otherSpeed);
            const double followSpeed = leadSpeed + (clearance - desiredGap) / following.responseTime;
            const double delay = following.braking * following.timeHeadway;
            const double safeSpeed = std::sqrt(std::max(0.0, delay * delay + leadSpeed * leadSpeed +
                2 * following.braking * (clearance - following.standstillGap))) - delay;
            double allowed = std::max(0.0, std::min(followSpeed, safeSpeed));
            if (speed < 0.2 && leadSpeed < 0.2 && clearance < following.standstillGap + 0.4) allowed = 0;
            if (urgent) { allowed = 0; emergency = true; }
            if (allowed < limit || (!leader && allowed == limit))
            {
                limit = allowed; leader = other->Handle(); gap = clearance;
            }
        }
    }
    return limit;
}

void TrafficDriver::Update(double seconds)
{
    auto* car = dynamic_cast<Car*>(Controlled());
    if (!car || seconds <= 0) return;
    PawnInput input;
    const auto frame = car->GetTransform().ToMeshTransform();
    const auto projection = lane.Project(frame.origin, std::max(0.0, progress - 5), progress + 25);
    progress = std::max(progress, projection.distance);
    targetSpeed = 0;
    leader = 0; gap = -1; desiredGap = 0;
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

    const double braking = following.braking;
    constexpr double bumperMargin = 3.5;
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
        // Static-world BVHs are separate from dynamic vehicle perception below.
        if (world.collision.SphereBlocked(p + Vector3{0, 1.0, 0}, 0.85))
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
    bool emergency = false;
    const double followingSpeed = FollowingSpeed(*car, speed, scan, emergency);
    if (followingSpeed < targetSpeed)
    {
        targetSpeed = followingSpeed;
        reason = targetSpeed < 0.05 ? "Queued / yielding" : "Following vehicle";
    }
    if (emergency) { targetSpeed = 0; reason = "Emergency braking"; }
    const double error = targetSpeed - car->Speed();
    if (targetSpeed <= 0.05) input.brake = 1;
    else if (error < -0.15) input.brake = std::clamp(-error * 0.3, 0.0, 1.0);
    else if (error > 0.1) input.forward = std::clamp(error * 0.35, 0.0, std::min(1.0, following.acceleration / std::max(0.1, double(car->acceleration))));
    Command(input);
}
