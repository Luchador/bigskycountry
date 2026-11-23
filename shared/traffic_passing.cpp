#include "traffic_driver.hpp"
#include "car.hpp"
#include "game_world.hpp"
#include <algorithm>
#include <cmath>

double TrafficDriver::Narrowness(double distance) const
{
    // A normal lane has 5 m. Each direction shares half of a one-lane road;
    // fade the caution/offset away as the authored pavement widens again.
    return std::clamp(3.5 - lane.WidthAt(distance), 0.0, 1.0);
}

double TrafficDriver::PassingWeight(double distance) const
{
    double weight = Narrowness(distance);
    if (Movement())
    {
        // Keep reserved intersection paths exact. Pull back in before the mouth
        // and ease out only after the whole car has cleared the outgoing mouth.
        const double clearance = distance < entry ? entry - distance - 8 : distance - exit - 8;
        weight *= std::clamp(clearance / 12.0, 0.0, 1.0);
    }
    return weight;
}

Vector3 TrafficDriver::DrivingPoint(double distance) const
{
    if (passingOffset == 0) return lane.At(distance);
    const auto right = Math::Normalize(Math::Cross({0,1,0}, lane.Tangent(distance)));
    return lane.At(distance) + right * (passingOffset * PassingWeight(distance));
}

void TrafficDriver::UpdatePassing(double seconds, double speed)
{
    passingHold = std::max(0.0, passingHold - seconds);
    passingCheck -= seconds;
    if (passingCheck <= 0)
    {
        passingCheck = 0.2;
        passingTarget = 0;
        bool narrow = false;
        for (double d = 0; d <= 40; d += 8) narrow = narrow || Narrowness(progress + d) > 0;
        if (narrow)
        {
            const double end = std::min(lane.Length(), progress + 80);
            Bounds3 corridor;
            for (double d = std::max(0.0, progress - 8); d < end; d += 4) corridor.Include(lane.At(d));
            corridor.Include(lane.At(end));
            corridor.min = corridor.min - Vector3{4,2,4}; corridor.max = corridor.max + Vector3{4,3,4};
            for (const auto* other : world.NearbyPawns(corridor, true))
            {
                if (other == Controlled() || !other->hasCollision || !dynamic_cast<const Car*>(other)) continue;
                const auto pose = other->GetTransform().ToMeshTransform();
                const auto p = lane.Project(pose.origin, std::max(0.0, progress - 8), end);
                const auto forward = lane.Tangent(p.distance);
                const auto right = Math::Normalize(Math::Cross({0,1,0}, forward));
                const auto delta = pose.origin - p.position;
                const double ahead = p.distance - progress + Math::Dot(delta, forward);
                if (ahead < -7 || ahead > 80 || std::abs(delta.y) > 2 ||
                    std::abs(Math::Dot(delta, right)) > 4 || Math::Dot(pose.z, forward) > -0.5) continue;
                passingHold = 1.5;
            }
            if (passingHold > 0)
            {
                // At most 45 cm farther right. Check the whole car's swept
                // corridor, not just its center: no blind move into a ditch,
                // obstacle, unloaded terrain, or over the side of a bridge.
                constexpr double offset = 0.45;
                bool supported = true;
                const double reach = std::max(14.0, speed * 2 + 5);
                for (double d = 0; supported && d <= reach; d += 2)
                {
                    const double at = std::clamp(progress + d, 0.0, lane.Length());
                    const auto right = Math::Normalize(Math::Cross({0,1,0}, lane.Tangent(at)));
                    const auto center = lane.At(at) + right * (offset * PassingWeight(at));
                    if ((collisionReady && !collisionReady(center)) ||
                        world.collision.SphereBlocked(center + Vector3{0,1,0}, 0.95)) { supported = false; break; }
                    for (double side : {-1.05, 1.05})
                    {
                        const auto hit = world.collision.Ground(center + right * side, 0.35, 0.35);
                        if (!hit || hit->normal.y < 0.85) { supported = false; break; }
                    }
                }
                if (supported) passingTarget = offset;
            }
        }
    }
    // A gradual steering target, with hysteresis through the actual encounter.
    // Existing following/emergency braking still veto unsafe body clearances.
    const double step = seconds * 0.35;
    passingOffset += std::clamp(passingTarget - passingOffset, -step, step);
}
