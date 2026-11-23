#pragma once

#include "pawn_controller.hpp"
#include "traffic_network.hpp"
#include <functional>

class Car;

struct TrafficFollowing
{
    double standstillGap = 3.0; // Clear bumper-to-bumper distance, in game meters.
    double timeHeadway = 1.6;   // Seconds of additional spacing at the follower's speed.
    double braking = 3.0, acceleration = 2.5, responseTime = 1.5;
};

// Physical lane following and queues; junction entry and lane changes come later.
class TrafficDriver : public PawnController
{
public:
    TrafficDriver(GameWorld& world, const TrafficLane& lane, double start, TrafficFollowing following = {});
    void Update(double seconds) override;
    bool SimulationReady() const override { return !suspended; }
    const TrafficLane& Lane() const { return lane; }
    double Progress() const { return progress; }
    double TargetSpeed() const { return targetSpeed; }
    Vector3 LookAhead() const { return lookAhead; }
    const char* Reason() const { return reason; }
    PawnHandle Leader() const { return leader; }
    double Gap() const { return gap; } // -1 when no blocker was found.
    double DesiredGap() const { return desiredGap; }
    std::function<bool(Vector3)> collisionReady;
    bool suspended = false;

private:
    const TrafficLane& lane;
    TrafficFollowing following;
    PawnHandle leader = 0;
    double gap = -1, desiredGap = 0;
    double FollowingSpeed(const Car& car, double speed, double scan, bool& emergency);
    double progress = 0, targetSpeed = 0;
    Vector3 lookAhead{};
    const char* reason = "Starting";
};
