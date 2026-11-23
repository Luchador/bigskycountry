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

// Physical lane following, queues and reserved intersection movements.
class TrafficDriver : public PawnController
{
public:
    TrafficDriver(GameWorld& world, const TrafficLane& lane, double start, TrafficFollowing following = {});
    void Update(double seconds) override;
    void SetNetwork(const TrafficNetwork* value) { network = value; }
    void PrepareRoute();
    bool FallenOutsideWorld() const;
    // Shared by following, junction intrusion and blocked-exit perception.
    static bool Detects(const Pawn& pawn);
    const TrafficMovement* Movement() const;
    unsigned JunctionVisit() const { return visits; }
    double Entry() const { return entry; }
    double Exit() const { return exit; }
    bool JunctionGranted() const { return granted; }
    void JunctionPermission(bool allow, const char* why) { granted = allow; junctionReason = why; }
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
    const TrafficLane* source;
    TrafficLane lane; // Current lane + chosen turn + outgoing lane, for continuous lookahead.
    const TrafficNetwork* network = nullptr;
    std::optional<std::size_t> movement;
    unsigned visits = 0;
    double entry = 0, exit = 0;
    bool granted = false;
    const char* junctionReason = "Approaching junction";
    TrafficFollowing following;
    PawnHandle leader = 0;
    double gap = -1, desiredGap = 0;
    double FollowingSpeed(const Car& car, double speed, double scan, bool& emergency);
    double progress = 0, targetSpeed = 0;
    double lastSupportHeight = 0;
    Vector3 lookAhead{};
    const char* reason = "Starting";
};
