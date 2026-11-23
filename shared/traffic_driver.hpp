#pragma once

#include "pawn_controller.hpp"
#include "traffic_network.hpp"
#include <functional>
#include <set>

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
    void CollisionSources(std::vector<Vector3>& points) const;
    bool ChangingLanes() const { return changing; }
    bool ClaimsMerge() const;
    double ManeuverStart() const { return entry; }
    double ManeuverEnd() const { return exit; }
    bool FallenOutsideWorld() const;
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
    const TrafficLane* destination = nullptr;
    double destinationStation = 0;
    std::optional<std::size_t> highwayLink, goalLink, goalMovement;
    std::uint64_t goalLane = 0;
    std::set<std::size_t> skippedExits;
    TrafficLane maneuver;
    bool changing = false, mergeGranted = false;
    double decisionDelay = 0, changeCooldown = 0, mergeDelay = 0;
    double laneWaitDistance = -1;
    const char* highwayReason = nullptr;
    void PlanRoute();
    void JoinRoute(const TrafficLane& path, const TrafficLane& to, double start, double end);
    void HighwayUpdate(double seconds);
    bool TryLaneChange(const TrafficLane& to, double length);
    bool GapAvailable(const TrafficLane& path, const TrafficLane& to, double toDistance) const;
    bool PathReady(const TrafficLane& path) const;
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
