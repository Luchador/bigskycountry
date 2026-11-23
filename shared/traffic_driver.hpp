#pragma once

#include "pawn_controller.hpp"
#include "traffic_network.hpp"
#include <functional>

// Milestone 1: one physical car, one directed lane, no junction entry or lane changes.
class TrafficDriver : public PawnController
{
public:
    TrafficDriver(GameWorld& world, const TrafficLane& lane, double start);
    void Update(double seconds) override;
    bool SimulationReady() const override { return !suspended; }
    const TrafficLane& Lane() const { return lane; }
    double Progress() const { return progress; }
    double TargetSpeed() const { return targetSpeed; }
    Vector3 LookAhead() const { return lookAhead; }
    const char* Reason() const { return reason; }
    std::function<bool(Vector3)> collisionReady;
    bool suspended = false;

private:
    const TrafficLane& lane;
    double progress = 0, targetSpeed = 0;
    Vector3 lookAhead{};
    const char* reason = "Starting";
};
