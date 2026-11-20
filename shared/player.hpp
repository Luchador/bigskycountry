#pragma once

#include "player_controller.hpp"
#include <string>

class Character;
class Vehicle;
class CookedWorld;

class Player
{
public:
    explicit Player(GameWorld& world) : controller(world), world(world) {}
    bool Spawn(Vector3 feet, double yaw);
    bool Interact();
    // Returns true when recovery teleports the controlled pawn. Streaming must
    // revalidate collision at the destination before another physics step.
    bool AfterStep(const CookedWorld* streaming = nullptr);
    void EnsurePossession();
    Character* Body() const;
    Vehicle* NearbyVehicle() const;
    bool HidePawn(PawnHandle handle) const;
    const std::string& Notice() const { return notice; }
    PlayerController controller;

private:
    GameWorld& world;
    PawnHandle character = 0;
    ObjectTransform spawn;
    PawnHandle recoveryPawn = 0;
    ObjectTransform lastSupported;
    std::string notice;
};
