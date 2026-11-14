#pragma once

#include "player_controller.hpp"
#include <string>

class Character;
class Vehicle;

class Player
{
public:
    explicit Player(GameWorld& world) : controller(world), world(world) {}
    bool Spawn(Vector3 feet, double yaw);
    bool Interact();
    void AfterStep();
    void EnsurePossession();
    Character* Body() const;
    Vehicle* NearbyVehicle() const;
    bool HidePawn(PawnHandle handle) const;
    const std::string& Notice() const { return notice; }
    PlayerController controller;

private:
    GameWorld& world;
    PawnHandle character = 0;
    ObjectTransform spawn, vehicleEntry;
    std::string notice;
};
