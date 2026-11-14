#pragma once

#include "vehicle.hpp"

class Car : public Vehicle
{
public:
    Car();
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Car>(*this); }
    void Teleport(const ObjectTransform& transform) override;
    void Simulate(double seconds, GameWorld& world) override;
    bool BlocksSphere(Vector3 center, double radius) const override;
    double Speed() const { return speed; }
    bool PlaceOnGround(GameWorld& world, ObjectTransform& transform, double drop) const;

private:
    double speed = 0, fallingSpeed = 0, heading = 0;
};
