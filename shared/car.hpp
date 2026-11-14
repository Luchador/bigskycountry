#pragma once

#include "vehicle.hpp"
#include "rigid_body.hpp"

// All distances and speeds use engine meters. These defaults favor quick steering,
// strong braking and forgiving tire grip while leaving flight under gravity.
struct CarHandling
{
    double mass = 1200;
    Vector3 centerOfMass{0,0.55,0};
    double forwardSpeed = 55, reverseSpeed = 12;
    double braking = 14, handbrake = 20;
    double tireGrip = 2.3, lateralResponse = 9;
    double lateralForceHeight = 0.70; // Fraction of contact-to-COM height used to reduce cornering roll.
    double springStrength = 25000, suspensionDamping = 3200;
    double suspensionLength = 0.52, wheelRadius = 0.35;
    double steeringAngle = 0.35, steeringResponse = 2;
    double yawAssist = 4, antiRoll = 5;
};

class Car : public Vehicle
{
public:
    explicit Car(CarHandling handling = {});
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Car>(*this); }
    void Teleport(const ObjectTransform& transform) override;
    int SimulationSubsteps(double seconds) const override;
    void Simulate(double seconds, GameWorld& world) override;
    bool BlocksSphere(Vector3 center, double radius) const override;
    double Speed() const;
    Vector3 Velocity() const { return body.velocity; }
    Vector3 AngularVelocity() const { return body.angularVelocity; }
    bool IsGrounded() const { return groundedWheels != 0; }
    bool PlaceOnGround(GameWorld& world, ObjectTransform& transform, double drop) const;
    void ResolveCollision(Car& other);

private:
    CarHandling handling;
    RigidBody body;
    double steering = 0;
    int groundedWheels = 0;
    void ApplyWheelForces(double seconds, GameWorld& world);
    void ResolveWorldContacts(GameWorld& world);
};
