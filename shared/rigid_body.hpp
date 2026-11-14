#pragma once

#include "mesh.hpp"

// World-space contact normal points out of the obstacle toward the body.
struct BodyContact
{
    Vector3 point, normal;
    double penetration = 0;
};

// A box-inertia rigid body. Vehicle forces and collision queries are supplied by
// the caller so the integrator can also be used by other simulated world objects.
class RigidBody
{
public:
    void Configure(double kilograms, Vector3 size, Vector3 localCenter);
    void Teleport(const ObjectTransform& pose);
    const ObjectTransform& Pose() const { return pose; }
    Vector3 Center() const;
    Vector3 PointVelocity(Vector3 point) const;
    Vector3 InverseInertia(Vector3 worldVector) const;
    double ImpulseResponse(Vector3 point, Vector3 direction) const;
    void AddForce(Vector3 force, Vector3 point);
    void AddTorque(Vector3 torque);
    void ApplyImpulse(Vector3 impulse, Vector3 point);
    void Integrate(double seconds);
    void ResolveContact(const BodyContact& contact, double restitution, double friction);
    void ResolveContact(RigidBody& other, const BodyContact& contact, double restitution, double friction);
    void Translate(Vector3 delta) { pose.position = pose.position + delta; }
    double Mass() const { return mass; }
    Vector3 velocity{}, angularVelocity{};

private:
    ObjectTransform pose;
    Vector3 localCenter{}, inverseInertia{}, force{}, torque{};
    double mass = 1;
};
