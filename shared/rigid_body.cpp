#include "rigid_body.hpp"

#include <algorithm>
#include <cmath>

void RigidBody::Configure(double kilograms, Vector3 size, Vector3 center)
{
    mass = kilograms;
    localCenter = center;
    inverseInertia = {12.0 / (mass * (size.y * size.y + size.z * size.z)),
                      12.0 / (mass * (size.x * size.x + size.z * size.z)),
                      12.0 / (mass * (size.x * size.x + size.y * size.y))};
}

void RigidBody::Teleport(const ObjectTransform& value)
{
    pose = value;
    velocity = angularVelocity = force = torque = {};
}

Vector3 RigidBody::Center() const
{
    return pose.ToMeshTransform().Point(localCenter);
}

Vector3 RigidBody::PointVelocity(Vector3 point) const
{
    return velocity + Math::Cross(angularVelocity, point - Center());
}

Vector3 RigidBody::InverseInertia(Vector3 worldVector) const
{
    const auto frame = pose.ToMeshTransform();
    const Vector3 local = frame.Unrotate(worldVector);
    return frame.Direction({local.x * inverseInertia.x, local.y * inverseInertia.y, local.z * inverseInertia.z});
}

double RigidBody::ImpulseResponse(Vector3 point, Vector3 direction) const
{
    const Vector3 lever = point - Center();
    return 1.0 / mass + Math::Dot(direction, Math::Cross(InverseInertia(Math::Cross(lever, direction)), lever));
}

void RigidBody::AddForce(Vector3 value, Vector3 point)
{
    force = force + value;
    torque = torque + Math::Cross(point - Center(), value);
}

void RigidBody::AddTorque(Vector3 value)
{
    torque = torque + value;
}

void RigidBody::ApplyImpulse(Vector3 impulse, Vector3 point)
{
    velocity = velocity + impulse * (1.0 / mass);
    angularVelocity = angularVelocity + InverseInertia(Math::Cross(point - Center(), impulse));
}

void RigidBody::Integrate(double seconds)
{
    velocity = velocity + (force * (1.0 / mass) + Vector3{0,-9.81,0}) * seconds;
    angularVelocity = angularVelocity + InverseInertia(torque) * seconds;
    const Vector3 center = Center() + velocity * seconds;
    const double spin = Math::Length(angularVelocity);
    if (spin > 1e-10) pose.Rotate(angularVelocity * (1.0 / spin), spin * seconds);
    pose.position = center - pose.ToMeshTransform().Direction(localCenter);
    force = torque = {};
}

void RigidBody::ResolveContact(const BodyContact& contact, double restitution, double friction)
{
    const double closing = Math::Dot(PointVelocity(contact.point), contact.normal);
    if (closing < 0)
    {
        const double bounce = closing < -2.0 ? restitution : 0.0;
        const double impulse = -(1.0 + bounce) * closing / ImpulseResponse(contact.point, contact.normal);
        ApplyImpulse(contact.normal * impulse, contact.point);
        const Vector3 pointVelocity = PointVelocity(contact.point);
        const Vector3 slip = pointVelocity - contact.normal * Math::Dot(pointVelocity, contact.normal);
        const double speed = Math::Length(slip);
        if (speed > 1e-8)
        {
            const Vector3 tangent = slip * (1.0 / speed);
            const double stop = speed / ImpulseResponse(contact.point, tangent);
            ApplyImpulse(tangent * -std::min(stop, friction * impulse), contact.point);
        }
    }
    // Position correction is separate from the velocity impulse to avoid adding
    // energy when a resting body has small residual penetration.
    Translate(contact.normal * (0.75 * std::max(0.0, contact.penetration - 0.002)));
}

void RigidBody::ResolveContact(RigidBody& other, const BodyContact& contact, double restitution, double friction)
{
    const auto apply = [&](Vector3 impulse) {
        ApplyImpulse(impulse, contact.point);
        other.ApplyImpulse(impulse * -1.0, contact.point);
    };
    const double closing = Math::Dot(PointVelocity(contact.point) - other.PointVelocity(contact.point), contact.normal);
    if (closing < 0)
    {
        const double response = ImpulseResponse(contact.point, contact.normal) + other.ImpulseResponse(contact.point, contact.normal);
        const double impulse = -(1.0 + (closing < -2.0 ? restitution : 0.0)) * closing / response;
        apply(contact.normal * impulse);
        const Vector3 relative = PointVelocity(contact.point) - other.PointVelocity(contact.point);
        const Vector3 slip = relative - contact.normal * Math::Dot(relative, contact.normal);
        const double speed = Math::Length(slip);
        if (speed > 1e-8)
        {
            const Vector3 tangent = slip * (1.0 / speed);
            const double stop = speed / (ImpulseResponse(contact.point, tangent) + other.ImpulseResponse(contact.point, tangent));
            apply(tangent * -std::min(stop, friction * impulse));
        }
    }
    const double correction = 0.75 * std::max(0.0, contact.penetration - 0.002);
    const double totalInverseMass = 1.0 / mass + 1.0 / other.mass;
    Translate(contact.normal * (correction / (mass * totalInverseMass)));
    other.Translate(contact.normal * (-correction / (other.mass * totalInverseMass)));
}
