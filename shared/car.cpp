#include "car.hpp"
#include "character.hpp"
#include "game_world.hpp"
#include "test_pawn_meshes.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace
{
    constexpr Vector3 wheelAnchors[] = {{-0.85,0.75,-1.35}, {0.85,0.75,-1.35},
                                        {-0.85,0.75,1.35}, {0.85,0.75,1.35}};
    // Rounded chassis, bumpers and cabin. They also catch roof/side landings.
    constexpr double hullRadius = 0.43;
    constexpr Vector3 hull[] = {{-0.55,0.60,-1.70}, {0.55,0.60,-1.70},
                                {-0.55,0.60,0}, {0.55,0.60,0},
                                {-0.55,0.60,1.70}, {0.55,0.60,1.70},
                                {-0.42,1.20,-0.65}, {0.42,1.20,-0.65},
                                {-0.42,1.20,0.65}, {0.42,1.20,0.65}};
}

Car::Car(CarHandling settings) : handling(settings)
{
    acceleration = 8.0f;
    body.Configure(handling.mass, {1.9,1.3,4.3}, handling.centerOfMass);
    SetMesh(TestPawnMeshes::Car());
}

void Car::Teleport(const ObjectTransform& transform)
{
    Pawn::Teleport(transform);
    body.Teleport(GetTransform());
    steering = 0;
    groundedWheels = 0;
}

double Car::Speed() const
{
    return Math::Dot(body.velocity, body.Pose().ToMeshTransform().z);
}

int Car::SimulationSubsteps(double seconds) const
{
    // Bound both time and movement of the outermost chassis point. This keeps
    // the contact spheres from crossing thin road decks or walls in one step.
    const double motion = Math::Length(body.velocity) + 2.5 * Math::Length(body.angularVelocity) + 9.81 * seconds;
    return std::clamp(static_cast<int>(std::ceil(std::max(seconds * 240.0, seconds * motion / 0.10))), 1, 128);
}

bool Car::BlocksSphere(Vector3 center, double radius) const
{
    const auto frame = GetTransform().ToMeshTransform();
    const Vector3 p = frame.Unrotate(center - frame.origin);
    const Vector3 closest{std::clamp(p.x, -1.0, 1.0), std::clamp(p.y, 0.15, 1.6), std::clamp(p.z, -2.2, 2.2)};
    return Math::Length(p - closest) < radius;
}

void Car::ApplyWheelForces(double seconds, GameWorld& world)
{
    const auto frame = body.Pose().ToMeshTransform();
    groundedWheels = 0;
    Vector3 groundNormal{};
    const double speed = Speed();
    const double targetSteering = Commands().right * handling.steeringAngle / (1.0 + std::abs(speed) * 0.035);
    steering = std::lerp(steering, targetSteering, 1.0 - std::exp(-handling.steeringResponse * seconds));
    const bool braking = !IsPossessed() || Commands().forward * speed < -0.5;

    for (size_t wheel = 0; wheel < std::size(wheelAnchors); ++wheel)
    {
        const Vector3 anchor = frame.Point(wheelAnchors[wheel]);
        const auto hit = world.collision.Raycast(anchor, frame.y * -1.0, handling.suspensionLength + handling.wheelRadius);
        if (!hit || frame.y.y <= 0.1 || Math::Dot(hit->normal, frame.y) < 0.45) continue;
        const double compression = handling.suspensionLength + handling.wheelRadius - hit->distance;
        const double normalSpeed = Math::Dot(body.PointVelocity(anchor), hit->normal);
        const double load = std::clamp(handling.springStrength * compression - handling.suspensionDamping * normalSpeed,
                                       0.0, handling.mass * 20.0);
        if (load <= 0) continue;
        ++groundedWheels;
        groundNormal = groundNormal + hit->normal;
        body.AddForce(hit->normal * load, anchor);

        Vector3 forward = Math::Normalize(frame.z - hit->normal * Math::Dot(frame.z, hit->normal));
        Vector3 right = Math::Cross(hit->normal, forward);
        const bool rear = wheel < 2;
        if (!rear)
        {
            forward = forward * std::cos(steering) + right * std::sin(steering);
            right = Math::Cross(hit->normal, forward);
        }
        const Vector3 contactVelocity = body.PointVelocity(hit->position);
        const double longitudinal = Math::Dot(contactVelocity, forward);
        const double sideways = Math::Dot(contactVelocity, right);
        const double wheelMass = handling.mass / 4;
        double traction = 0;
        if (braking || (Commands().handbrake && rear))
        {
            const double strength = Commands().handbrake && rear ? handling.handbrake : handling.braking;
            traction = -std::copysign(std::min(strength, std::abs(longitudinal) / seconds), longitudinal) * wheelMass;
        }
        else if (Commands().forward != 0)
        {
            const double limit = Commands().forward > 0 ? handling.forwardSpeed : handling.reverseSpeed;
            const double fraction = std::max(0.0, Commands().forward * speed) / limit;
            traction = Commands().forward * acceleration * wheelMass * std::max(0.0, 1.0 - std::pow(fraction, 4));
        }
        else
        {
            const double drag = std::min(0.5 + std::abs(longitudinal) * 0.015, std::abs(longitudinal) / seconds);
            traction = -std::copysign(drag * wheelMass, longitudinal);
        }
        const double grip = load * handling.tireGrip;
        const double lateralGrip = Commands().handbrake && rear ? grip * 0.22 : grip;
        const double lateral = std::clamp(-sideways * wheelMass * handling.lateralResponse, -lateralGrip, lateralGrip);
        const double force = std::hypot(traction, lateral);
        const double scale = force > grip ? grip / force : 1.0;
        body.AddForce(forward * (traction * scale), hit->position);
        // Keep a little cornering lean while giving the arcade car more rollover
        // resistance. Longitudinal tire forces still produce normal brake/drive pitch.
        const double height = Math::Dot(body.Center() - hit->position, hit->normal) * handling.lateralForceHeight;
        body.AddForce(right * (lateral * scale), hit->position + hit->normal * height);
    }
    if (!groundedWheels) return;
    const Vector3 normal = Math::Normalize(groundNormal);
    const double contactFraction = groundedWheels / 4.0;
    const double targetYaw = std::clamp(speed * std::tan(steering) / 2.7, -1.25, 1.25);
    const double yawError = targetYaw - Math::Dot(body.angularVelocity, normal);
    const double assist = Commands().handbrake ? handling.yawAssist * 0.35 : handling.yawAssist;
    const double yawAcceleration = std::clamp(yawError * assist, -5.0, 5.0) * contactFraction;
    const double yawInverseInertia = Math::Dot(normal, body.InverseInertia(normal));
    body.AddTorque(normal * (yawAcceleration / yawInverseInertia));
    // Resist excessive body roll on the ground. Pitch remains free to follow
    // ramps through the suspension, and no leveling torque is applied in flight.
    const double roll = Math::Dot(Math::Cross(frame.y, normal), frame.z);
    const double rollRate = Math::Dot(body.angularVelocity, frame.z);
    body.AddTorque(frame.z * ((roll * handling.antiRoll - rollRate * 0.8) * handling.mass * contactFraction));
}

void Car::ResolveWorldContacts(GameWorld& world)
{
    for (int pass = 0; pass < 3; ++pass)
    {
        for (Vector3 local : hull)
        {
            Vector3 center = body.Pose().ToMeshTransform().Point(local);
            if (const auto hit = world.collision.SphereContact(center, hullRadius))
                body.ResolveContact(*hit, 0.05, 0.65);
            // Characters remain kinematic obstacles. Seated occupants have no
            // collision, while another walking character can stop the chassis.
            for (const auto& pawn : world.Pawns())
            {
                const auto* character = dynamic_cast<const Character*>(pawn.get());
                if (!character || character->IsSeated()) continue;
                center = body.Pose().ToMeshTransform().Point(local);
                const auto feet = character->GetTransform().position;
                const Vector3 nearest{feet.x, std::clamp(center.y, feet.y + 0.35, feet.y + 1.45), feet.z};
                const Vector3 offset = center - nearest;
                const double distance = Math::Length(offset);
                if (distance < hullRadius + 0.35 && distance > 1e-8)
                {
                    const Vector3 normal = offset * (1.0 / distance);
                    body.ResolveContact({nearest + normal * 0.35, normal, hullRadius + 0.35 - distance}, 0, 0.3);
                }
            }
        }
    }
}

void Car::Simulate(double seconds, GameWorld& world)
{
    if (GetTransform() != body.Pose()) body.Teleport(GetTransform());
    ApplyWheelForces(seconds, world);
    body.angularVelocity = body.angularVelocity * std::exp(-0.08 * seconds);
    body.Integrate(seconds);
    ResolveWorldContacts(world);
    SetTransform(body.Pose());
}

void Car::ResolveCollision(Car& other)
{
    if (Math::Length(body.Center() - other.body.Center()) > 5.5) return;
    for (int pass = 0; pass < 2; ++pass)
    {
        for (Vector3 a : hull)
        {
            for (Vector3 b : hull)
            {
                const Vector3 pa = body.Pose().ToMeshTransform().Point(a);
                const Vector3 pb = other.body.Pose().ToMeshTransform().Point(b);
                const Vector3 offset = pa - pb;
                const double distance = Math::Length(offset);
                if (distance >= 2 * hullRadius || distance < 1e-8) continue;
                const Vector3 normal = offset * (1.0 / distance);
                body.ResolveContact(other.body, {(pa + pb) * 0.5, normal, 2 * hullRadius - distance}, 0.1, 0.4);
            }
        }
    }
    SetTransform(body.Pose());
    other.SetTransform(other.body.Pose());
}

bool Car::PlaceOnGround(GameWorld& world, ObjectTransform& transform, double drop) const
{
    ObjectTransform horizontal = transform;
    horizontal.rotation.x = horizontal.rotation.z = 0;
    const auto frame = horizontal.ToMeshTransform();
    std::array<Vector3, 4> contact;
    constexpr Vector3 wheels[] = {{-0.85, 0, -1.35}, {0.85, 0, -1.35}, {-0.85, 0, 1.35}, {0.85, 0, 1.35}};
    for (size_t i = 0; i < contact.size(); ++i)
    {
        const auto hit = world.collision.Ground(frame.Point(wheels[i]), 0.4, drop);
        if (!hit) return false;
        contact[i] = hit->position;
    }
    const double rear = (contact[0].y + contact[1].y) * 0.5;
    const double front = (contact[2].y + contact[3].y) * 0.5;
    const double left = (contact[0].y + contact[2].y) * 0.5;
    const double right = (contact[1].y + contact[3].y) * 0.5;
    transform.position.y = (front + rear) * 0.5;
    const Vector3 normal = Math::Normalize(Vector3{0,1,0} - frame.z * ((front - rear) / 2.7)
                                            - frame.x * ((right - left) / 1.7));
    transform.rotation = horizontal.rotation;
    const Vector3 tiltAxis = Math::Cross({0,1,0}, normal);
    if (Math::Length(tiltAxis) > 1e-10)
        transform.Rotate(Math::Normalize(tiltAxis), std::acos(std::clamp(normal.y, -1.0, 1.0)));
    return true;
}
