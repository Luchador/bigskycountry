#include "player.hpp"
#include "car.hpp"
#include "character.hpp"
#include "cooked_world.hpp"
#include "game_world.hpp"
#include "test_pawn_meshes.hpp"

#include <array>
#include <cmath>
#include <limits>

Character* Player::Body() const
{
    return dynamic_cast<Character*>(world.Find(character));
}

bool Player::Spawn(Vector3 feet, double yaw)
{
    auto body = std::make_unique<Character>();
    body->SetMesh(TestPawnMeshes::Character());
    spawn = {feet, {0,yaw,0}};
    body->Teleport(spawn);
    character = world.Add(std::move(body));
    recoveryPawn = 0;
    return controller.Possess(character);
}

Vehicle* Player::NearbyVehicle() const
{
    const auto* body = Body();
    if (!body || body->IsSeated()) return nullptr;
    Vehicle* best = nullptr;
    double distance = 3.6;
    const Vector3 eye = body->GetTransform().position + Vector3{0,1.2,0};
    for (const auto& pawn : world.Pawns())
    {
        auto* vehicle = dynamic_cast<Vehicle*>(pawn.get());
        if (!vehicle || vehicle->IsPossessed() || vehicle->health <= 0) continue;
        const Vector3 seat = vehicle->Instance().transform.Point(vehicle->SeatPosition());
        const double current = Math::Length(seat - eye);
        if (current < distance && !world.collision.Raycast(eye, seat - eye, current))
        {
            distance = current;
            best = vehicle;
        }
    }
    return best;
}

bool Player::Interact()
{
    notice.clear();
    auto* body = Body();
    auto* active = controller.Controlled();
    if (!body || !active) return false;
    if (active == body)
    {
        auto* vehicle = NearbyVehicle();
        if (!vehicle || !controller.Possess(vehicle->Handle())) return false;
        body->SetSeated(true);
        AfterStep();
        return true;
    }
    auto* vehicle = dynamic_cast<Vehicle*>(active);
    if (!vehicle) return false;
    if (auto* car = dynamic_cast<Car*>(vehicle); car && (Math::Length(car->Velocity()) > 1.0 || Math::Length(car->AngularVelocity()) > 0.7))
    {
        notice = "Stop the car before exiting.";
        return false;
    }
    const auto frame = vehicle->Instance().transform;
    constexpr Vector3 exits[] = {{-1.65,0,0}, {1.65,0,0}, {0,0,-3.0}, {0,0,3.0}};
    for (const auto& offset : exits)
    {
        auto transform = body->GetTransform();
        transform.position = frame.Point(offset);
        const auto ground = world.collision.Ground(transform.position, 0.5, 2.0);
        if (!ground) continue;
        transform.position.y = ground->position.y;
        if (!world.CharacterFits(transform.position, character)) continue;
        const auto seat = frame.Point(vehicle->SeatPosition());
        const auto target = transform.position + Vector3{0,0.9,0};
        if (world.collision.Raycast(seat, target - seat, Math::Length(target - seat))) continue;
        transform.rotation = {0,std::atan2(frame.z.x, frame.z.z),0};
        body->Teleport(transform);
        body->SetSeated(false);
        return controller.Possess(character);
    }
    notice = "There is no clear place to exit.";
    return false;
}

void Player::EnsurePossession()
{
    if (!Body())
    {
        controller.Release();
        Spawn(spawn.position, spawn.rotation.y);
        return;
    }
    if (auto* pawn = controller.Controlled(); pawn && pawn->health > 0) return;
    controller.Release();
    if (auto* body = Body())
    {
        body->SetSeated(false);
        body->health = 100;
        body->Teleport(spawn);
        controller.Possess(character);
    }
}

bool Player::AfterStep(const CookedWorld* streaming)
{
    EnsurePossession();
    auto* body = Body();
    auto* active = controller.Controlled();
    if (!body || !active) return false;
    const auto pose = active->GetTransform();
    if (recoveryPawn != active->Handle())
    {
        recoveryPawn = active->Handle();
        lastSupported = pose;
    }
    bool supported = false;
    double verticalSpeed = 0;
    if (const auto* car = dynamic_cast<const Car*>(active))
    {
        supported = car->IsGrounded();
        verticalSpeed = car->Velocity().y;
    }
    else if (active == body)
    {
        supported = body->IsGrounded();
        verticalSpeed = body->Velocity().y;
    }
    bool recovered = false;
    if (supported) lastSupported = pose;
    // Elevation relative to spawn says nothing about whether a pawn fell out
    // of the world. Follow its latest support, and allow long falls when there
    // is still a landing surface below. This query runs only after a 100 m fall;
    // GameSession already waits for nearby streamed collision before stepping.
    else if (verticalSpeed < -1.0 && pose.position.y < lastSupported.position.y - 100.0 &&
             !world.collision.Ground(pose.position, 0, std::numeric_limits<double>::infinity()) &&
             (!streaming || !streaming->MayHaveGroundBelow(pose.position)))
    {
        if (active == body) body->SetSeated(false); // Clear character fall velocity.
        active->Teleport(lastSupported); // Car::Teleport also clears rigid-body velocity.
        notice = "Recovered after falling outside the world.";
        recovered = true;
    }
    if (auto* vehicle = dynamic_cast<Vehicle*>(active))
    {
        auto transform = vehicle->GetTransform();
        transform.position = vehicle->Instance().transform.Point(vehicle->SeatPosition());
        body->SetTransform(transform);
    }
    return recovered;
}

bool Player::HidePawn(PawnHandle handle) const
{
    const auto* body = Body();
    return body && handle == character && (body->IsSeated() || controller.Controlled() == body);
}
