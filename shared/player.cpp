#include "player.hpp"
#include "car.hpp"
#include "character.hpp"
#include "game_world.hpp"
#include "test_pawn_meshes.hpp"

#include <array>
#include <cmath>

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
        vehicleEntry = vehicle->GetTransform();
        body->SetSeated(true);
        AfterStep();
        return true;
    }
    auto* vehicle = dynamic_cast<Vehicle*>(active);
    if (!vehicle) return false;
    if (auto* car = dynamic_cast<Car*>(vehicle); car && std::abs(car->Speed()) > 1.0)
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

void Player::AfterStep()
{
    EnsurePossession();
    auto* body = Body();
    if (!body) return;
    if (auto* vehicle = dynamic_cast<Vehicle*>(controller.Controlled()))
    {
        auto transform = vehicle->GetTransform();
        transform.position = vehicle->Instance().transform.Point(vehicle->SeatPosition());
        body->SetTransform(transform);
    }
    // Recover the test session after falling off the currently authored world.
    if (auto* active = controller.Controlled(); active && active->GetTransform().position.y < spawn.position.y - 100.0)
    {
        if (dynamic_cast<Vehicle*>(active)) active->Teleport(vehicleEntry);
        controller.Release();
        body->SetSeated(false);
        body->Teleport(spawn);
        controller.Possess(character);
    }
}

bool Player::HidePawn(PawnHandle handle) const
{
    const auto* body = Body();
    return body && handle == character && (body->IsSeated() || controller.Controlled() == body);
}
