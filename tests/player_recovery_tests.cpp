#include "player.hpp"
#include "car.hpp"
#include "character.hpp"
#include "game_world.hpp"
#include "world_document.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
constexpr double dt = 1.0 / 60.0;
void Check(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}
struct Fixture
{
    WorldDocument document{AssetRoot::Project};
    GameWorld world{document};
    Player player{world};
    Car* car = nullptr;

    Fixture(bool driving)
    {
        AddGround(0, 560);
        AddGround(10000, 420);
        world.collision.Finish();
        auto vehicle = std::make_unique<Car>();
        car = vehicle.get();
        vehicle->Teleport({{0, 560, 0}});
        world.Add(std::move(vehicle));
        Check(player.Spawn({-2.8, 560, 0}, 0), "Spawn failed");
        if (driving) Check(player.Interact(), "Could not enter test car");
        Settle();
    }
    void AddGround(double x, double height)
    {
        auto mesh = std::make_shared<Mesh>();
        mesh->vertices = {{{x-100,height,-100}, {0,1,0}}, {{x+100,height,-100}, {0,1,0}},
                          {{x+100,height,100}, {0,1,0}}, {{x-100,height,100}, {0,1,0}}};
        mesh->indices = {0,2,1,0,3,2};
        mesh->UpdateBounds();
        world.collision.AddMesh({mesh, {}});
    }
    bool Step()
    {
        world.Step(dt);
        return player.AfterStep();
    }
    void Settle()
    {
        for (int i = 0; i < 120; ++i) Check(!Step(), "Supported pawn recovered unexpectedly");
    }
    void Move(Vector3 position)
    {
        auto* pawn = player.controller.Controlled();
        auto pose = pawn->GetTransform();
        pose.position = position;
        if (pawn == player.Body()) player.Body()->SetSeated(false);
        pawn->Teleport(pose);
    }
};

void LowerGround(bool driving)
{
    Fixture f(driving);
    auto* active = f.player.controller.Controlled();
    // The supplied world's initial spawn is around 560 m and Concordia is
    // around 416 m. Distance from spawn must not matter on supported ground.
    f.Move({10000, 420, 0});
    f.Settle();
    Check(f.player.controller.Controlled() == active, "Downhill travel changed possession");
    Check(std::abs(active->GetTransform().position.x - 10000) < 1,
          "Supported pawn returned to spawn");
    Check(std::abs(active->GetTransform().position.y - 420) < 1, "Lost lower ground support");
    if (driving) Check(f.player.Body()->IsSeated(), "Driver was ejected on lower ground");

    const auto supported = active->GetTransform();
    f.Move({10300, 420, 0}); // Beyond the authored terrain.
    bool recovered = false;
    for (int i = 0; i < 600 && !recovered; ++i) recovered = f.Step();
    Check(recovered, "Falling outside the world did not recover");
    Check(f.player.controller.Controlled() == active, "Fall recovery changed possession");
    Check(Math::Length(active->GetTransform().position - supported.position) < .1,
          "Recovery did not return to latest supported position");
    const auto velocity = driving ? f.car->Velocity() : f.player.Body()->Velocity();
    Check(Math::Length(velocity) < 1e-9, "Recovery retained falling velocity");
    if (driving)
    {
        Check(f.player.Body()->IsSeated(), "Recovery ejected driver");
        Check(Math::Length(f.player.Body()->GetTransform().position -
              f.car->Instance().transform.Point(f.car->SeatPosition())) < 1e-8,
              "Driver did not follow recovered car");
    }
    f.Settle();
}

void LongDropOntoTerrain(bool driving)
{
    Fixture f(driving);
    auto* active = f.player.controller.Controlled();
    f.Move({10000, 580, 0}); // 160 m drop with a real landing surface below.
    bool landed = false;
    for (int i = 0; i < 600; ++i)
    {
        Check(!f.Step(), "Legitimate long fall was treated as leaving the world");
        landed |= active->GetTransform().position.y < 425 &&
                  (driving ? f.car->IsGrounded() : f.player.Body()->IsGrounded());
    }
    Check(f.player.controller.Controlled() == active, "Long fall changed possession");
    Check(landed, "Did not land on lower terrain");
}

void ChangingPawns()
{
    Fixture f(true);
    f.Move({10000, 420, 0}); f.Settle();
    Check(f.player.Interact(), "Could not exit car at lower elevation");
    f.Settle();
    const auto feet = f.player.Body()->GetTransform().position;
    f.Move({10300, 420, 0});
    bool recovered = false;
    for (int i = 0; i < 600 && !recovered; ++i) recovered = f.Step();
    Check(recovered && f.player.controller.Controlled() == f.player.Body(), "On-foot recovery after exiting failed");
    Check(Math::Length(f.player.Body()->GetTransform().position - feet) < .1,
          "On-foot recovery reused the car's support position");
}
}

int main()
{
    try
    {
        LowerGround(false); LowerGround(true);
        LongDropOntoTerrain(false); LongDropOntoTerrain(true);
        ChangingPawns();
        std::cout << "Player recovery tests passed\n";
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
