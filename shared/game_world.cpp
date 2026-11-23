#include "game_world.hpp"
#include "car.hpp"
#include "pawn_controller.hpp"

#include <algorithm>
#include <cmath>

PawnHandle GameWorld::Add(std::unique_ptr<Pawn> pawn)
{
    if (!pawn) return 0;
    pawn->handle = ++nextHandle;
    pawn->controller = nullptr;
    pawn->input = {};
    pawn->previous = pawn->GetTransform();
    const auto handle = pawn->handle;
    byHandle.emplace(handle, pawn.get());
    pawnIndex.Update(*pawn);
    pawns.push_back(std::move(pawn));
    return handle;
}

Pawn* GameWorld::Find(PawnHandle handle) const
{
    const auto it = byHandle.find(handle);
    return it == byHandle.end() ? nullptr : it->second;
}

bool GameWorld::Remove(PawnHandle handle)
{
    if (auto* pawn = Find(handle); pawn && pawn->controller) pawn->controller->Release();
    pawnIndex.Remove(handle);
    byHandle.erase(handle);
    return std::erase_if(pawns, [handle](const auto& pawn) { return pawn->Handle() == handle; }) != 0;
}

void GameWorld::Step(double seconds)
{
    if (!std::isfinite(seconds) || seconds <= 0) return;
    // All controllers observe the same pre-physics poses. Teleports, seating,
    // collision toggles and spawning are reflected before perception runs.
    SyncPawns();
    stepping = true;
    pairChecks = 0;
    pawnIndex.ResetStatistics();
    for (auto& pawn : pawns) if (pawn->controller) pawn->controller->Update(seconds);
    int steps = 1;
    for (auto& pawn : pawns)
    {
        pawn->previous = pawn->GetTransform();
        if (!pawn->controller || pawn->controller->SimulationReady())
            steps = std::max(steps, pawn->SimulationSubsteps(seconds));
    }
    const double dt = seconds / steps;
    for (int step = 0; step < steps; ++step)
    {
        for (auto& pawn : pawns)
            if (!pawn->controller || pawn->controller->SimulationReady())
            {
                pawn->Simulate(dt, *this);
                pawnIndex.Update(*pawn);
            }
        for (auto& pawn : pawns)
        {
            auto* car = dynamic_cast<Car*>(pawn.get());
            if (!car || !car->hasCollision || (car->controller && !car->controller->SimulationReady())) continue;
            for (auto* candidate : NearbyPawns(car->CollisionBounds()))
                if (auto* other = dynamic_cast<Car*>(candidate); other && other->Handle() > car->Handle() &&
                    (!other->controller || other->controller->SimulationReady()))
                {
                    ++pairChecks;
                    car->ResolveCollision(*other);
                    pawnIndex.Update(*car); pawnIndex.Update(*other);
                }
        }
    }
    stepping = false;
}

void GameWorld::SyncPawns() const
{
    for (const auto& pawn : pawns) pawnIndex.Update(*pawn);
}

std::vector<Pawn*> GameWorld::NearbyPawns(const Bounds3& bounds, bool predicted) const
{
    // Calls outside Step include placement and entering/exiting vehicles. They
    // can follow direct pose changes, so refresh before consulting the grid.
    if (!stepping) SyncPawns();
    return pawnIndex.Query(bounds, predicted);
}

bool GameWorld::SphereBlocked(Vector3 center, double radius, PawnHandle ignore) const
{
    if (collision.SphereBlocked(center, radius)) return true;
    for (const auto* pawn : NearbyPawns(PawnIndex::Around(center, radius)))
        if (pawn->hasCollision && pawn->Handle() != ignore && pawn->BlocksSphere(center, radius)) return true;
    return false;
}

bool GameWorld::CharacterFits(Vector3 feet, PawnHandle ignore) const
{
    for (double height : {0.36, 0.90, 1.44})
        if (SphereBlocked(feet + Vector3{0, height, 0}, 0.35, ignore)) return false;
    return true;
}
