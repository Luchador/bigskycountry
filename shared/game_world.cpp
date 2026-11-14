#include "game_world.hpp"
#include "car.hpp"

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
    pawns.push_back(std::move(pawn));
    return handle;
}

Pawn* GameWorld::Find(PawnHandle handle) const
{
    if (!handle) return nullptr;
    for (const auto& pawn : pawns)
        if (pawn->Handle() == handle) return pawn.get();
    return nullptr;
}

bool GameWorld::Remove(PawnHandle handle)
{
    return std::erase_if(pawns, [handle](const auto& pawn) { return pawn->Handle() == handle; }) != 0;
}

void GameWorld::Step(double seconds)
{
    if (!std::isfinite(seconds) || seconds <= 0) return;
    int steps = 1;
    for (auto& pawn : pawns)
    {
        pawn->previous = pawn->GetTransform();
        steps = std::max(steps, pawn->SimulationSubsteps(seconds));
    }
    const double dt = seconds / steps;
    for (int step = 0; step < steps; ++step)
    {
        for (auto& pawn : pawns) pawn->Simulate(dt, *this);
        for (size_t i = 0; i < pawns.size(); ++i)
        {
            auto* car = dynamic_cast<Car*>(pawns[i].get());
            if (!car) continue;
            for (size_t j = i + 1; j < pawns.size(); ++j)
                if (auto* other = dynamic_cast<Car*>(pawns[j].get())) car->ResolveCollision(*other);
        }
    }
}

bool GameWorld::SphereBlocked(Vector3 center, double radius, PawnHandle ignore) const
{
    if (collision.SphereBlocked(center, radius)) return true;
    for (const auto& pawn : pawns)
        if (pawn->Handle() != ignore && pawn->BlocksSphere(center, radius)) return true;
    return false;
}

bool GameWorld::CharacterFits(Vector3 feet, PawnHandle ignore) const
{
    for (double height : {0.36, 0.90, 1.44})
        if (SphereBlocked(feet + Vector3{0, height, 0}, 0.35, ignore)) return false;
    return true;
}
