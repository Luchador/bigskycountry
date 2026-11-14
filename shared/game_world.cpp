#include "game_world.hpp"

#include <algorithm>

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
    for (auto& pawn : pawns) pawn->previous = pawn->GetTransform();
    for (auto& pawn : pawns) pawn->Simulate(seconds, *this);
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
