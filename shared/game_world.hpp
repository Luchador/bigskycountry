#pragma once

#include "pawn.hpp"
#include "world_collision.hpp"

class GameWorld
{
public:
    explicit GameWorld(const WorldDocument& document) { collision.Build(document); }
    PawnHandle Add(std::unique_ptr<Pawn> pawn);
    Pawn* Find(PawnHandle handle) const;
    bool Remove(PawnHandle handle);
    void Step(double seconds);
    bool SphereBlocked(Vector3 center, double radius, PawnHandle ignore = 0) const;
    bool CharacterFits(Vector3 feet, PawnHandle ignore = 0) const;
    const auto& Pawns() const { return pawns; }
    WorldCollision collision;

private:
    std::vector<std::unique_ptr<Pawn>> pawns;
    PawnHandle nextHandle = 0;
};
