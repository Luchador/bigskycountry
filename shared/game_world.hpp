#pragma once

#include "pawn.hpp"
#include "world_collision.hpp"
#include "pawn_index.hpp"

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
    std::vector<Pawn*> NearbyPawns(const Bounds3& bounds, bool predicted = false) const;
    const PawnIndex::Statistics& QueryStatistics() const { return pawnIndex.Stats(); }
    std::size_t CollisionPairChecks() const { return pairChecks; }
    const auto& Pawns() const { return pawns; }
    WorldCollision collision;

private:
    std::vector<std::unique_ptr<Pawn>> pawns;
    std::unordered_map<PawnHandle, Pawn*> byHandle;
    mutable PawnIndex pawnIndex;
    bool stepping = false;
    std::size_t pairChecks = 0;
    void SyncPawns() const;
    PawnHandle nextHandle = 0;
};
