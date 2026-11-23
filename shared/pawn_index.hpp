#pragma once

#include "pawn.hpp"
#include <unordered_map>

// Persistent XZ hash grid. Exact 3D bounds reject other floors/overpasses.
// Membership covers current bounds and a short velocity sweep for perception.
class PawnIndex
{
public:
    static constexpr double predictionSeconds = 2.5;
    struct Statistics { std::size_t queries = 0, candidates = 0, results = 0; };
    void Update(Pawn& pawn);
    void Remove(PawnHandle handle);
    std::vector<Pawn*> Query(const Bounds3& bounds, bool predicted = false) const;
    void ResetStatistics() { statistics = {}; }
    const Statistics& Stats() const { return statistics; }
    static bool Overlap(const Bounds3& a, const Bounds3& b);
    static Bounds3 Around(Vector3 center, double radius);

private:
    struct Entry
    {
        Pawn* pawn = nullptr;
        Bounds3 current, predicted;
        std::vector<std::uint64_t> cells;
    };
    std::unordered_map<PawnHandle, Entry> entries;
    std::unordered_map<std::uint64_t, std::vector<PawnHandle>> cells;
    std::vector<PawnHandle> oversized;
    mutable Statistics statistics;
    static std::vector<std::uint64_t> Keys(const Bounds3& bounds);
    void Unlink(PawnHandle handle, const Entry& entry);
};
