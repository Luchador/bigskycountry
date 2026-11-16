#pragma once

#include "mesh.hpp"
#include "scene_hierarchy.hpp"
#include <span>
#include <unordered_set>

// Conservative world-space bounds, including mirrored/nonuniform placements.
Bounds3 WorldBounds(const Bounds3& local, const MeshTransform& transform);
double BoundsDistanceSquared(const Bounds3& bounds, Vector3 point);

// Sparse XZ broad phase with a 3D bounds-distance narrow phase. SceneKey is a
// transient lookup into the current document, not a future persistent asset ID.
class SpatialIndex
{
public:
    static constexpr double cellMeters = 512;
    struct Statistics
    {
        size_t entries = 0, cells = 0, oversized = 0, updated = 0;
        size_t visitedCells = 0, candidates = 0, inRange = 0;
    };

    // A sync touches every live record cheaply; only changed bounds reindex.
    // EndSync removes stale keys after deletion, undo, or document replacement.
    void BeginSync();
    void Upsert(SceneKey key, const Bounds3& bounds);
    void EndSync();
    std::span<const SceneKey> Query(Vector3 eye, double radius);
    const Bounds3& Bounds(SceneKey key) const { return entries.at(key).bounds; }
    const Statistics& Stats() const { return stats; }

private:
    struct KeyHash
    {
        size_t operator()(SceneKey key) const
        { return std::hash<int>{}(key.index) ^ (static_cast<size_t>(key.kind) << 24); }
    };
    struct Cell
    {
        std::int64_t x, z;
        bool operator==(const Cell&) const = default;
    };
    struct CellHash
    {
        size_t operator()(Cell c) const
        { return std::hash<std::int64_t>{}(c.x) ^ (std::hash<std::int64_t>{}(c.z) + 0x9e3779b9u + (std::hash<std::int64_t>{}(c.x) << 6)); }
    };
    struct Entry
    {
        Bounds3 bounds;
        std::vector<Cell> cells;
        std::uint64_t touched = 0, queried = 0;
    };
    using Keys = std::unordered_set<SceneKey, KeyHash>;
    std::unordered_map<SceneKey, Entry, KeyHash> entries;
    std::unordered_map<Cell, Keys, CellHash> cells;
    Keys oversized;
    std::vector<SceneKey> result;
    Statistics stats;
    std::uint64_t sync = 0, query = 0;
    void Unlink(SceneKey key, const Entry& entry);
    static bool CellRange(const Bounds3& bounds, Cell& low, Cell& high);
};
