#pragma once

#include "river.hpp"
#include "terrain_fill.hpp"

#include <filesystem>

// Disposable editor-only acceleration. The authored world remains the source of
// truth; cache misses/errors always fall back to the ordinary geometry builders.
// Owns no meshes. Shared with water chunks so edits and undo use the same cache.
class EditorGeometryCache
{
public:
    struct Statistics { std::uint64_t hits = 0, misses = 0, writes = 0, errors = 0; };
    explicit EditorGeometryCache(std::filesystem::path directory,
                                 std::uint64_t budgetBytes = 1024ull * 1024 * 1024);
    static std::shared_ptr<EditorGeometryCache> CreateDefault();
    const Statistics& Stats() const { return statistics; }

    std::optional<LakeGeometry> Lake(const LakeDefinition& definition);
    std::optional<LakeGeometry> River(const RiverDefinition& definition);
    std::shared_ptr<const Mesh> Fill(std::span<const RoadGeometry* const> footprints,
                                   std::span<const std::array<Vector3, 2>> links,
                                   std::span<const std::vector<Vector3>> exclusions,
                                   std::span<const std::array<Vector3, 2>> junctionSeams,
                                   TerrainFillScope scope,
                                   std::span<const RoadGeometry* const> supportingFootprints);

private:
    using Key = std::array<std::uint64_t, 2>;
    std::filesystem::path directory;
    std::uint64_t budget, bytesWritten = 0;
    Statistics statistics;
    std::filesystem::path File(Key key) const;
    std::optional<std::vector<char>> Read(Key key);
    void Write(Key key, std::span<const char> payload);
    void Reject(Key key);
    void Prune();
    std::optional<LakeGeometry> Water(const WaterDefinition& definition);
};
