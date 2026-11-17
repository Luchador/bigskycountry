#pragma once

#include "mesh.hpp"
#include "lake.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <set>

struct TerrainBoundary
{
    std::set<int> roads, intersections; // Stable world IDs, independent of outliner order.
    bool operator==(const TerrainBoundary&) const = default;
};

// Imported chunks are rigid placements. Generated chunks share world coordinates with their rim.
class TerrainChunk : private MeshPlacement
{
public:
    using MeshPlacement::persistentId;
    using MeshPlacement::GetTransform;
    using MeshPlacement::Instance;
    using MeshPlacement::SetMesh;

    const std::optional<LakeDefinition>& Lake() const { return lakeDraft ? lakeDraft : lake; }
    const MeshInstance& Water() const { return water; }
    const auto& Shoreline() const { return lakeDraft ? draftShoreline : shoreline; }
    // Terrain seams follow the committed mesh, not the lightweight edit preview.
    const auto& BuiltShoreline() const { return shoreline; }
    std::uint64_t LakeRevision() const { return lakeRevision; }
    static std::uint64_t LatestLakeRevision() { return latestLakeRevision; }
    bool SetLake(const LakeDefinition& definition);
    bool EditingLake() const { return lakeDraft.has_value(); }
    bool BeginLakeEdit();
    bool FinishLakeEdit(); // Failed validation keeps the draft open and old meshes intact.
    void CancelLakeEdit();

    struct OutlineEdge
    {
        Vector3 a, b, normal; // Adjacent face normal, for surface-aligned outline depth.
    };
    // Local-space boundary edges, rebuilt lazily when the mesh changes.
    const std::vector<OutlineEdge>& Outline() const;

    std::optional<TerrainBoundary> boundary;
    static constexpr SDL_FColor groundColor{0.28f, 0.40f, 0.19f, 1};

    // Derived state, never serialized. A missing rim temporarily suspends the mesh.
    std::vector<std::uint64_t> sourceRevisions;
    std::vector<std::array<Vector3, 2>> anchorLinks;
    // Candidate lake IDs and committed revisions. IDs survive outliner reordering;
    // these derived dependencies are rediscovered after loading the authored world.
    std::vector<std::pair<std::uint64_t, std::uint64_t>> sourceLakes;
    std::string material; // Relative .bmat path, or empty to use the mesh colors.

    bool SetTransform(const ObjectTransform& t)
    {
        if (lakeDraft && t != GetTransform()) return SDL_SetError("Finish shoreline editing before moving the lake");
        if (lake && (t.rotation.x != 0 || t.rotation.z != 0)) return SDL_SetError("Lakes must remain horizontal");
        if (boundary && t != ObjectTransform{}) return SDL_SetError("Move the terrain boundary to transform this chunk");
        if (t.scale != Vector3{1, 1, 1})
        {
            return SDL_SetError("Terrain chunks use unit scale");
        }
        const bool changed = t != GetTransform();
        if (!MeshPlacement::SetTransform(t)) return false;
        water.transform = Instance().transform;
        if (lake && changed) LakeChanged();
        return true;
    }

private:
    void LakeChanged() { lakeRevision = ++latestLakeRevision; }
    inline static std::uint64_t latestLakeRevision = 0;
    std::uint64_t lakeRevision = 0;
    std::optional<LakeDefinition> lake;
    std::optional<LakeDefinition> lakeDraft; // Transient editor state; never serialized.
    std::vector<LakeSample> draftShoreline;
    MeshInstance water;
    std::vector<LakeSample> shoreline;
    mutable std::weak_ptr<const Mesh> outlinedMesh;
    mutable std::vector<OutlineEdge> outline;
};
