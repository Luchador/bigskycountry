#pragma once

#include "mesh.hpp"
#include "river.hpp"

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
    const std::optional<RiverDefinition>& River() const { return riverDraft ? riverDraft : river; }
    bool HasWater() const { return lake.has_value() || river.has_value(); }
    std::optional<WaterDefinition> WaterSource() const
    {
        if (River()) return *River();
        if (Lake()) return *Lake();
        return {};
    }
    bool SetWater(const WaterDefinition& definition);
    bool SetRiver(const RiverDefinition& definition);
    bool EditingWater() const { return lakeDraft.has_value() || riverDraft.has_value(); }
    bool BeginWaterEdit();
    bool FinishWaterEdit();
    void CancelWaterEdit();
    const MeshInstance& Water() const { return water; }
    const auto& Shoreline() const { return EditingWater() ? draftShoreline : shoreline; }
    // Terrain seams follow the committed mesh, not the lightweight edit preview.
    const auto& BuiltShoreline() const { return shoreline; }
    std::uint64_t WaterRevision() const { return waterRevision; }
    static std::uint64_t LatestWaterRevision() { return latestWaterRevision; }
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
    // Candidate lake/river IDs and committed revisions. IDs survive outliner reordering;
    // these derived dependencies are rediscovered after loading the authored world.
    std::vector<std::pair<std::uint64_t, std::uint64_t>> sourceWaterBodies;
    std::string material; // Relative .bmat path, or empty to use the mesh colors.

    bool SetTransform(const ObjectTransform& t)
    {
        if (EditingWater() && t != GetTransform()) return SDL_SetError("Finish shoreline editing before moving the water body");
        if (HasWater() && (t.rotation.x != 0 || t.rotation.z != 0)) return SDL_SetError("Water surfaces must remain horizontal");
        if (boundary && t != ObjectTransform{}) return SDL_SetError("Move the terrain boundary to transform this chunk");
        if (t.scale != Vector3{1, 1, 1})
        {
            return SDL_SetError("Terrain chunks use unit scale");
        }
        const bool changed = t != GetTransform();
        if (!MeshPlacement::SetTransform(t)) return false;
        water.transform = Instance().transform;
        if (HasWater() && changed) WaterChanged();
        return true;
    }

private:
    void WaterChanged() { waterRevision = ++latestWaterRevision; }
    inline static std::uint64_t latestWaterRevision = 0;
    std::uint64_t waterRevision = 0;
    std::optional<RiverDefinition> river, riverDraft;
    std::optional<LakeDefinition> lake;
    std::optional<LakeDefinition> lakeDraft; // Transient editor state; never serialized.
    std::vector<LakeSample> draftShoreline;
    MeshInstance water;
    std::vector<LakeSample> shoreline;
    mutable std::weak_ptr<const Mesh> outlinedMesh;
    mutable std::vector<OutlineEdge> outline;
};
