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
    using MeshPlacement::GetTransform;
    using MeshPlacement::Instance;
    using MeshPlacement::SetMesh;

    const std::optional<LakeDefinition>& Lake() const { return lake; }
    const MeshInstance& Water() const { return water; }
    const auto& Shoreline() const { return shoreline; }
    bool SetLake(const LakeDefinition& definition);

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
    std::string material; // Relative .bmat path, or empty to use the mesh colors.

    bool SetTransform(const ObjectTransform& t)
    {
        if (lake && (t.rotation.x != 0 || t.rotation.z != 0)) return SDL_SetError("Lakes must remain horizontal");
        if (boundary && t != ObjectTransform{}) return SDL_SetError("Move the terrain boundary to transform this chunk");
        if (t.scale != Vector3{1, 1, 1})
        {
            return SDL_SetError("Terrain chunks use unit scale");
        }
        if (!MeshPlacement::SetTransform(t)) return false;
        water.transform = Instance().transform;
        return true;
    }

private:
    std::optional<LakeDefinition> lake;
    MeshInstance water;
    std::vector<LakeSample> shoreline;
    mutable std::weak_ptr<const Mesh> outlinedMesh;
    mutable std::vector<OutlineEdge> outline;
};
