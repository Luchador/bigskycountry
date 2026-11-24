#pragma once
#include "mesh.hpp"
#include <atomic>

// Immutable snapshots let editor bakes update all instances without modifying
// geometry that streaming workers or the renderer may still be reading.
struct MeshAOData
{
    bool cleared = false;
    std::vector<std::vector<float>> alpha;
};

struct MeshAOState
{
    std::string file;
    std::uint64_t fingerprint = 0;
    std::atomic<std::shared_ptr<const MeshAOData>> data;
};

namespace MeshAO
{
    using Snapshot = std::shared_ptr<const MeshAOData>;
    Snapshot Current(const Mesh& mesh);
    float Alpha(const Mesh& mesh, std::size_t vertex, const Snapshot& data);
    // Sidecar: model.fbx.bao / model.glb.bao. Stale topology is ignored safely.
    bool Bind(Mesh& mesh, AssetRoot root);
    // Writes atomically before publishing the new alpha to all loaded instances.
    bool Save(const Mesh& mesh, Snapshot data);
    bool Clear(const Mesh& mesh); // White alpha on every LOD, including imported AO.
}
