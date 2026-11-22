#pragma once
#include "river.hpp"
#include <span>

namespace RiverJunction
{
    struct Source { WaterDefinition definition; MeshTransform transform; };
    std::optional<RiverConnection> Locate(const Source& tributary, int end, const Source& receiver,
                                          std::uint64_t receiverId, Vector3 position);
    bool Attach(Source& tributary, const Source& receiver);
    // Receiver before tributaries. One shared triangulation, partitioned into
    // separately owned meshes with identical vertices at every internal seam.
    std::optional<std::vector<LakeGeometry>> Build(std::span<const Source> sources);
}
