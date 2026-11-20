#pragma once

#include "spatial_index.hpp"

class WorldDocument;
class WorldRenderer;
class CameraView;

// Milestone 1 indexes already-loaded document geometry. It never changes
// ownership, simulation, collision, editor selection, or serialized world data.
class WorldVisibility
{
public:
    void Synchronize(const WorldDocument& world);
    std::span<const SceneKey> Query(const WorldDocument& world, const CameraView& camera,
                                  const WorldRenderer& renderer, double distance, bool editor);
    bool DrawStatistics(SDL_Renderer* renderer, double distance, float y = -1) const;
    const SpatialIndex& Index() const { return index; }

private:
    struct MeshBoundsCache
    {
        Bounds3 local, world;
        MeshTransform transform;
        bool valid = false;
        Bounds3 Get(const MeshInstance& instance);
    };
    SpatialIndex index;
    std::vector<MeshBoundsCache> meshes, terrain, water, objects;
    std::vector<SceneKey> visible;
    size_t hidden = 0, outsideFrustum = 0;
    double syncMilliseconds = 0, queryMilliseconds = 0;
};
