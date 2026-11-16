#pragma once
#include "world_package.hpp"
#include "spatial_index.hpp"
class WorldRenderer;
class CameraView;
// Milestone 2 preview: eager CPU payload loading, lazy renderer uploads.
// Reader supports independent cells; asynchronous residency is the next milestone.
class CookedWorld
{
public:
    bool Load(const WorldDocument& source);
    bool Ready() const { return ready; }
    void Submit(WorldRenderer& renderer, const CameraView& camera, double radius);
private:
    bool ready = false;
    std::vector<WorldPackage::Cell> cells;
    SpatialIndex index;
};
