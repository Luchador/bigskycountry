#pragma once

#include "road_render.hpp"
#include <memory>

class TerrainChunk;

// Retained, unlit geometry for the editor's orthographic road view. Pan/zoom
// changes shader uniforms; road edits replace only that road's cached batches.
class PlanRenderer
{
public:
    PlanRenderer();
    ~PlanRenderer();
    PlanRenderer(const PlanRenderer&) = delete;
    PlanRenderer& operator=(const PlanRenderer&) = delete;
    bool Begin(SDL_Renderer* renderer, Vector3 center, double scale, SDL_Rect viewport);
    void Road(const RoadGeometry& road, RoadLayer layer);
    void Terrain(const TerrainChunk& chunk);
    bool End();
    void Clear(); // Release geometry when leaving the 2D view.

    struct Statistics
    {
        std::size_t rebuiltBatches = 0, uploadedBytes = 0, projectedVertices = 0, drawCalls = 0;
    };
    const Statistics& Stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
