#pragma once

#include "spatial_index.hpp"
#include <SDL3/SDL.h>
#include <array>

class WorldDocument;
class CookedWorld;
struct RoadGeometry;

// A heading-up map of roads, railroad embankments and water surfaces. It
// projects their footprints onto a tilted camera plane without world lighting.
class Minimap
{
public:
    enum class Layer { Water, Railroad, Road, Count }; // Draw order, bottom to top.
    static constexpr double radiusMeters = 600;
    bool Draw(SDL_Renderer* renderer, const WorldDocument& document, const CookedWorld* streaming,
              Vector3 position, Vector3 facing);
    bool BoundsNeeded(const Bounds3& bounds) const;
    void Submit(const MeshInstance& mesh, Layer layer);

private:
    struct Source
    {
        const RoadGeometry* geometry = nullptr;
        const MeshInstance* mesh = nullptr;
        Layer layer = Layer::Road;
        size_t first = 0, end = 0;
        std::array<Vector3, 3> Triangle(size_t i) const;
    };
    std::vector<Source> sources;
    SpatialIndex index;
    const WorldDocument* sourceDocument = nullptr;
    std::uint64_t sourceRevision = 0, waterRevision = 0;
    size_t roadCount = 0, junctionCount = 0, terrainCount = 0;
    std::array<std::vector<SDL_Vertex>, size_t(Layer::Count)> vertices;
    SDL_FRect rectangle{};
    Vector3 position{}, right{}, forward{0, 0, 1};
    double focal = 0;
    void Synchronize(const WorldDocument& document);
    void SubmitTriangle(const std::array<Vector3, 3>& triangle, Layer layer);
};
