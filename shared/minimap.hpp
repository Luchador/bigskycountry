#pragma once

#include "spatial_index.hpp"
#include <SDL3/SDL.h>
#include <array>

class WorldDocument;
class CookedWorld;
struct RoadGeometry;

// A heading-up pavement map. It projects the road footprint onto a tilted
// camera plane; world lighting, textures, terrain and traffic are not rendered.
class Minimap
{
public:
    static constexpr double radiusMeters = 600;
    bool Draw(SDL_Renderer* renderer, const WorldDocument& document, const CookedWorld* streaming,
              Vector3 position, Vector3 facing);
    bool BoundsNeeded(const Bounds3& bounds) const;
    void Submit(const MeshInstance& mesh);

private:
    struct Source { const RoadGeometry* geometry; size_t first, end; };
    std::vector<Source> sources;
    SpatialIndex index;
    const WorldDocument* sourceDocument = nullptr;
    std::uint64_t sourceRevision = 0;
    size_t roadCount = 0, junctionCount = 0;
    std::vector<SDL_Vertex> vertices;
    SDL_FRect rectangle{};
    Vector3 position{}, right{}, forward{0, 0, 1};
    double focal = 0;
    void Synchronize(const WorldDocument& document);
    void SubmitTriangle(const std::array<Vector3, 3>& triangle);
};
