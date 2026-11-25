#pragma once
#include "world_package.hpp"
#include "streaming_settings.hpp"
#include <memory>
#include <span>

class WorldRenderer;
class WorldCollision;
class CameraView;
class Minimap;

// Main-thread residency controller. The worker owns only CPU decoding/BVH work;
// SDL GPU objects, scene publication and collision attachment stay on the main thread.
class CookedWorld
{
  public:
    CookedWorld();
    ~CookedWorld();
    CookedWorld(const CookedWorld &) = delete;
    CookedWorld &operator=(const CookedWorld &) = delete;
    bool Load(const WorldDocument &source);
    bool LoadPath(const std::string &source);
    bool Ready() const;
    void Update(Vector3 eye, std::span<const Vector3> physicsSources, WorldCollision &collision,
                WorldRenderer *renderer, double radius, const StreamingSettings &settings);
    bool CollisionReady(Vector3 position, double radius = 64) const;
    const std::vector<WorldPackage::Spawn> &Spawns() const;
    const TrafficNetwork& Traffic() const;
    void Submit(WorldRenderer &renderer, const CameraView &camera, double radius);
    void SubmitMinimap(Minimap& minimap) const;
    void SubmitLabels(WorldRenderer &renderer, const CameraView &camera);
    bool DrawStatistics(SDL_Renderer *renderer, float y = -1) const;
    // Conservative metadata check, including collision not yet resident.
    bool MayHaveGroundBelow(Vector3 position) const;
    const std::string &Error() const;
    struct Statistics
    {
        std::size_t totalCells = 0, nearCells = 0, distantCells = 0, collisionChunks = 0;
        std::size_t modelAssets = 0, modelInstances = 0;
        std::size_t geometryBytes = 0, modelBytes = 0, collisionBytes = 0;
        std::size_t completed = 0, discarded = 0, evicted = 0, queued = 0, failed = 0;
        bool pressure = false;
    };
    const Statistics &Stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
