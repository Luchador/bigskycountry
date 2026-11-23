#pragma once

#include "game_world.hpp"
#include "player.hpp"
#include "traffic_driver.hpp"
#include "traffic_junctions.hpp"
#include "traffic_population.hpp"

class WorldRenderer;
class CookedWorld;

class GameSession
{
public:
    GameSession(SDL_Window* window, WorldDocument& document, CookedWorld* streaming = nullptr);
    bool Start();
    bool Handle(const SDL_Event& event);
    bool HandleTrafficKey(const SDL_Event& event, const CameraView& view);
    bool DrawTrafficDebug(SDL_Renderer* renderer, const CameraView& view) const;
    bool SetDebugCamera(bool enabled);
    bool ReleaseMouse();
    bool MouseCaptured() const { return input.IsCaptured(); }
    void Stream(Vector3 eye, WorldRenderer& renderer);
    void Update(double seconds, const CameraView* externalView = nullptr);
    const CameraView& View() const { return started ? player.controller.camera.View() : loadingCamera; }
    void Submit(WorldRenderer& renderer) const;
    bool DrawHUD(SDL_Renderer* renderer, double framesPerSecond) const;
    void SetProfiling(bool enabled) { world.profiling = enabled; }
    const SimulationProfile& Profile() const { return profile; }

private:
    WorldDocument& document;
    GameWorld world;
    Player player;
    GameplayInput input;
    MeshInstance testGround;
    double accumulator = 0;
    SimulationProfile profile;
    bool debugCamera = false;
    CookedWorld* streaming = nullptr;
    CameraView loadingCamera;
    bool started = false, collisionReady = false;
    std::size_t spawnIndex = 0;
    std::vector<Vector3> PhysicsSources(bool includeTraffic = true) const;
    TrafficNetwork traffic;
    static constexpr std::size_t maxTrafficCars = 64;
    std::vector<std::unique_ptr<TrafficDriver>> trafficDrivers;
    std::unique_ptr<TrafficJunctions> junctionTraffic;
    std::unique_ptr<TrafficPopulation> population;
    std::optional<TrafficPopulation::Site> pendingSpawn;
    bool automaticTraffic = true;
    double spawnDelay = 0, spawnWait = 0;
    bool trafficDebug = false;
    std::string trafficNotice;
    const TrafficNetwork& Traffic() const;
    void SpawnTrafficTest(const CameraView& view, int count = 1);
    bool SpawnTrafficCar(const TrafficLane& lane, double start, bool automatic = false);
    void UpdateTrafficPopulation(double seconds, const CameraView& view);
    void RemoveTrafficTest();
    void RemoveExpiredTraffic();
    bool FinishStart();
    bool TryStartAt(Vector3 position, double yaw, bool authored = false);
};
