#pragma once

#include "game_world.hpp"
#include "player.hpp"

class WorldRenderer;
class CookedWorld;

class GameSession
{
public:
    GameSession(SDL_Window* window, WorldDocument& document, CookedWorld* streaming = nullptr);
    bool Start();
    bool Handle(const SDL_Event& event);
    bool SetDebugCamera(bool enabled);
    bool ReleaseMouse();
    bool MouseCaptured() const { return input.IsCaptured(); }
    void Stream(Vector3 eye, WorldRenderer& renderer);
    void Update(double seconds);
    const CameraView& View() const { return started ? player.controller.camera.View() : loadingCamera; }
    void Submit(WorldRenderer& renderer) const;
    bool DrawHUD(SDL_Renderer* renderer) const;

private:
    WorldDocument& document;
    GameWorld world;
    Player player;
    GameplayInput input;
    MeshInstance testGround;
    double accumulator = 0;
    bool debugCamera = false;
    CookedWorld* streaming = nullptr;
    CameraView loadingCamera;
    bool started = false, collisionReady = false;
    std::size_t spawnIndex = 0;
    std::vector<Vector3> PhysicsSources() const;
    bool FinishStart();
    bool TryStartAt(Vector3 position, double yaw, bool authored = false);
};
