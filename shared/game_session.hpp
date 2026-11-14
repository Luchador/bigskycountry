#pragma once

#include "game_world.hpp"
#include "player.hpp"

class WorldRenderer;

class GameSession
{
public:
    GameSession(SDL_Window* window, WorldDocument& document);
    bool Start();
    bool Handle(const SDL_Event& event);
    bool SetDebugCamera(bool enabled);
    bool ReleaseMouse();
    bool MouseCaptured() const { return input.IsCaptured(); }
    void Update(double seconds);
    const CameraView& View() const { return player.controller.camera.View(); }
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
    bool TryStartAt(Vector3 position, double yaw);
};
