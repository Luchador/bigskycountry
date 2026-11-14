#pragma once

#include "gameplay_input.hpp"
#include "player_camera.hpp"

class GameWorld;

class PlayerController
{
public:
    explicit PlayerController(GameWorld& world) : world(world) {}
    ~PlayerController();
    PlayerController(const PlayerController&) = delete;
    PlayerController& operator=(const PlayerController&) = delete;
    bool Possess(PawnHandle handle);
    void Release();
    Pawn* Controlled() const;
    void Input(const GameplayActions& actions);
    void StopInput();
    void ConsumePresses();
    PlayerCamera camera;

private:
    GameWorld& world;
    PawnHandle controlled = 0;
};
