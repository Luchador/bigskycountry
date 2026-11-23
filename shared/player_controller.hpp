#pragma once

#include "gameplay_input.hpp"
#include "pawn_controller.hpp"
#include "player_camera.hpp"

class GameWorld;

class PlayerController : public PawnController
{
public:
    explicit PlayerController(GameWorld& world) : PawnController(world) {}
    PlayerController(const PlayerController&) = delete;
    PlayerController& operator=(const PlayerController&) = delete;
    bool Possess(PawnHandle handle) override;
    void Input(const GameplayActions& actions);
    void Update(double seconds) override;
    void StopInput();
    void ConsumePresses();
    PlayerCamera camera;

private:
    double SteeringInput() const;
    double steeringDirection = 0, steeringHeldSeconds = 0;
};
