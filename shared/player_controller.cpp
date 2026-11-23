#include "player_controller.hpp"
#include "game_world.hpp"

bool PlayerController::Possess(PawnHandle handle)
{
    const bool changed = !Controlled() || Controlled()->Handle() != handle;
    if (!PawnController::Possess(handle)) return false;
    if (changed) camera.Possess(*Controlled());
    return true;
}

void PlayerController::Input(const GameplayActions& actions)
{
    camera.Look(actions.lookX, actions.lookY);
    if (auto* pawn = Controlled())
    {
        const bool pendingJump = pawn->input.jump;
        pawn->input = actions.pawn;
        // Preserve an edge across render frames that have no simulation step.
        pawn->input.jump = pawn->input.jump || pendingJump;
        pawn->input.yaw = camera.Yaw();
    }
}

void PlayerController::StopInput()
{
    if (auto* pawn = Controlled())
    {
        pawn->input = {};
        pawn->input.yaw = camera.Yaw();
    }
}

void PlayerController::ConsumePresses()
{
    if (auto* pawn = Controlled()) pawn->input.jump = false;
}
