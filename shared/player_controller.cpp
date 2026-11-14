#include "player_controller.hpp"
#include "game_world.hpp"

PlayerController::~PlayerController()
{
    Release();
}

Pawn* PlayerController::Controlled() const
{
    return world.Find(controlled);
}

bool PlayerController::Possess(PawnHandle handle)
{
    Pawn* target = world.Find(handle);
    if (!target || target->health <= 0 || (target->controller && target->controller != this)) return false;
    if (target == Controlled()) return true;
    Release();
    controlled = handle;
    target->controller = this;
    target->input = {};
    camera.Possess(*target);
    return true;
}

void PlayerController::Release()
{
    if (auto* previous = Controlled())
    {
        previous->input = {};
        previous->controller = nullptr;
    }
    controlled = 0;
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
