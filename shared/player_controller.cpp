#include "player_controller.hpp"
#include "game_world.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double keyboardSteeringTapStrength = 0.15;
    constexpr double keyboardSteeringRiseSeconds = 0.5;
}

bool PlayerController::Possess(PawnHandle handle)
{
    const bool changed = !Controlled() || Controlled()->Handle() != handle;
    if (!PawnController::Possess(handle)) return false;
    if (changed)
    {
        camera.Possess(*Controlled());
        StopInput();
    }
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
        const double direction = pawn->InputProfile() == PawnInputProfile::Driving ? actions.pawn.right : 0;
        if (direction != steeringDirection || direction == 0) steeringHeldSeconds = 0;
        steeringDirection = direction;
        if (pawn->InputProfile() == PawnInputProfile::Driving) pawn->input.right = SteeringInput();
    }
}

double PlayerController::SteeringInput() const
{
    const double t = std::clamp(steeringHeldSeconds / keyboardSteeringRiseSeconds, 0.0, 1.0);
    const double strength = std::lerp(keyboardSteeringTapStrength, 1.0, t * t * (3.0 - 2.0 * t));
    return steeringDirection * strength;
}

void PlayerController::Update(double seconds)
{
    if (auto* pawn = Controlled(); pawn && pawn->InputProfile() == PawnInputProfile::Driving)
    {
        // Advance once per simulation tick, independent of render frequency or
        // the car's physics substeps. Key-up clears the request in Input().
        if (steeringDirection != 0 && std::isfinite(seconds) && seconds > 0)
            steeringHeldSeconds = std::min(keyboardSteeringRiseSeconds, steeringHeldSeconds + seconds);
        pawn->input.right = SteeringInput();
    }
}

void PlayerController::StopInput()
{
    steeringDirection = steeringHeldSeconds = 0;
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
