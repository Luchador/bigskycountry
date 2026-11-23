#include "pawn_controller.hpp"
#include "game_world.hpp"

#include <algorithm>
#include <cmath>

PawnController::~PawnController() { Release(); }
Pawn* PawnController::Controlled() const { return world.Find(controlled); }
bool PawnController::Possess(PawnHandle handle)
{
    Pawn* target = world.Find(handle);
    if (!target || target->health <= 0 || (target->controller && target->controller != this)) return false;
    if (target == Controlled()) return true;
    Release();
    controlled = handle;
    target->controller = this;
    target->input = {};
    return true;
}
void PawnController::Release()
{
    if (auto* previous = Controlled())
    {
        previous->input = {};
        previous->controller = nullptr;
    }
    controlled = 0;
}
void PawnController::Command(PawnInput input)
{
    const auto axis = [](double x) { return std::isfinite(x) ? std::clamp(x, -1.0, 1.0) : 0.0; };
    input.forward = axis(input.forward);
    input.right = axis(input.right);
    input.brake = std::max(0.0, axis(input.brake));
    if (auto* pawn = Controlled()) pawn->input = input;
}
