#pragma once

#include "pawn.hpp"
#include <array>

struct GameplayActions
{
    PawnInput pawn;
    double lookX = 0, lookY = 0;
    bool interact = false;
};

class GameplayInput
{
public:
    explicit GameplayInput(SDL_Window* window) : window(window) {}
    bool SetCaptured(bool value);
    bool IsCaptured() const { return captured; }
    bool Handle(const SDL_Event& event);
    GameplayActions Read(PawnInputProfile profile);
    void Clear();

private:
    SDL_Window* window;
    bool captured = false, interact = false, jump = false;
    double lookX = 0, lookY = 0;
    std::array<bool, SDL_SCANCODE_COUNT> held{};
};
