#include "gameplay_input.hpp"

void GameplayInput::Clear()
{
    held.fill(false);
    interact = jump = false;
    lookX = lookY = 0;
}

bool GameplayInput::SetCaptured(bool value)
{
    if (!SDL_SetWindowRelativeMouseMode(window, value)) return false;
    captured = value;
    Clear();
    return true;
}

bool GameplayInput::Handle(const SDL_Event& event)
{
    if (SDL_GetWindowFromEvent(&event) != window) return true;
    if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) return SetCaptured(false);
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT && !captured)
        return SetCaptured(true);
    if (!captured) return true;
    if (event.type == SDL_EVENT_MOUSE_MOTION)
    {
        lookX += event.motion.xrel;
        lookY += event.motion.yrel;
    }
    if (event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_KEY_UP)
    {
        const int key = event.key.scancode;
        if (key > SDL_SCANCODE_UNKNOWN && key < SDL_SCANCODE_COUNT)
            held[key] = event.type == SDL_EVENT_KEY_DOWN;
        if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat)
        {
            interact = interact || key == SDL_SCANCODE_F;
            jump = jump || key == SDL_SCANCODE_SPACE;
        }
    }
    return true;
}

GameplayActions GameplayInput::Read(PawnInputProfile profile)
{
    GameplayActions result;
    if (captured)
    {
        result.pawn.forward = int(held[SDL_SCANCODE_W]) - int(held[SDL_SCANCODE_S]);
        result.pawn.right = int(held[SDL_SCANCODE_D]) - int(held[SDL_SCANCODE_A]);
        result.pawn.sprint = profile == PawnInputProfile::OnFoot && (held[SDL_SCANCODE_LSHIFT] || held[SDL_SCANCODE_RSHIFT]);
        result.pawn.jump = profile == PawnInputProfile::OnFoot && jump;
        result.pawn.handbrake = profile == PawnInputProfile::Driving && held[SDL_SCANCODE_SPACE];
        result.lookX = lookX;
        result.lookY = lookY;
        result.interact = interact;
    }
    interact = jump = false;
    lookX = lookY = 0;
    return result;
}
