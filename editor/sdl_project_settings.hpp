#pragma once

#include <SDL3/SDL.h>

class SdlProjectSettings
{
public:
    bool Open(SDL_Window* owner);
    bool Handle(const SDL_Event& event);
    bool Focused() const { return window && SDL_GetKeyboardFocus() == window; }

private:
    bool Paint() const;
    SDL_Window* window = nullptr; // The editor frame owns the utility window.
};
