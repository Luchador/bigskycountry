#pragma once

#include "sdl_transform_panel.hpp"

class SdlProjectSettings
{
public:
    bool Open(SDL_Window* owner);
    bool Handle(const SDL_Event& event);
    bool Focused() const { return window && SDL_GetKeyboardFocus() == window; }

private:
    bool Paint() const;
    void Close();
    void Apply();
    SDL_Renderer* renderer = nullptr;
    SdlTransformPanel distanceField;
    SDL_Window* window = nullptr; // The editor frame owns the utility window.
};
