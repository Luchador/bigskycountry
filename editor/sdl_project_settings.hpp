#pragma once

#include "sdl_transform_panel.hpp"
#include <array>

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
    void StoreField();
    std::array<std::string, 5> values;
    int activeField = -1;
    SDL_Renderer* renderer = nullptr;
    SdlTransformPanel distanceField;
    SDL_Window* window = nullptr; // The editor frame owns the utility window.
};
