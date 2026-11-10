#pragma once

#include "transform_settings.hpp"
#include <SDL3/SDL.h>
#include <array>

class FreeCamera;

class CameraToolbar
{
public:
    static constexpr int height = 36;

    void LoadIcons(SDL_Renderer* renderer);
    bool Draw(SDL_Renderer* renderer, int width, const FreeCamera& camera, const TransformSettings& settings) const;
    // Pointer coordinates are relative to the content viewport below the bar.
    bool HandlePointer(const SDL_Event& event, int width, FreeCamera& camera, TransformSettings& settings);
    void CancelDrag() { dragging = false; pressed = Button::None; }

private:
    enum class Button { None, Snap, Space };
    std::array<SDL_Texture*, transformIcons.size()> icons{}; // Owned by the window renderer until shutdown.
    Button pressed = Button::None;
    bool dragging = false;
    float grabOffset = 0.0f;
};
