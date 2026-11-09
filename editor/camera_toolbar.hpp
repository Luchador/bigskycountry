#pragma once

#include "transform_space.hpp"
#include <SDL3/SDL.h>
#include <array>

class FreeCamera;

class CameraToolbar
{
public:
    static constexpr int height = 36;

    void LoadIcons(SDL_Renderer* renderer);
    bool Draw(SDL_Renderer* renderer, int width, const FreeCamera& camera, GizmoSpace space) const;
    // Pointer coordinates are relative to the content viewport below the bar.
    bool HandlePointer(const SDL_Event& event, int width, FreeCamera& camera, GizmoSpace& space);
    void CancelDrag() { dragging = spacePressed = false; }

private:
    std::array<SDL_Texture*, 2> icons{}; // Owned by the window renderer until shutdown.
    bool spacePressed = false;
    bool dragging = false;
    float grabOffset = 0.0f;
};
