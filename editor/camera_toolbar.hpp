#pragma once

#include <SDL3/SDL.h>

class FreeCamera;

class CameraToolbar
{
public:
    static constexpr int height = 36;

    bool Draw(SDL_Renderer* renderer, int width, const FreeCamera& camera) const;
    // Pointer coordinates are relative to the content viewport below the bar.
    bool HandlePointer(const SDL_Event& event, int width, FreeCamera& camera);
    void CancelDrag() { dragging = false; }

private:
    bool dragging = false;
    float grabOffset = 0.0f;
};
