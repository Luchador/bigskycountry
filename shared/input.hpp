#pragma once

#include <SDL3/SDL.h>

struct CameraInput
{
    int forward = 0;
    int right = 0;
    int up = 0;
    float lookX = 0.0f;
    float lookY = 0.0f;
};

class Input
{
public:
    explicit Input(SDL_Window* window);

    void BeginFrame();
    bool HandleEvent(const SDL_Event& event);
    CameraInput ReadCameraInput(bool allowBackward = true, bool allowForwardAndUp = true) const;

    bool SetCameraEnabled(bool enabled);
    bool IsCameraEnabled() const;

private:
    SDL_Window* window;
    bool cameraEnabled = false;
    float lookX = 0.0f;
    float lookY = 0.0f;
};
