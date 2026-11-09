#include "input.hpp"

// Constructor
Input::Input(SDL_Window* window) : window(window)
{
}

// Reset look deltas at the start of every frame.
void Input::BeginFrame()
{
    lookX = 0.0f;
    lookY = 0.0f;
}

bool Input::SetCameraEnabled(bool enabled)
{
    if (!enabled && SDL_GetWindowRelativeMouseMode(window) && !SDL_SetWindowRelativeMouseMode(window, false))
    {
        return false;
    }

    cameraEnabled = enabled;
    BeginFrame();

    return true;
}

bool Input::IsCameraEnabled() const
{
    return cameraEnabled;
}

bool Input::HandleEvent(const SDL_Event& event)
{
    if (!cameraEnabled || SDL_GetWindowFromEvent(&event) != window)
    {
        return true;
    }

    switch (event.type)
    {
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            BeginFrame();
            return SDL_SetWindowRelativeMouseMode(window, false);

        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event.button.button == SDL_BUTTON_RIGHT)
            {
                const bool pressed = event.type == SDL_EVENT_MOUSE_BUTTON_DOWN;

                if (pressed && SDL_GetKeyboardFocus() != window)
                {
                    return true;
                }

                return SDL_SetWindowRelativeMouseMode(window, pressed);
            }
            break;

        case SDL_EVENT_MOUSE_MOTION:
            if (SDL_GetWindowRelativeMouseMode(window))
            {
                lookX += event.motion.xrel;
                lookY += event.motion.yrel;
            }
            break;
    }

    return true;
}

CameraInput Input::ReadCameraInput(bool allowBackward, bool allowForwardAndUp) const
{
    CameraInput input;

    if (!cameraEnabled || SDL_GetKeyboardFocus() != window)
    {
        return input;
    }

    const bool* keys = SDL_GetKeyboardState(nullptr);

    // Opposing keys cancel each other out.
    input.forward = (allowForwardAndUp && keys[SDL_SCANCODE_W]) - (allowBackward && keys[SDL_SCANCODE_S]);
    input.right = keys[SDL_SCANCODE_D] - keys[SDL_SCANCODE_A];
    input.up = (allowForwardAndUp && keys[SDL_SCANCODE_E]) - keys[SDL_SCANCODE_Q];

    input.lookX = lookX;
    input.lookY = lookY;

    return input;
}
