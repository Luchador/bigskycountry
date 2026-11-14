#include "input.hpp"
#include "application.hpp"

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

bool Input::CoalesceMouseMotion(SDL_Event& event)
{
    if (event.type != SDL_EVENT_MOUSE_MOTION) return true;
    const int queued = SDL_PeepEvents(nullptr, 0, SDL_PEEKEVENT, SDL_EVENT_FIRST, SDL_EVENT_LAST);
    if (queued < 0) return false;
    SDL_Event next;
    for (int i = 0; i < queued; ++i)
    {
        const int count = SDL_PeepEvents(&next, 1, SDL_PEEKEVENT, SDL_EVENT_FIRST, SDL_EVENT_LAST);
        if (count < 0) return false;
        if (!count || next.type != SDL_EVENT_MOUSE_MOTION || next.motion.windowID != event.motion.windowID ||
            next.motion.which != event.motion.which || next.motion.state != event.motion.state) return true;
        // Leave SDL's internal poll sentinel in place so this frame's event
        // loop still ends. Bound the batch to avoid chasing new input forever.
        if (SDL_PeepEvents(&next, 1, SDL_GETEVENT, SDL_EVENT_MOUSE_MOTION, SDL_EVENT_MOUSE_MOTION) != 1) return false;
        next.motion.xrel += event.motion.xrel;
        next.motion.yrel += event.motion.yrel;
        event = next;
    }
    return true;
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

CameraInput Input::ReadCameraInput(ApplicationMode mode) const
{
    CameraInput input;

    if (!cameraEnabled || SDL_GetKeyboardFocus() != window)
    {
        return input;
    }

    input.lookX = lookX;
    input.lookY = lookY;

    if (mode == ApplicationMode::Editor && !SDL_GetWindowRelativeMouseMode(window))
    {
        return input;
    }

    const bool* keys = SDL_GetKeyboardState(nullptr);
    // Ctrl+S belongs to File > Save in the editor.
    const bool allowBackward = mode != ApplicationMode::Editor || !(SDL_GetModState() & SDL_KMOD_CTRL);

    // Opposing keys cancel each other out.
    input.forward = keys[SDL_SCANCODE_W] - (allowBackward && keys[SDL_SCANCODE_S]);
    input.right = keys[SDL_SCANCODE_D] - keys[SDL_SCANCODE_A];
    input.up = keys[SDL_SCANCODE_E] - keys[SDL_SCANCODE_Q];

    return input;
}
