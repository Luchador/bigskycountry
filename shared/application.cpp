#include <SDL3/SDL.h>
#include <algorithm>

#include "application.hpp"
#include "free_camera.hpp"
#include "input.hpp"

namespace
{
    bool DrawTriangle(SDL_Renderer* renderer, const FreeCamera& camera)
    {
        int width;
        int height;
    
        if (!SDL_GetCurrentRenderOutputSize(renderer, &width, &height))
        {
            return false;
        }

        // A camera-facing triangle, four meters in front of the starting camera.
        const Vector3 center = camera.WorldToView({ 0.0, 0.0, 4.0 });

        if (center.z < 0.1)
        {
            return true;
        }

        const double pixelsPerMeter = 0.5 * std::min(width, height) / center.z;
        const double x = width * 0.5 + center.x * pixelsPerMeter;
        const double y = height * 0.5 - center.y * pixelsPerMeter;
        const double halfExtent = 2.0 * pixelsPerMeter;

        if (x + halfExtent < 0.0 || x - halfExtent > width || y + halfExtent < 0.0 || y - halfExtent > height)
        {
            return true;
        }

        const float centerX = static_cast<float>(x);
        const float centerY = static_cast<float>(y);
        const float halfSize = static_cast<float>(halfExtent);

        const SDL_Vertex vertices[] =
        {
            { { centerX, centerY - halfSize },            { 1.0f, 0.0f, 0.0f, 1.0f }, {} },
            { { centerX - halfSize, centerY + halfSize }, { 0.0f, 1.0f, 0.0f, 1.0f }, {} },
            { { centerX + halfSize, centerY + halfSize }, { 1.0f, 1.0f, 1.0f, 1.0f }, {} },
        };

        return SDL_RenderGeometry(renderer, nullptr, vertices, 3, nullptr, 0);
    }

    bool RunWindowLoop(SDL_Window* window, SDL_Renderer* renderer, ApplicationMode mode)
    {
        Input input(window);

        if (!input.SetCameraEnabled(mode == ApplicationMode::Editor))
        {
            return false;
        }

        FreeCamera freeCamera;
        const FreeCamera gameView;
        Uint64 previousTime = SDL_GetTicksNS();
        const bool vsyncEnabled = SDL_SetRenderVSync(renderer, 1);

        if (!SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255))
        {
            return false;
        }

        while (true)
        {
            input.BeginFrame();

            SDL_Event event;

            while (SDL_PollEvent(&event))
            {
                if (event.type == SDL_EVENT_QUIT || (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE))
                {
                    return true;
                }

                if (mode == ApplicationMode::Game && event.type == SDL_EVENT_KEY_DOWN && event.key.scancode == SDL_SCANCODE_F1 && !event.key.repeat)
                {
                    if (!input.SetCameraEnabled(!input.IsCameraEnabled()))
                    {
                        return false;
                    }

                    const char* title = input.IsCameraEnabled() ? "Big Sky Country (Debug Camera)" : "Big Sky Country";

                    if (!SDL_SetWindowTitle(window, title))
                    {
                        return false;
                    }
                }

                if (!input.HandleEvent(event))
                {
                    return false;
                }
            }

            const Uint64 now = SDL_GetTicksNS();
            const double elapsedSeconds = std::min((now - previousTime) / 1.0e9, 0.1);

            previousTime = now;

            if (input.IsCameraEnabled())
            {
                freeCamera.Update(input.ReadCameraInput(), elapsedSeconds);
            }

            const FreeCamera& view = input.IsCameraEnabled() ? freeCamera : gameView;

            if (!SDL_RenderClear(renderer) || !DrawTriangle(renderer, view) || !SDL_RenderPresent(renderer))
            {
                return false;
            }

            if (!vsyncEnabled)
            {
                SDL_Delay(16);
            }
        }
    }
}

int RunApplication(ApplicationMode mode)
{
    const char* title = mode == ApplicationMode::Editor ? "Big Sky Country Editor" : "Big Sky Country";
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    bool success = SDL_Init(SDL_INIT_VIDEO);

    if (success)
    {
        success = SDL_CreateWindowAndRenderer(title, 1920, 1080, SDL_WINDOW_RESIZABLE, &window, &renderer);
    }

    if (success)
    {
        success = RunWindowLoop(window, renderer, mode);
    }

    if (!success)
    {
        const char* error = SDL_GetError();

        SDL_Log("%s: %s", title, error);
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, title, error, window);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return success ? 0 : 1;
}
