#include "application.hpp"

#include <SDL3/SDL.h>
#include <algorithm>

namespace
{
    bool DrawTriangle(SDL_Renderer* renderer)
    {
        int width;
        int height;
        if (!SDL_GetCurrentRenderOutputSize(renderer, &width, &height))
        {
            return false;
        }

        const float centerX = width * 0.5f;
        const float centerY = height * 0.5f;
        const float halfSize = std::min(width, height) * 0.25f;

        const SDL_Vertex vertices[] =
        {
            { { centerX, centerY - halfSize },            { 1.0f, 0.0f, 0.0f, 1.0f }, {} },
            { { centerX - halfSize, centerY + halfSize }, { 0.0f, 1.0f, 0.0f, 1.0f }, {} },
            { { centerX + halfSize, centerY + halfSize }, { 1.0f, 1.0f, 1.0f, 1.0f }, {} },
        };

        return SDL_RenderGeometry(renderer, nullptr, vertices, 3, nullptr, 0);
    }

    bool RunWindowLoop(SDL_Renderer* renderer)
    {
        const bool vsyncEnabled = SDL_SetRenderVSync(renderer, 1);
        if (!SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255))
        {
            return false;
        }

        while (true)
        {
            SDL_Event event;
            while (SDL_PollEvent(&event))
            {
                if (event.type == SDL_EVENT_QUIT ||
                    (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE))
                {
                    return true;
                }
            }

            if (!SDL_RenderClear(renderer) || !DrawTriangle(renderer) || !SDL_RenderPresent(renderer))
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

int RunApplication(const char* title)
{
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    bool success = SDL_Init(SDL_INIT_VIDEO);

    if (success)
    {
        success = SDL_CreateWindowAndRenderer(title, 1920, 1080, SDL_WINDOW_RESIZABLE, &window, &renderer);
    }

    if (success)
    {
        success = RunWindowLoop(renderer);
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
