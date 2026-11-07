#include "application.hpp"

#include <SDL3/SDL.h>

namespace
{
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

            if (!SDL_RenderClear(renderer) || !SDL_RenderPresent(renderer))
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
