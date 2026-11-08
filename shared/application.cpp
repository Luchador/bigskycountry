#include <SDL3/SDL.h>
#include <algorithm>

#include "road_render.hpp"
#include "sky.hpp"
#include "application.hpp"
#include "free_camera.hpp"
#include "input.hpp"
#include "world_state.hpp"

namespace
{
    bool DrawScene(SDL_Renderer* renderer, const FreeCamera& view, const RoadMap& roads, Sky& sky)
    {
        return sky.Draw(renderer, view, g_WorldState.GetTimeOfDayMinutes()) &&
               DrawWorldRoads(renderer, view, roads);
    }

    bool RunWindowLoop(SDL_Window* window, SDL_Renderer* renderer, ApplicationMode mode,
                       RoadMap& roads, EditorTool* roadTool)
    {
        Input input(window);

        if (!input.SetCameraEnabled(mode == ApplicationMode::Editor))
        {
            return false;
        }

        Sky sky;
        FreeCamera freeCamera;
        if (!roads.roads.empty())
        {
            const Road& first = roads.roads.front();
            const Vector3 focus = (first.points[0].position + first.points[1].position) * 0.5;
            const double distance = std::max(30.0, first.style.LeftWidth() + first.style.RightWidth());
            freeCamera.LookAt(focus + Vector3{0.0, distance, -distance}, focus);
        }
        const FreeCamera gameView = freeCamera;
        Uint64 previousTime = SDL_GetTicksNS();
        const bool vsyncEnabled = SDL_SetRenderVSync(renderer, 1);

        while (true)
        {
            input.BeginFrame();

            SDL_Event event;

            while (SDL_PollEvent(&event))
            {
                if (roadTool)
                {
                    const bool wasPlanView = roadTool->IsPlanView();
                    const EditorEvent result = roadTool->HandleEvent(window, renderer, event, roads, freeCamera);
                    if (result == EditorEvent::Error)
                    {
                        return false;
                    }
                    if (wasPlanView != roadTool->IsPlanView() && !input.SetCameraEnabled(!roadTool->IsPlanView()))
                    {
                        return false;
                    }
                    if (result == EditorEvent::Handled)
                    {
                        continue;
                    }
                }

                if (event.type == SDL_EVENT_QUIT ||
                    (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE && !event.key.repeat))
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
            const Uint64 elapsedNanoseconds = now - previousTime;
            previousTime = now;

            if (mode == ApplicationMode::Game)
            {
                g_WorldState.AdvanceTime(elapsedNanoseconds);
            }

            if (input.IsCameraEnabled())
            {
                const double cameraElapsedSeconds = std::min(elapsedNanoseconds / 1.0e9, 0.1);
                // Ctrl+S belongs to File > Save in the editor.
                const bool allowBackward = !roadTool || !(SDL_GetModState() & SDL_KMOD_CTRL);
                freeCamera.Update(input.ReadCameraInput(allowBackward), cameraElapsedSeconds);
            }

            const FreeCamera& view = input.IsCameraEnabled() ? freeCamera : gameView;

            if ((!roadTool || !roadTool->IsPlanView()) &&
                !DrawScene(renderer, view, roads, sky))
            {
                return false;
            }
            if ((roadTool && !roadTool->Render(renderer, roads, freeCamera)) || !SDL_RenderPresent(renderer))
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

int RunApplication(ApplicationMode mode, const char* roadFile, EditorTool* roadTool)
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
        RoadMap roads;
        success = roads.Open(roadFile) && (!roadTool || roadTool->Initialize(window)) &&
                  RunWindowLoop(window, renderer, mode, roads, roadTool);
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
