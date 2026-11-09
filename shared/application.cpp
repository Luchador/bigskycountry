#include <SDL3/SDL.h>
#include <algorithm>

#include "world_renderer.hpp"
#include "sky.hpp"
#include "application.hpp"
#include "free_camera.hpp"
#include "input.hpp"
#include "world_state.hpp"
#include "world_document.hpp"

namespace
{
    SDL_Renderer* CreateSceneRenderer(SDL_Window* window)
    {
        // Prefer the GPU renderer when a driver hasn't been requested explicitly.
#ifdef BIGSKY_WORLD_GPU
        if (!SDL_GetHint(SDL_HINT_RENDER_DRIVER))
        {
            if (SDL_Renderer* renderer = SDL_CreateRenderer(window, "gpu"))
            {
                return renderer;
            }
        }
#endif
        return SDL_CreateRenderer(window, nullptr);
    }

    bool RunWindowLoop(SDL_Window* window, SDL_Renderer* renderer, ApplicationMode mode, WorldDocument& world, EditorTool* roadTool)
    {
        Input input(window);

        if (!input.SetCameraEnabled(mode == ApplicationMode::Editor))
        {
            return false;
        }

        Sky sky;
        WorldRenderer worldRenderer;
        FreeCamera freeCamera;

        if (!world.roads.empty())
        {
            const Road& first = world.roads.front();
            const Vector3 focus = (first.points[0].position + first.points[1].position) * 0.5;
            const double distance = std::max(30.0, first.style.LeftWidth() + first.style.RightWidth());
            freeCamera.LookAt(focus + Vector3{0.0, distance, -distance}, focus);
        }

        if (world.roads.empty() && !world.meshes.empty())
        {
            const auto& first = world.meshes.front();
            Vector3 lo = first.transform.Point(first.mesh->vertices.front().position), hi = lo;
            for (const auto& vertex : first.mesh->vertices)
            {
                const Vector3 p = first.transform.Point(vertex.position);
                lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
                hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
            }
            const Vector3 focus = (lo + hi) * 0.5;
            const double distance = std::max(1.0, Math::Length(hi - lo));
            freeCamera.LookAt(focus + Vector3{0, distance * 0.5, -distance}, focus);
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
                    const EditorEvent result = roadTool->HandleEvent(window, renderer, event, world, freeCamera);
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

                    const char* title = "Big Sky Country";

                    if (!SDL_SetWindowTitle(window, title))
                    {
                        return false;
                    }
                }

                // The editor consumes wheel events used for top-down map zoom.
                if (event.type == SDL_EVENT_MOUSE_WHEEL && event.wheel.windowID == SDL_GetWindowID(window) &&
                    (input.IsCameraEnabled() || roadTool))
                {
                    const float steps = event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -event.wheel.y : event.wheel.y;
                    freeCamera.ScrollSpeed(steps);
                    continue;
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

            if (input.IsCameraEnabled() && (!roadTool || !roadTool->DraggingGizmo()))
            {
                const double cameraElapsedSeconds = std::min(elapsedNanoseconds / 1.0e9, 0.1);
                // Ctrl+S belongs to File > Save in the editor.
                const bool allowBackward = !roadTool || !(SDL_GetModState() & SDL_KMOD_CTRL);
                const bool allowForwardAndUp = !roadTool || !roadTool->Selected(world) || SDL_GetWindowRelativeMouseMode(window);
                freeCamera.Update(input.ReadCameraInput(allowBackward, allowForwardAndUp), cameraElapsedSeconds);
            }

            const FreeCamera& view = input.IsCameraEnabled() ? freeCamera : gameView;

            if (roadTool && !roadTool->SetContentViewport(renderer))
            {
                return false;
            }

            if (!roadTool || !roadTool->IsPlanView())
            {
                if (!sky.Draw(renderer, view, g_WorldState.GetTimeOfDayMinutes()) || !worldRenderer.Begin(renderer, view))
                {
                    return false;
                }

                const Road* selected = roadTool ? roadTool->Selected(world) : nullptr;

                for (const Road& road : world.roads)
                {
                    worldRenderer.SubmitRoad(road, &road == selected);
                }

                for (const MeshInstance& instance : world.meshes)
                {
                    worldRenderer.SubmitMesh(instance);
                }

                if (roadTool)
                {
                    roadTool->SubmitGizmos(worldRenderer, world, view);
                }

                if (!worldRenderer.End())
                {
                    return false;
                }
            }

            if ((roadTool && !roadTool->Render(renderer, world, freeCamera)) || !SDL_RenderPresent(renderer))
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

int RunApplication(ApplicationMode mode, const char* worldFile, EditorTool* roadTool)
{
    const char* title = mode == ApplicationMode::Editor ? "Big Sky Country Editor" : "Big Sky Country";
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    bool success = SDL_Init(SDL_INIT_VIDEO);

    if (success)
    {
        window = SDL_CreateWindow(title, 1920, 1080, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN);
        if (window)
        {
            renderer = CreateSceneRenderer(window);
        }
        success = renderer && SDL_ShowWindow(window);
    }

    if (success)
    {
        WorldDocument world;
        success = world.Open(worldFile) && (!roadTool || roadTool->Initialize(window, world.models)) &&
                  RunWindowLoop(window, renderer, mode, world, roadTool);
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
