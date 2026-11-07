#include <SDL3/SDL.h>
#include <algorithm>

#include "road_render.hpp"
#include "application.hpp"
#include "free_camera.hpp"
#include "input.hpp"
#include "world_state.hpp"

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

    constexpr SDL_FRect roadButton{12, 12, 150, 28};

    bool OpenRoadTool(const SDL_Event& event, SDL_Renderer* renderer)
    {
        if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_R && !event.key.repeat)
        {
            return true;
        }
        if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT)
        {
            SDL_FPoint position{};
            return SDL_RenderCoordinatesFromWindow(renderer, event.button.x, event.button.y, &position.x, &position.y) &&
                   SDL_PointInRectFloat(&position, &roadButton);
        }
        return false;
    }

    bool DrawScene(SDL_Renderer* renderer, const FreeCamera& view, const RoadMap& roads, bool editor)
    {
        if (!SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255) || !SDL_RenderClear(renderer) ||
            !DrawWorldRoads(renderer, view, roads) || !DrawTriangle(renderer, view))
        {
            return false;
        }
        if (editor && (!SDL_SetRenderDrawColor(renderer, 32, 43, 47, 255) || !SDL_RenderFillRect(renderer, &roadButton) ||
            !SDL_SetRenderDrawColor(renderer, 230, 235, 238, 255) || !SDL_RenderDebugText(renderer, 23, 22, "Road editor  [R]")))
        {
            return false;
        }
        return SDL_RenderPresent(renderer);
    }

    bool RunWindowLoop(SDL_Window* window, SDL_Renderer* renderer, ApplicationMode mode,
                       RoadMap& roads, EditorTool* roadTool)
    {
        Input input(window);

        if (!input.SetCameraEnabled(mode == ApplicationMode::Editor))
        {
            return false;
        }

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
                if (event.type == SDL_EVENT_QUIT || (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE))
                {
                    return true;
                }

                if (roadTool && OpenRoadTool(event, renderer))
                {
                    if (!input.SetCameraEnabled(false))
                    {
                        return false;
                    }
                    const ToolResult result = roadTool->Run(window, renderer, roads, freeCamera);
                    if (result != ToolResult::Closed)
                    {
                        return result == ToolResult::Quit;
                    }
                    if (!input.SetCameraEnabled(true))
                    {
                        return false;
                    }
                    previousTime = SDL_GetTicksNS();
                    continue;
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
                freeCamera.Update(input.ReadCameraInput(), cameraElapsedSeconds);
            }

            const FreeCamera& view = input.IsCameraEnabled() ? freeCamera : gameView;

            if (!DrawScene(renderer, view, roads, roadTool != nullptr))
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
        success = roads.Open(roadFile) && RunWindowLoop(window, renderer, mode, roads, roadTool);
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
