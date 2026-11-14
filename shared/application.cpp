#include <SDL3/SDL.h>
#include <algorithm>

#include "world_renderer.hpp"
#include "sky.hpp"
#include "application.hpp"
#include "project_settings.hpp"
#include "free_camera.hpp"
#include "input.hpp"
#include "marker.hpp"
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
            SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Could not start the GPU renderer: %s", SDL_GetError());
        }
#endif
        return SDL_CreateRenderer(window, nullptr);
    }

    const MeshInstance* FirstVisiblePlacement(const WorldDocument& world)
    {
        if (!world.meshes.empty()) return &world.meshes.front();
        if (!world.Objects().empty()) return &world.Objects().front()->Instance();
        for (const auto& chunk : world.Terrain())
            if (chunk->Instance().mesh) return &chunk->Instance();
        return nullptr;
    }

    bool RunWindowLoop(SDL_Window* window, SDL_Renderer* renderer, ApplicationMode mode, WorldDocument& world, EditorTool* roadTool)
    {
        Input input(window);

        if (!input.SetCameraEnabled(mode == ApplicationMode::Editor))
        {
            return false;
        }

        Sky sky;
        WorldRenderer worldRenderer(world.models.Location());
        FreeCamera freeCamera;

        if (!world.roads.empty())
        {
            const Road& first = world.roads.front();
            const Vector3 focus = (first.points[0].position + first.points[1].position) * 0.5;
            const double distance = std::max(30.0, first.style.LeftWidth() + first.style.RightWidth());
            freeCamera.LookAt(focus + Vector3{0.0, distance, -distance}, focus);
        }
        else if (const auto* first = FirstVisiblePlacement(world))
        {
            Vector3 lo = first->transform.Point(first->mesh->vertices.front().position), hi = lo;
            for (const auto& vertex : first->mesh->vertices)
            {
                const Vector3 p = first->transform.Point(vertex.position);
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
            const Uint64 frameStart = SDL_GetTicksNS();
            input.BeginFrame();

            SDL_Event event;

            while (SDL_PollEvent(&event))
            {
                if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && SDL_GetWindowFromEvent(&event) == window)
                {
                    event.type = SDL_EVENT_QUIT;
                }
                if (roadTool)
                {
#ifdef _WIN32
                    // Returning focus from a native child control can leave an SDL keystroke
                    // without a window ID even though the editor frame has keyboard focus.
                    if ((event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_KEY_UP) &&
                        event.key.windowID == 0 && roadTool->ViewportFocused())
                    {
                        event.key.windowID = SDL_GetWindowID(window);
                    }
#endif
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

            if (roadTool)
            {
                world.RefreshRampConnections();
                world.RefreshTerrainFills();
            }
            const Uint64 now = SDL_GetTicksNS();
            const Uint64 elapsedNanoseconds = now - previousTime;
            previousTime = now;

            if (mode == ApplicationMode::Game)
            {
                g_WorldState.AdvanceTime(elapsedNanoseconds);
                world.TickObjects(static_cast<float>(elapsedNanoseconds / 1.0e9));
            }

            if (input.IsCameraEnabled() && (!roadTool || (roadTool->ViewportFocused() && !roadTool->DraggingGizmo())))
            {
                const double cameraElapsedSeconds = std::min(elapsedNanoseconds / 1.0e9, 0.1);
                freeCamera.Update(input.ReadCameraInput(mode), cameraElapsedSeconds);
            }

            const FreeCamera& view = input.IsCameraEnabled() ? freeCamera : gameView;

            if (roadTool && !roadTool->SetContentViewport(renderer))
            {
                return false;
            }

            if (!roadTool || !roadTool->IsPlanView())
            {
                if (!sky.Draw(renderer, view, g_WorldState.GetTimeOfDayMinutes()) || !worldRenderer.Begin(renderer, view, &sky.Lighting()))
                {
                    return false;
                }

                const auto hidden = [&](SceneKey key) { return roadTool && world.hierarchy.EditorHidden(key); };
                for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
                {
                    if (hidden({SceneKind::Road, i})) continue;
                    const Road& road = world.roads[i];
                    worldRenderer.SubmitRoad(road, road.style.material, roadTool && roadTool->GeometrySelected({SceneKind::Road, i}));
                }
                if (const Road* preview = roadTool ? roadTool->PreviewRoad() : nullptr)
                    worldRenderer.SubmitRoad(*preview, preview->style.material, false);
                for (const auto& [id, junction] : world.IntersectionSurfaces())
                {
                    if (hidden({SceneKind::Intersection, id})) continue;
                    worldRenderer.SubmitRoad(junction.geometry, world.intersections.at(id).material, roadTool && roadTool->GeometrySelected({SceneKind::Intersection, id}));
                }

                for (int i = 0; i < static_cast<int>(world.meshes.size()); ++i)
                    if (!hidden({SceneKind::Mesh, i})) worldRenderer.SubmitWorldMesh(world.meshes[i]);
                for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
                    if (!hidden({SceneKind::Terrain, i})) worldRenderer.SubmitTerrain(*world.Terrain()[i]);
                for (int i = 0; i < static_cast<int>(world.Objects().size()); ++i)
                {
                    if (hidden({SceneKind::Object, i})) continue;
                    const auto& object = world.Objects()[i];
                    if (const auto* marker = dynamic_cast<const Marker*>(object.get()))
                        worldRenderer.SubmitWorldMesh(marker->Instance(), marker->material);
                    else
                        worldRenderer.SubmitWorldMesh(object->Instance());
                }

                if (roadTool && roadTool->WireframeEnabled()) worldRenderer.WireframeSubmittedGeometry();
                worldRenderer.BeginUnlit();
                for (int i = 0; i < static_cast<int>(world.Objects().size()); ++i)
                {
                    if (hidden({SceneKind::Object, i})) continue;
                    if (const auto* marker = dynamic_cast<const Marker*>(world.Objects()[i].get()))
                        marker->SubmitLabel(worldRenderer, view);
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
                constexpr Uint64 frameBudget = SDL_NS_PER_SECOND / 60;
                const Uint64 workTime = SDL_GetTicksNS() - frameStart;
                if (workTime < frameBudget) SDL_DelayNS(frameBudget - workTime);
            }
        }
    }
}

int RunApplication(ApplicationMode mode, const char* worldFile, EditorTool* roadTool)
{
    const char* title = mode == ApplicationMode::Editor ? "Big Sky Country Editor" : "Big Sky Country";
    SDL_Window* window = nullptr;
    SDL_Window* renderWindow = nullptr;
    SDL_Renderer* renderer = nullptr;
    bool success = SDL_Init(SDL_INIT_VIDEO);

    if (success)
    {
        window = SDL_CreateWindow(title, 1920, 1080, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN);
        if (window)
        {
            renderWindow = roadTool ? roadTool->CreateRenderWindow(window) : window;
            if (renderWindow)
            {
                renderer = CreateSceneRenderer(renderWindow);
            }
        }
        success = renderer && SDL_ShowWindow(window);
    }

    if (success)
    {
        const auto location = mode == ApplicationMode::Editor ? AssetRoot::Project : AssetRoot::Cooked;
        if (!g_ProjectSettings.Load(location)) SDL_Log("Project Settings: %s", SDL_GetError());
        WorldDocument world(location);
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
    if (renderWindow && renderWindow != window)
    {
        SDL_DestroyWindow(renderWindow);
    }
    SDL_DestroyWindow(window);
    SDL_Quit();

    return success ? 0 : 1;
}
