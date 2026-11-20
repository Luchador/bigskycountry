#include <SDL3/SDL.h>
#include <algorithm>

#include "world_renderer.hpp"
#include "sky.hpp"
#include "application.hpp"
#include "project_settings.hpp"
#include "free_camera.hpp"
#include "input.hpp"
#include "game_session.hpp"
#include "marker.hpp"
#include "world_state.hpp"
#include "world_document.hpp"
#include "world_visibility.hpp"
#include "cooked_world.hpp"

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

    bool RunWindowLoop(SDL_Window* window, SDL_Renderer* renderer, ApplicationMode mode, WorldDocument& world, EditorTool* roadTool, CookedWorld& cooked)
    {
        Input input(window);

        if (!input.SetCameraEnabled(mode == ApplicationMode::Editor))
        {
            return false;
        }

        Sky sky;
        WorldRenderer worldRenderer(world.models.Location());
        FreeCamera freeCamera;
        WorldVisibility visibility;
        bool gameVisibilityStats = false;
        unsigned cacheSweep = 0;

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
        std::unique_ptr<GameSession> game;
        if (mode == ApplicationMode::Game)
        {
            game = std::make_unique<GameSession>(window, world, cooked.Ready() ? &cooked : nullptr);
            if (!game->Start()) return false;
        }
        Uint64 previousTime = SDL_GetTicksNS();
        const bool vsyncEnabled = SDL_SetRenderVSync(renderer, 1);

        while (true)
        {
            const Uint64 frameStart = SDL_GetTicksNS();
            input.BeginFrame();
            if (++cacheSweep == 120) { world.models.PruneUnused(); cacheSweep = 0; }

            SDL_Event event;

            while (SDL_PollEvent(&event))
            {
                // Dragging a spline rebuilds its geometry. Process the latest
                // queued pointer position, instead of rebuilding at mouse polling
                // frequency. Keep right-button gesture thresholds and camera input unchanged.
                if (roadTool && roadTool->ViewportFocused() && event.type == SDL_EVENT_MOUSE_MOTION &&
                    event.motion.windowID == SDL_GetWindowID(window) && event.motion.state == SDL_BUTTON_LMASK &&
                    !Input::CoalesceMouseMotion(event)) return false;
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

                if (event.type == SDL_EVENT_QUIT) return true;
                if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE && !event.key.repeat)
                {
                    if (game && !input.IsCameraEnabled() && game->MouseCaptured())
                    {
                        if (!game->ReleaseMouse()) return false;
                        continue;
                    }
                    return true;
                }

                if (game && event.type == SDL_EVENT_KEY_DOWN && event.key.scancode == SDL_SCANCODE_F1 && !event.key.repeat)
                {
                    const bool debug = !input.IsCameraEnabled();
                    if (debug)
                        freeCamera.LookAt(game->View().GetPosition(), game->View().GetPosition() + game->View().Forward());
                    if (!input.SetCameraEnabled(debug) || !game->SetDebugCamera(debug)) return false;
                    continue;
                }
                if (game && event.type == SDL_EVENT_KEY_DOWN && event.key.scancode == SDL_SCANCODE_F3 && !event.key.repeat)
                {
                    gameVisibilityStats = !gameVisibilityStats;
                    continue;
                }
                if (game && !input.IsCameraEnabled())
                {
                    if (!game->Handle(event)) return false;
                    continue;
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
                world.RefreshRailroadCrossings();
                world.RefreshTerrainFills();
            }
            const Uint64 now = SDL_GetTicksNS();
            const Uint64 elapsedNanoseconds = now - previousTime;
            previousTime = now;

            if (mode == ApplicationMode::Game)
            {
                g_WorldState.AdvanceTime(elapsedNanoseconds);
                game->Stream(input.IsCameraEnabled() ? freeCamera.GetPosition() : game->View().GetPosition(), worldRenderer);
                game->Update(elapsedNanoseconds / 1.0e9);
            }

            if (input.IsCameraEnabled() && (!roadTool || (roadTool->ViewportFocused() && !roadTool->DraggingGizmo())))
            {
                const double cameraElapsedSeconds = std::min(elapsedNanoseconds / 1.0e9, 0.1);
                freeCamera.Update(input.ReadCameraInput(mode), cameraElapsedSeconds);
            }

            const CameraView& view = game && !input.IsCameraEnabled() ? game->View() : freeCamera;

            if (roadTool && !roadTool->SetContentViewport(renderer))
            {
                return false;
            }

            if (!roadTool || !roadTool->IsPlanView())
            {
                const bool fogEnabled = !roadTool || roadTool->FogEnabled();
                if (!sky.Draw(renderer, view, g_WorldState.GetTimeOfDayMinutes()) ||
                    !worldRenderer.Begin(renderer, view, &sky.Lighting(), fogEnabled ? &sky.Fog() : nullptr, g_ProjectSettings.ViewDistance()))
                {
                    return false;
                }

                visibility.Synchronize(world);
                const auto visible = visibility.Query(world, view, worldRenderer, g_ProjectSettings.ViewDistance(), roadTool != nullptr);
                if (cooked.Ready()) cooked.Submit(worldRenderer, view, g_ProjectSettings.ViewDistance());
                for (const auto key : visible)
                {
                    if (cooked.Ready() && (key.kind == SceneKind::Road || key.kind == SceneKind::Intersection || key.kind == SceneKind::Terrain)) continue;
                    const bool selected = roadTool && roadTool->GeometrySelected(key);
                    switch (key.kind)
                    {
                    case SceneKind::Road:
                        worldRenderer.SubmitRoad(world.roads[key.index], world.roads[key.index].style.material, selected);
                        break;
                    case SceneKind::Intersection:
                        worldRenderer.SubmitRoad(world.IntersectionSurfaces().at(key.index).geometry,
                                                 world.intersections.at(key.index).material, selected);
                        break;
                    case SceneKind::Mesh:
                        worldRenderer.SubmitWorldMesh(world.meshes[key.index]);
                        break;
                    case SceneKind::Terrain:
                        worldRenderer.SubmitTerrain(*world.Terrain()[key.index], selected);
                        break;
                    case SceneKind::Object:
                        if (const auto* marker = dynamic_cast<const Marker*>(world.Objects()[key.index].get()))
                            worldRenderer.SubmitWorldMesh(marker->Instance(), marker->material);
                        else
                            worldRenderer.SubmitWorldMesh(world.Objects()[key.index]->Instance());
                        break;
                    default: break;
                    }
                }
                if (const Road* preview = roadTool ? roadTool->PreviewRoad() : nullptr)
                    worldRenderer.SubmitRoad(*preview, preview->style.material, false);

                if (game) game->Submit(worldRenderer);

                if (roadTool && roadTool->WireframeEnabled()) worldRenderer.WireframeSubmittedGeometry();
                worldRenderer.BeginUnlit();
                if (cooked.Ready()) cooked.SubmitLabels(worldRenderer, view);
                for (const auto key : visible)
                    if (key.kind == SceneKind::Object)
                        if (const auto* marker = dynamic_cast<const Marker*>(world.Objects()[key.index].get()))
                            marker->SubmitLabel(worldRenderer, view);

                if (roadTool)
                {
                    roadTool->SubmitGizmos(worldRenderer, world, freeCamera);
                }

                if (!worldRenderer.End())
                {
                    return false;
                }
                if (roadTool ? roadTool->VisibilityStatisticsEnabled() : gameVisibilityStats)
                {
                    // Game diagnostics stack below the 56-pixel controls/status
                    // bar. Keep the editor's existing overlay positions.
                    const float statisticsY = game ? 122.f : -1.f;
                    if (!(cooked.Ready() ? cooked.DrawStatistics(renderer, statisticsY) :
                        visibility.DrawStatistics(renderer, g_ProjectSettings.ViewDistance(), statisticsY)) ||
                        !worldRenderer.DrawCacheStatistics(renderer, game ? 64.f : 36.f)) return false;
                }
            }

            if (game && !game->DrawHUD(renderer)) return false;

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
        CookedWorld cooked;
        success = world.OpenPath(worldFile);
        if (success && mode == ApplicationMode::Game && cooked.Load(world))
            SDL_Log("Streaming cooked world asynchronously");
        else if (success)
        {
            if (mode == ApplicationMode::Game) SDL_Log("Using authored world (cook format v2 for streaming): %s", SDL_GetError());
            SDL_ClearError();
            const std::string source = world.Path();
            success = world.Open(source.c_str());
        }
        success = success && (!roadTool || roadTool->Initialize(window, world.models)) &&
                  RunWindowLoop(window, renderer, mode, world, roadTool, cooked);
    }

    if (!success)
    {
        const char* error = SDL_GetError();

        SDL_Log("%s: %s", title, error);
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, title, error, window);
    }

    if (roadTool) roadTool->ReleaseRenderResources();
    SDL_DestroyRenderer(renderer);
    if (renderWindow && renderWindow != window)
    {
        SDL_DestroyWindow(renderWindow);
    }
    SDL_DestroyWindow(window);
    SDL_Quit();

    return success ? 0 : 1;
}
