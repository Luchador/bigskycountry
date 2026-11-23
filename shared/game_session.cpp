#include "game_session.hpp"
#include "cooked_world.hpp"
#include "project_settings.hpp"
#include "car.hpp"
#include "character.hpp"
#include "test_pawn_meshes.hpp"
#include "speedometer.hpp"
#include "traffic_overlay.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>
#include <tuple>

namespace
{
    constexpr double fixedStep = 1.0 / 60.0;
}

GameSession::GameSession(SDL_Window* window, WorldDocument& document, CookedWorld* streaming)
    : document(document), world(document), player(world), input(window), streaming(streaming), traffic(streaming ? TrafficNetwork{} : TrafficNetwork::Build(document))
{
    junctionTraffic = std::make_unique<TrafficJunctions>(world, Traffic());
    world.prepareControllers = [this](double seconds) { junctionTraffic->Update(seconds, trafficDrivers); };
}

bool GameSession::TryStartAt(Vector3 position, double yaw, bool authored)
{
    auto car = std::make_unique<Car>();
    ObjectTransform transform{position, {0,yaw,0}};
    if (!car->PlaceOnGround(world, transform, 5.0) && !authored) return false;
    car->Teleport(transform);
    const auto frame = car->Instance().transform;
    for (double z : {-1.25,0.0,1.25})
        if (!authored && world.SphereBlocked(frame.Point({0,1.05,z}), 0.9)) return false;
    for (double lateral : {-2.8,2.8,-2.0,2.0})
    {
        const auto foot = world.collision.Ground(frame.Point({lateral,0,0}), 0.5, 2.0);
        if (!foot || !world.CharacterFits(foot->position)) continue;
        world.Add(std::move(car));
        return player.Spawn(foot->position, yaw - std::copysign(std::numbers::pi / 2, lateral));
    }
    if (authored)
    {
        // Honor an explicit location even in mid-air or on a narrow surface.
        // Never silently move the player back to an unrelated automatic spawn.
        // Start beside the car; gravity handles unsupported placements.
        const auto foot = frame.Point({-2.8,0,0});
        world.Add(std::move(car));
        return player.Spawn(foot, yaw + std::numbers::pi / 2);
    }
    return false;
}

bool GameSession::Start()
{
    if (streaming)
    {
        if (streaming->Spawns().empty()) return SDL_SetError("Cooked world has no spawn candidates");
        const auto position = streaming->Spawns().front().position;
        loadingCamera.LookAt(position + Vector3{0, 5, -10}, position);
        return input.SetCaptured(true);
    }
    if (const int start = document.SpawnPointIndex(); start >= 0)
    {
        const auto& pose = document.Objects()[start]->GetTransform();
        if (!TryStartAt(pose.position, pose.rotation.y, true)) return false;
        FinishStart();
        return input.SetCaptured(true);
    }
    bool placed = false;
    for (const auto& road : document.roads)
    {
        if (road.style.railroad) continue;
        const auto samples = road.laneCenterlines.empty() ? road.SegmentSamples(0) :
                             std::span<const Vector3>(road.laneCenterlines.front());
        if (samples.size() < 3) continue;
        const auto i = samples.size() / 2;
        const Vector3 tangent = samples[i + 1] - samples[i - 1];
        if (TryStartAt(samples[i], std::atan2(tangent.x, tangent.z)))
        {
            placed = true;
            break;
        }
    }
    const auto tryPlacement = [&](const MeshInstance& mesh) {
        if (placed || !mesh.mesh || mesh.mesh->bounds.Empty()) return;
        const auto& bounds = mesh.mesh->bounds;
        const auto center = mesh.transform.Point((bounds.min + bounds.max) * 0.5);
        placed = TryStartAt(center, 0);
    };
    for (const auto& chunk : document.Terrain()) tryPlacement(chunk->Instance());
    for (const auto& mesh : document.meshes) tryPlacement(mesh);
    if (!placed)
    {
        // Empty or unsuitable worlds still provide a visible surface to test on.
        testGround = {TestPawnMeshes::Ground(), {}};
        world.collision.AddMesh(testGround);
        world.collision.Finish();
        placed = TryStartAt({0,0,0}, 0);
    }
    if (!placed) return SDL_SetError("Could not find a clear player and test car spawn");
    FinishStart();
    return input.SetCaptured(true);
}

bool GameSession::FinishStart()
{
    started = true;
    player.controller.camera.Update(*player.controller.Controlled(), 1.0, 0, world.collision);
    return true;
}

std::vector<Vector3> GameSession::PhysicsSources(bool includeTraffic) const
{
    if (!started) return {streaming->Spawns()[spawnIndex].position};
    std::vector<Vector3> sources;
    for (const auto& pawn : world.Pawns())
    {
        if (!includeTraffic && dynamic_cast<const TrafficDriver*>(pawn->Controller())) continue;
        if (const auto* character = dynamic_cast<const Character*>(pawn.get()); character && character->IsSeated()) continue;
        const auto position = pawn->GetTransform().position;
        Vector3 velocity{};
        if (const auto* car = dynamic_cast<const Car*>(pawn.get())) velocity = car->Velocity();
        else if (const auto* character = dynamic_cast<const Character*>(pawn.get())) velocity = character->Velocity();
        // Cover the complete upcoming frame, including fast falls and airborne vehicles.
        const auto travel = velocity * 0.5;
        const int steps = std::max(1, static_cast<int>(std::ceil(Math::Length(travel) / 32.0)));
        for (int i = 0; i <= steps; ++i) sources.push_back(position + travel * (double(i) / steps));
    }
    if (includeTraffic)
    {
        for (const auto& driver : trafficDrivers)
            for (double d = 0; d <= 120; d += 24) sources.push_back(driver->Lane().At(driver->Progress() + d));
        // Queued cars share residency. Coalesce nearby preload requests, while
        // each car's exact collision-readiness checks remain independent.
        std::set<std::tuple<double,double,double>> seen;
        std::erase_if(sources, [&](Vector3 p) {
            return !seen.emplace(std::floor(p.x / 16), std::floor(p.y / 16), std::floor(p.z / 16)).second;
        });
    }
    return sources;
}

void GameSession::Stream(Vector3 eye, WorldRenderer& renderer)
{
    if (!streaming) return;
    const auto sources = PhysicsSources();
    streaming->Update(eye, sources, world.collision, &renderer, g_ProjectSettings.ViewDistance(), g_ProjectSettings.Streaming());
    const auto playerSources = PhysicsSources(false);
    collisionReady = std::all_of(playerSources.begin(), playerSources.end(), [&](Vector3 p) { return streaming->CollisionReady(p); });
    for (auto& driver : trafficDrivers)
        if (auto* pawn = driver->Controlled()) driver->suspended = !streaming->CollisionReady(pawn->GetTransform().position, 16);
    if (!started && collisionReady)
    {
        const auto& candidates = streaming->Spawns();
        const auto& spawn = candidates[spawnIndex];
        if (TryStartAt(spawn.position, spawn.yaw, spawn.authored)) FinishStart();
        else if (spawnIndex + 1 < candidates.size())
        {
            ++spawnIndex;
            loadingCamera.LookAt(candidates[spawnIndex].position + Vector3{0, 5, -10}, candidates[spawnIndex].position);
        }
        else
        {
            // This fallback is permitted only after collision is complete for every attempted spawn.
            testGround = {TestPawnMeshes::Ground(), {}};
            world.collision.AddMesh(testGround); world.collision.Finish();
            if (TryStartAt({0, 0, 0}, 0)) FinishStart();
        }
        collisionReady = false; // Revalidate around the newly spawned pawns next frame.
    }
}

const TrafficNetwork& GameSession::Traffic() const
{
    return streaming ? streaming->Traffic() : traffic;
}

void GameSession::RemoveTrafficTest()
{
    for (auto& driver : trafficDrivers)
    {
        const auto* pawn = driver->Controlled();
        const auto handle = pawn ? pawn->Handle() : 0;
        driver.reset();
        if (handle) world.Remove(handle);
    }
    trafficDrivers.clear();
    junctionTraffic->Clear();
}

void GameSession::RemoveLostTraffic()
{
    std::erase_if(trafficDrivers, [&](auto& driver) {
        if (!driver->FallenOutsideWorld()) return false;
        const auto* pawn = driver->Controlled();
        if (streaming && streaming->MayHaveGroundBelow(pawn->GetTransform().position)) return false;
        // An endlessly falling body demands more substeps for every other car
        // and keeps requesting collision for an ever-longer fall trajectory.
        const auto handle = pawn->Handle();
        junctionTraffic->Remove(handle);
        driver->Release();
        world.Remove(handle);
        return true;
    });
}

void GameSession::SpawnTrafficTest(const CameraView& view, int count)
{
    trafficDebug = true;
    if (!started) { trafficNotice = "Wait for the world to load."; return; }
    const auto& network = Traffic();
    if (network.lanes.empty()) { trafficNotice = "No traffic lanes. Recook this world with the traffic update."; return; }
    const auto hit = world.collision.Raycast(view.GetPosition(), view.Forward(), 250, true);
    if (!hit) { trafficNotice = "Aim at a loaded road surface within 250 m."; return; }
    const auto pick = network.Nearest(hit->position, 2.5);
    if (!pick) { trafficNotice = "Aim at a through lane; shared single-lane roads are not supported yet."; return; }
    const auto& lane = network.lanes[pick->lane];
    const double start = std::max(3.0, pick->distance);
    if (lane.Length() - start < 15) { trafficNotice = "Choose a lane with at least 15 m remaining."; return; }
    int added = 0;
    for (int i = 0; i < count; ++i)
    {
        const double at = start - i * 18.0;
        if (at < 3) { trafficNotice = "Not enough lane behind the chosen point."; break; }
        if (!SpawnTrafficCar(lane, at)) break;
        ++added;
    }
    if (added == count) trafficNotice = "Added " + std::to_string(added) + (added == 1 ? " car." : " cars.");
    else if (added) trafficNotice = "Added " + std::to_string(added) + " of " + std::to_string(count) + ": " + trafficNotice;
}

bool GameSession::SpawnTrafficCar(const TrafficLane& lane, double start)
{
    if (trafficDrivers.size() >= maxTrafficCars) { trafficNotice = "Traffic test limit: 64 cars. F7 clears them."; return false; }
    const auto position = lane.At(start);
    const auto* playerPawn = player.controller.Controlled();
    if (!playerPawn || Math::Length(position - playerPawn->GetTransform().position) > 250)
    { trafficNotice = "Place the test car within 250 m of the player."; return false; }
    if (streaming && !streaming->CollisionReady(position, 32))
    { trafficNotice = "Move closer and wait for collision to load before placing the test car."; return false; }
    auto car = std::make_unique<Car>();
    const auto tangent = lane.Tangent(start);
    ObjectTransform pose{position, {0, std::atan2(tangent.x, tangent.z), 0}};
    if (!car->PlaceOnGround(world, pose, 3.0)) { trafficNotice = "No supporting road surface at this lane."; return false; }
    const auto frame = pose.ToMeshTransform();
    for (double z : {-1.5, 0.0, 1.5})
        if (world.SphereBlocked(frame.Point({0,1.0,z}), 0.9))
        { trafficNotice = "The test car needs clear space, away from the player and other vehicles."; return false; }
    // Do not insert a stationary car into the braking space of approaching traffic.
    for (const auto* other : world.NearbyPawns(PawnIndex::Around(position, 80)))
    {
        if (!dynamic_cast<const Car*>(other)) continue;
        const auto p = lane.Project(other->GetTransform().position, std::max(0.0, start - 80), start + 80);
        if (p.error > 2.3) continue;
        const double delta = p.distance - start;
        const double velocity = Math::Dot(other->Velocity(), lane.Tangent(p.distance));
        const double closing = std::max(0.0, delta < 0 ? velocity : -velocity);
        if (std::abs(delta) < 7.5 + closing * 1.6 + closing * closing / 16.0)
        { trafficNotice = "Leave more space for nearby or approaching cars."; return false; }
    }
    car->Teleport(pose);
    const auto handle = world.Add(std::move(car));
    auto driver = std::make_unique<TrafficDriver>(world, lane, start);
    driver->SetNetwork(&Traffic());
    driver->collisionReady = [this](Vector3 p) { return !streaming || streaming->CollisionReady(p, 8); };
    if (!driver->Possess(handle)) { driver.reset(); world.Remove(handle); trafficNotice = "Cannot control the test car."; return false; }
    trafficDrivers.push_back(std::move(driver));
    trafficNotice.clear();
    return true;
}

bool GameSession::HandleTrafficKey(const SDL_Event& event, const CameraView& view)
{
    if (event.type != SDL_EVENT_KEY_DOWN || event.key.repeat ||
        (event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_GUI))) return false;
    if (event.key.key == SDLK_F6) SpawnTrafficTest(view, event.key.mod & SDL_KMOD_SHIFT ? 5 : 1);
    else if (event.key.key == SDLK_F7) { RemoveTrafficTest(); trafficNotice = "Test traffic removed."; }
    else if (event.key.key == SDLK_F8) trafficDebug = !trafficDebug;
    else return false;
    return true;
}

bool GameSession::DrawTrafficDebug(SDL_Renderer* renderer, const CameraView& view) const
{
    if (!trafficDebug) return true;
    const auto* testDriver = trafficDrivers.empty() ? nullptr : trafficDrivers.back().get();
    SDL_Rect viewport{}; if (!SDL_GetRenderViewport(renderer, &viewport)) return false;
    if (!DrawTrafficLanes(renderer, Traffic(), view, viewport, false, {}, 1, testDriver ? testDriver->Lane().id : 0)) return false;
    SDL_SetRenderDrawColor(renderer, 255, 220, 140, 255);
    const float x = viewport.w * 0.5f, y = viewport.h * 0.5f;
    if (!SDL_RenderLine(renderer, x-5, y, x+5, y) || !SDL_RenderLine(renderer, x, y-5, x, y+5)) return false;
    const SDL_FRect panel{8, float(viewport.h - 100), float(std::min(1000, viewport.w - 16)), 92};
    SDL_SetRenderDrawColor(renderer, 22, 24, 28, 255); if (!SDL_RenderFillRect(renderer, &panel)) return false;
    SDL_SetRenderDrawColor(renderer, 225, 228, 235, 255);
    if (!SDL_RenderDebugText(renderer, 16, panel.y+8, "Traffic: F6 add | Shift+F6 add 5 behind aim | F7 clear all | F8 overlay")) return false;
    if (testDriver)
    {
        char spacing[96] = "lane clear";
        if (testDriver->Leader()) SDL_snprintf(spacing, sizeof(spacing), "gap %.1f / desired %.1f m", testDriver->Gap(), testDriver->DesiredGap());
        if (!SDL_RenderDebugTextFormat(renderer, 16, panel.y+26, "%u cars | last: road %d | target %.1f mph | %s | %s",
            static_cast<unsigned>(trafficDrivers.size()), testDriver->Lane().roadId, testDriver->TargetSpeed() * 2.2369362921,
            spacing, testDriver->Reason())) return false;
    }
    const auto& stats = world.QueryStatistics();
    if (!SDL_RenderDebugTextFormat(renderer, 16, panel.y+44, "Physics: %u nearby queries | %u results | %u car-pair checks | %u junction reservations",
        static_cast<unsigned>(stats.queries), static_cast<unsigned>(stats.results), static_cast<unsigned>(world.CollisionPairChecks()), static_cast<unsigned>(junctionTraffic->Reservations()))) return false;
    return SDL_RenderDebugText(renderer, 16, panel.y+64, trafficNotice.c_str());
}

bool GameSession::Handle(const SDL_Event& event)
{
    if (debugCamera) return true;
    const bool success = input.Handle(event);
    if (!input.IsCaptured()) player.controller.StopInput();
    return success;
}

bool GameSession::ReleaseMouse()
{
    player.controller.StopInput();
    return input.SetCaptured(false);
}

bool GameSession::SetDebugCamera(bool enabled)
{
    debugCamera = enabled;
    player.controller.StopInput();
    return input.SetCaptured(!enabled);
}

void GameSession::Update(double seconds)
{
    seconds = std::clamp(seconds, 0.0, 0.25);
    if (streaming && (!started || !collisionReady))
    {
        accumulator = 0; // Never catch up physics through an unloaded collision boundary.
        input.Read(PawnInputProfile::OnFoot); // Consume presses/look, but preserve held movement keys.
        player.controller.StopInput();
        return;
    }
    player.EnsurePossession();
    auto* pawn = player.controller.Controlled();
    if (!pawn) return;
    auto actions = input.Read(pawn->InputProfile());
    if (actions.interact && player.Interact())
    {
        input.Clear();
        actions = {};
    }
    player.controller.Input(actions);
    accumulator += seconds;
    while (accumulator + 1e-12 >= fixedStep)
    {
        world.Step(fixedStep);
        RemoveLostTraffic();
        document.TickObjects(static_cast<float>(fixedStep));
        if (player.AfterStep(streaming))
        {
            // The saved support can be in a cell that was unloaded during the
            // fall. Do not spend the rest of this frame stepping through it.
            accumulator = 0;
            if (streaming) collisionReady = false;
            player.controller.ConsumePresses();
            break;
        }
        player.controller.ConsumePresses();
        accumulator = std::max(0.0, accumulator - fixedStep);
    }
    if (auto* active = player.controller.Controlled())
        player.controller.camera.Update(*active, accumulator / fixedStep, seconds, world.collision);
}

void GameSession::Submit(WorldRenderer& renderer) const
{
    if (testGround.mesh) renderer.SubmitWorldMesh(testGround);
    for (const auto& pawn : world.Pawns())
    {
        if (!debugCamera && player.HidePawn(pawn->Handle())) continue;
        if (const auto* character = dynamic_cast<const Character*>(pawn.get()); character && character->IsSeated()) continue;
        renderer.SubmitWorldMesh(pawn->RenderInstance(accumulator / fixedStep));
    }
}

bool GameSession::DrawHUD(SDL_Renderer* renderer, double framesPerSecond) const
{
    SDL_Rect viewport{};
    if (!SDL_GetRenderViewport(renderer, &viewport)) return false;
    const char* controls = "WASD: walk   Mouse: look   Shift: sprint   Space: jump   F: enter car";
    std::string status;
    const auto* vehicle = dynamic_cast<const Vehicle*>(player.controller.Controlled());
    if (vehicle)
    {
        controls = "W/S: accelerate, brake/reverse   A/D: steer   Space: handbrake   Mouse: look   F: exit";
    }
    else if (player.NearbyVehicle()) status = "Press F to enter the car";
    if (!player.Notice().empty()) status = player.Notice();
    if (debugCamera)
    {
        controls = "F1: return to player   WASD/QE: fly   Right mouse: look   Wheel: speed";
        status = "Debug camera";
    }
    else if (!input.IsCaptured())
    {
        controls = "Click to resume   Escape: quit";
        status = "Mouse released";
    }
    else if (status.empty()) status = "Escape: release mouse   F1: debug camera";
    if (streaming && (!started || !collisionReady))
        status = streaming->Error().empty() ? "Loading nearby collision..." : "Collision unavailable. Recook the world and restart.";
    constexpr float y = 0;
    const SDL_FRect background{0,y,float(viewport.w),56};
    const bool console = SDL_SetRenderDrawColor(renderer, 22,24,28,255) &&
           SDL_RenderFillRect(renderer, &background) &&
           SDL_SetRenderDrawColor(renderer, 225,228,235,255) &&
           SDL_RenderDebugText(renderer, 12, y + 9, controls) &&
           SDL_RenderDebugText(renderer, 12, y + 30, status.c_str());
    if (!console) return false;
    char fpsText[32] = "FPS: --";
    if (framesPerSecond > 0) SDL_snprintf(fpsText, sizeof(fpsText), "FPS: %.1f", framesPerSecond);
    const float fpsWidth = static_cast<float>(SDL_strlen(fpsText)) * 8.f + 12.f;
    const SDL_FRect fpsBackground{std::max(0.f, viewport.w - fpsWidth - 8.f), 4.f, fpsWidth, 22.f};
    return SDL_SetRenderDrawColor(renderer, 22,24,28,255) &&
           SDL_RenderFillRect(renderer, &fpsBackground) &&
           SDL_SetRenderDrawColor(renderer, 255,255,255,255) &&
           SDL_RenderDebugText(renderer, fpsBackground.x + 6.f, fpsBackground.y + 7.f, fpsText) &&
           (!vehicle || debugCamera || !started || DrawSpeedometer(renderer, vehicle->Speed()));
}
