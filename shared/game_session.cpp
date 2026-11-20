#include "game_session.hpp"
#include "cooked_world.hpp"
#include "project_settings.hpp"
#include "car.hpp"
#include "character.hpp"
#include "test_pawn_meshes.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr double fixedStep = 1.0 / 60.0;
}

GameSession::GameSession(SDL_Window* window, WorldDocument& document, CookedWorld* streaming)
    : document(document), world(document), player(world), input(window), streaming(streaming)
{
}

bool GameSession::TryStartAt(Vector3 position, double yaw)
{
    auto car = std::make_unique<Car>();
    ObjectTransform transform{position, {0,yaw,0}};
    if (!car->PlaceOnGround(world, transform, 5.0)) return false;
    car->Teleport(transform);
    const auto frame = car->Instance().transform;
    for (double z : {-1.25,0.0,1.25})
        if (world.SphereBlocked(frame.Point({0,1.05,z}), 0.9)) return false;
    for (double lateral : {-2.8,2.8,-2.0,2.0})
    {
        const auto foot = world.collision.Ground(frame.Point({lateral,0,0}), 0.5, 2.0);
        if (!foot || !world.CharacterFits(foot->position)) continue;
        world.Add(std::move(car));
        return player.Spawn(foot->position, yaw - std::copysign(std::numbers::pi / 2, lateral));
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

std::vector<Vector3> GameSession::PhysicsSources() const
{
    if (!started) return {streaming->Spawns()[spawnIndex].position};
    std::vector<Vector3> sources;
    for (const auto& pawn : world.Pawns())
    {
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
    return sources;
}

void GameSession::Stream(Vector3 eye, WorldRenderer& renderer)
{
    if (!streaming) return;
    const auto sources = PhysicsSources();
    streaming->Update(eye, sources, world.collision, &renderer, g_ProjectSettings.ViewDistance(), g_ProjectSettings.Streaming());
    collisionReady = std::all_of(sources.begin(), sources.end(), [&](Vector3 p) { return streaming->CollisionReady(p); });
    if (!started && collisionReady)
    {
        const auto& candidates = streaming->Spawns();
        const auto& spawn = candidates[spawnIndex];
        if (TryStartAt(spawn.position, spawn.yaw)) FinishStart();
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

bool GameSession::DrawHUD(SDL_Renderer* renderer) const
{
    SDL_Rect viewport{};
    if (!SDL_GetRenderViewport(renderer, &viewport)) return false;
    const char* controls = "WASD: walk   Mouse: look   Shift: sprint   Space: jump   F: enter car";
    std::string status;
    if (const auto* car = dynamic_cast<const Car*>(player.controller.Controlled()))
    {
        controls = "W/S: accelerate, brake/reverse   A/D: steer   Space: handbrake   Mouse: look   F: exit";
        char speed[64];
        SDL_snprintf(speed, sizeof(speed), "%.0f km/h", std::abs(car->Speed()) * 3.6);
        status = speed;
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
    return SDL_SetRenderDrawColor(renderer, 22,24,28,255) &&
           SDL_RenderFillRect(renderer, &background) &&
           SDL_SetRenderDrawColor(renderer, 225,228,235,255) &&
           SDL_RenderDebugText(renderer, 12, y + 9, controls) &&
           SDL_RenderDebugText(renderer, 12, y + 30, status.c_str());
}
