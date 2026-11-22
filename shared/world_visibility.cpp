#include "world_visibility.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"
#include "camera_view.hpp"
#include "marker.hpp"
#include <algorithm>
#include <cmath>

Bounds3 WorldVisibility::MeshBoundsCache::Get(const MeshInstance& instance)
{
    if (!instance.mesh) return {};
    if (!valid || local != instance.mesh->bounds || transform != instance.transform)
    {
        local = instance.mesh->bounds;
        transform = instance.transform;
        world = WorldBounds(local, transform);
        valid = true;
    }
    return world;
}

void WorldVisibility::Synchronize(const WorldDocument& world)
{
    const auto start = SDL_GetTicksNS();
    index.BeginSync();
    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
        index.Upsert({SceneKind::Road, i}, world.roads[i].bounds);
    for (const auto& [id, surface] : world.IntersectionSurfaces())
        index.Upsert({SceneKind::Intersection, id}, surface.geometry.bounds);
    meshes.resize(world.meshes.size());
    for (int i = 0; i < static_cast<int>(meshes.size()); ++i)
        if (world.meshes[i].mesh) index.Upsert({SceneKind::Mesh, i}, meshes[i].Get(world.meshes[i]));
    terrain.resize(world.Terrain().size());
    water.resize(terrain.size());
    for (int i = 0; i < static_cast<int>(terrain.size()); ++i)
    {
        const auto& chunk = *world.Terrain()[i];
        if (!chunk.Instance().mesh && !chunk.Water().mesh) continue;
        auto bounds = terrain[i].Get(chunk.Instance());
        const auto lake = water[i].Get(chunk.Water());
        if (!lake.Empty()) { bounds.Include(lake.min); bounds.Include(lake.max); }
        index.Upsert({SceneKind::Terrain, i}, bounds);
    }
    objects.resize(world.Objects().size());
    for (int i = 0; i < static_cast<int>(objects.size()); ++i)
    {
        const auto& object = *world.Objects()[i];
        if (!object.Instance().mesh) continue;
        auto bounds = objects[i].Get(object.Instance());
        if (const auto* marker = dynamic_cast<const Marker*>(&object))
            bounds = Marker::VisibilityBounds(bounds, marker->Text(), marker->GetTransform().scale);
        index.Upsert({SceneKind::Object, i}, bounds);
    }
    index.EndSync();
    syncMilliseconds = double(SDL_GetTicksNS() - start) / 1e6;
}

std::span<const SceneKey> WorldVisibility::Query(const WorldDocument& world, const CameraView& camera,
                                               const WorldRenderer& renderer, double distance, bool editor)
{
    const auto start = SDL_GetTicksNS();
    visible.clear();
    hidden = outsideFrustum = 0;
    for (const auto key : index.Query(camera.GetPosition(), distance))
    {
        if (!editor && key.kind == SceneKind::Object && world.Objects()[key.index]->EditorOnly()) continue;
        if (editor && world.hierarchy.EditorHidden(key)) { ++hidden; continue; }
        if (!renderer.BoundsNeeded(index.Bounds(key))) { ++outsideFrustum; continue; }
        visible.push_back(key);
    }
    // Keep the previous world submission order, independent of grid layout.
    const auto rank = [](SceneKind kind) {
        switch (kind)
        {
        case SceneKind::Road: return 0;
        case SceneKind::Intersection: return 1;
        case SceneKind::Mesh: return 2;
        case SceneKind::Terrain: return 3;
        default: return 4;
        }
    };
    std::sort(visible.begin(), visible.end(), [&](SceneKey a, SceneKey b) {
        const auto ar = rank(a.kind), br = rank(b.kind);
        return ar == br ? a.index < b.index : ar < br;
    });
    queryMilliseconds = double(SDL_GetTicksNS() - start) / 1e6;
    return visible;
}

bool WorldVisibility::DrawStatistics(SDL_Renderer* renderer, double distance, float y) const
{
    SDL_Rect viewport{};
    if (!SDL_GetRenderViewport(renderer, &viewport)) return false;
    if (y < 0) y = std::max(48.f, float(viewport.h) - 168.f);
    const SDL_FRect box{8, y, std::min(520.f, float(viewport.w) - 16), 100};
    const auto& s = index.Stats();
    return SDL_SetRenderDrawColor(renderer, 24, 26, 30, 255) && SDL_RenderFillRect(renderer, &box) &&
           SDL_SetRenderDrawColor(renderer, 230, 232, 235, 255) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 8, "Visibility (F3)  %.2f km  |  World fully loaded", distance / 1000) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 24, "Scene entries: %zu  Nearby: %zu  Submitted: %zu", s.entries, s.inRange, visible.size()) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 40, "Cells: %zu/%zu  Candidates: %zu  Large: %zu", s.visitedCells, s.cells, s.candidates, s.oversized) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 56, "Hidden: %zu  Outside view: %zu  Updated: %zu", hidden, outsideFrustum, s.updated) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 72, "Index sync: %.3f ms  Query: %.3f ms", syncMilliseconds, queryMilliseconds);
}
