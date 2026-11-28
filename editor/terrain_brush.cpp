#include "road_editor.hpp"
#include "free_camera.hpp"
#include "view_geometry.hpp"
#include "world_renderer.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

const TerrainBrushSettings& RoadEditor::TerrainBrush() const
{
#ifdef _WIN32
    return panels.TerrainBrush();
#else
    static const TerrainBrushSettings disabled;
    return disabled;
#endif
}

std::optional<Vector3> RoadEditor::PickTerrainBrush(const WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer) const
{
    if (pointer.x < 0 || pointer.y < 0 || pointer.x >= scene.w || pointer.y >= scene.h) return {};
    if (planView)
    {
        auto p = ToWorld(pointer.x, pointer.y);
        std::optional<double> height;
        for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
            if (!world.hierarchy.EditorHidden({SceneKind::Terrain, i}))
                if (auto h = world.Terrain()[i]->SculptHeight(p.x, p.z); h && (!height || *h > *height)) height = h;
        if (!height) return {};
        p.y = *height;
        return p;
    }
    ViewportPicker picker(camera, scene, pointer);
    // Respect roads, props and water occluding the terrain under the pointer.
    const auto hit = picker.TestPlacementSurfaces(world, true);
    if (!hit || hit->kind != SceneKind::Terrain || !world.Terrain()[hit->index]->CanSculpt()) return {};
    for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
        if (!world.hierarchy.EditorHidden({SceneKind::Terrain, i}) && world.Terrain()[i]->HasWater() &&
            picker.TestMesh(world.Terrain()[i]->Water())) return {};
    return picker.HitPosition();
}

bool RoadEditor::TerrainBrushInput(const SDL_Event& event, SDL_FPoint pointer, WorldDocument& world, const FreeCamera& camera)
{
    if (!TerrainBrush().enabled || drag != Drag::None || rampDrag || waterEdit || state.drawing) return false;
    if (pointer.x < 0 || pointer.y < 0 || pointer.x >= scene.w || pointer.y >= scene.h) return false;
    if (event.type == SDL_EVENT_MOUSE_WHEEL && (SDL_GetModState() & SDL_KMOD_CTRL))
    {
        FinishTerrainStroke(world);
        const double steps = event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -event.wheel.y : event.wheel.y;
#ifdef _WIN32
        panels.SetTerrainRadius(std::clamp(TerrainBrush().radius * std::pow(1.15, steps), 0.1, 1000.0));
#endif
        return true;
    }
    if (terrainStroke && event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button != SDL_BUTTON_LEFT)
        FinishTerrainStroke(world);
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT)
    {
        const auto position = PickTerrainBrush(world, camera, pointer);
        if (!position) { status = "Point at generated land to sculpt terrain"; return true; }
        status.clear();
        terrainStroke.emplace(TerrainStroke{EditSnapshot(state), TerrainBrush(), position->y, 0, position});
        terrainHover = position;
        SDL_CaptureMouse(true);
        StampTerrain(world, *position, 0.05); // Quick clicks make one small stamp too.
        return true;
    }
    if (event.type == SDL_EVENT_MOUSE_MOTION)
    {
        mouse = pointer;
        return true;
    }
    return false;
}

void RoadEditor::StampTerrain(WorldDocument& world, Vector3 center, double elapsed)
{
    if (!terrainStroke) return;
    auto& stroke = *terrainStroke;
    const auto& settings = stroke.settings;
    center.y = stroke.height;
    TerrainStamp stamp{settings.tool, center, settings.radius,
        settings.tool == TerrainTool::Flatten ? 1 - std::exp(-settings.strength * elapsed) : settings.strength * elapsed,
        settings.seamFalloff};
    for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
    {
        auto& chunk = *world.Terrain()[i];
        if (!chunk.CanSculpt() || world.hierarchy.EditorHidden({SceneKind::Terrain, i})) continue;
        const auto original = chunk.SculptEdits();
        SDL_ClearError();
        if (chunk.Sculpt(stamp)) stroke.before.terrainSculpt.try_emplace(chunk.persistentId, original);
        else if (*SDL_GetError()) status = SDL_GetError();
    }
}

void RoadEditor::FinishTerrainStroke(WorldDocument& world, bool cancel)
{
    if (!terrainStroke) return;
    auto stroke = std::move(*terrainStroke);
    terrainStroke.reset();
    SDL_CaptureMouse(false);
    if (stroke.before.terrainSculpt.empty()) return;
    EditSnapshot after(state);
    for (const auto& chunk : world.Terrain())
        if (auto it = stroke.before.terrainSculpt.find(chunk->persistentId); it != stroke.before.terrainSculpt.end())
        {
            if (cancel)
            {
                if (!chunk->SetSculptEdits(it->second)) status = SDL_GetError();
            }
            else after.terrainSculpt.emplace(chunk->persistentId, chunk->SculptEdits());
        }
    if (!cancel && history.Record(-1, std::move(stroke.before), std::move(after))) Save(world);
    UpdateWindowTitle();
}

void RoadEditor::Update(WorldDocument& world, const FreeCamera& camera, double elapsed)
{
    const auto& settings = TerrainBrush();
    if (terrainStroke && (!settings.enabled || settings.tool != terrainStroke->settings.tool ||
        settings.radius != terrainStroke->settings.radius || settings.strength != terrainStroke->settings.strength ||
        settings.seamFalloff != terrainStroke->settings.seamFalloff ||
        !ViewportFocused() || !(SDL_GetMouseState(nullptr, nullptr) & SDL_BUTTON_LMASK))) FinishTerrainStroke(world);
    terrainHover.reset();
    if (!settings.enabled || drag != Drag::None || rampDrag || waterEdit || state.drawing) return;
    const auto pointer = GizmoPointer();
    if (!pointer) { if (terrainStroke) terrainStroke->previous.reset(); return; }
    terrainHover = PickTerrainBrush(world, camera, *pointer);
    if (!terrainStroke) return;
    if (!terrainHover) { terrainStroke->previous.reset(); terrainStroke->elapsed = 0; return; }
    auto& stroke = *terrainStroke;
    stroke.elapsed += std::clamp(elapsed, 0.0, 0.1);
    if (stroke.elapsed < 0.05) return;
    const double dt = std::min(stroke.elapsed, 0.1);
    stroke.elapsed = 0;
    const auto from = stroke.previous.value_or(*terrainHover), to = *terrainHover;
    const double distance = std::hypot(to.x - from.x, to.z - from.z);
    // Sample fast mouse motion as a path, with bounded per-frame work. Do not
    // bridge a lost terrain hit or jump between distant islands.
    const int steps = std::clamp(static_cast<int>(std::min(32.0, std::ceil(distance / (stroke.settings.radius * 0.35)))), 1, 32);
    if (distance > stroke.settings.radius * 12)
        StampTerrain(world, to, dt);
    else
        for (int i = 1; i <= steps; ++i) StampTerrain(world, from + (to - from) * (double(i) / steps), dt / steps);
    stroke.previous = to;
}

namespace
{
    SDL_Color BrushColor(TerrainTool tool)
    {
        return tool == TerrainTool::Raise ? SDL_Color{255,205,70,255} :
               tool == TerrainTool::Lower ? SDL_Color{90,190,255,255} : SDL_Color{160,255,150,255};
    }
}

std::vector<std::optional<Vector3>> RoadEditor::TerrainBrushContour(const WorldDocument& world) const
{
    if (!TerrainBrush().enabled || !terrainHover) return {};
    const double radius = TerrainBrush().radius;
    const int segments = std::clamp(static_cast<int>(std::ceil(2 * std::numbers::pi * radius /
                                                std::clamp(radius / 24, 0.025, 4.0))), 128, 2048);
    std::vector<const TerrainChunk*> candidates;
    for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
    {
        const auto& chunk = *world.Terrain()[i];
        if (!chunk.CanSculpt() || world.hierarchy.EditorHidden({SceneKind::Terrain, i})) continue;
        const auto& b = chunk.Instance().mesh->bounds;
        if (b.max.x < terrainHover->x-radius || b.min.x > terrainHover->x+radius ||
            b.max.z < terrainHover->z-radius || b.min.z > terrainHover->z+radius) continue;
        candidates.push_back(&chunk);
    }
    std::vector<std::optional<Vector3>> contour;
    contour.reserve(segments+1);
    for (int i = 0; i <= segments; ++i)
    {
        const double angle = i * (2 * std::numbers::pi / segments);
        auto p = *terrainHover + Vector3{std::cos(angle),0,std::sin(angle)} * radius;
        std::optional<double> height;
        for (const auto* chunk : candidates)
            if (auto h = chunk->SculptHeight(p.x,p.z); h && (!height || *h > *height)) height = h;
        if (height) { p.y = *height; contour.push_back(p); }
        else contour.push_back({});
    }
    return contour;
}

void RoadEditor::SubmitTerrainBrush(WorldRenderer& renderer, const WorldDocument& world, const FreeCamera& camera) const
{
    if (planView || !terrainHover) return;
    const auto color = BrushColor(TerrainBrush().tool);
    const SDL_FColor ink{color.r/255.f,color.g/255.f,color.b/255.f,1};
    const SDL_Rect viewport{0,0,scene.w,scene.h};
    const double focal = ViewportFocalLength(scene);
    if (focal <= 0) return;
    const auto sides = ViewGeometry::SidePlanes(focal,viewport);
    const auto eye = camera.GetPosition(), forward = camera.ViewRay(0,0);
    const auto right = (camera.ViewRay(1,0)-forward)*(1/focal), up = (camera.ViewRay(0,1)-forward)*(1/focal);
    std::optional<Vector3> previous;
    for (const auto& point : TerrainBrushContour(world))
    {
        if (!point) { previous.reset(); continue; }
        const auto p = camera.WorldToView(*point);
        if (previous)
            if (auto line = ViewGeometry::ProjectStroke(*previous,p,sides,focal,viewport,worldNearDepth,1.2f))
            {
                std::array<Vector3,4> corners;
                for (int j = 0; j < 4; ++j)
                {
                    const auto screen = line->corners[j];
                    const auto ray = forward + right*(screen.x-scene.w*0.5) + up*(scene.h*0.5-screen.y);
                    const double depth = j < 2 ? line->startDepth : line->endDepth;
                    corners[j] = eye + ray*std::max(worldNearDepth,depth/1.0001);
                }
                renderer.SubmitTriangle(corners[0],corners[1],corners[2],ink);
                renderer.SubmitTriangle(corners[0],corners[2],corners[3],ink);
            }
        previous = p;
    }
}

bool RoadEditor::DrawTerrainBrush(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera&) const
{
    if (!TerrainBrush().enabled || !terrainHover) return true;
    const auto tool = TerrainBrush().tool;
    const auto color = BrushColor(tool);
    if (!SDL_SetRenderDrawColor(renderer,color.r,color.g,color.b,color.a)) return false;
    if (planView)
    {
        std::optional<SDL_FPoint> previous;
        for (const auto& point : TerrainBrushContour(world))
        {
            if (!point) { previous.reset(); continue; }
            const auto p = ToScreen(*point);
            if (previous && !SDL_RenderLine(renderer,previous->x,previous->y,p.x,p.y)) return false;
            previous = p;
        }
    }
    const char* name = tool == TerrainTool::Raise ? "Raise" : tool == TerrainTool::Lower ? "Lower" : "Flatten";
    return SDL_RenderDebugTextFormat(renderer,12,46,"Terrain: %s | Radius %.2f m | Ctrl+wheel: radius | Esc: cancel",name,TerrainBrush().radius);
}
