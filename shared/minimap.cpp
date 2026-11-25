#include "minimap.hpp"
#include "game_hud_layout.hpp"
#include "world_document.hpp"
#include "cooked_world.hpp"
#include "view_geometry.hpp"
#include <cmath>
#include <numbers>

namespace
{
    constexpr double pitch = 55 * std::numbers::pi / 180;
    constexpr double cameraDistance = 260;
    constexpr float markerHeight = .70f;
    constexpr std::array<SDL_FColor, size_t(Minimap::Layer::Count)> colors{{
        {.16f, .40f, .65f, 1}, // Water.
        {.72f, .58f, .40f, 1}, // Railroad embankments.
        {.48f, .50f, .53f, 1}  // Road pavement.
    }};

    Bounds3 Flatten(Bounds3 bounds)
    {
        bounds.min.y = bounds.max.y = 0;
        return bounds;
    }
}

std::array<Vector3, 3> Minimap::Source::Triangle(size_t i) const
{
    if (geometry)
    {
        const auto& t = geometry->pavementTriangles[i];
        return {t[0].position, t[1].position, t[2].position};
    }
    const auto& data = *mesh->mesh;
    return {mesh->transform.Point(data.vertices[data.indices[i*3]].position),
            mesh->transform.Point(data.vertices[data.indices[i*3+1]].position),
            mesh->transform.Point(data.vertices[data.indices[i*3+2]].position)};
}

bool Minimap::BoundsNeeded(const Bounds3& bounds) const
{
    return !bounds.Empty() && BoundsDistanceSquared(Flatten(bounds), {position.x, 0, position.z}) <= radiusMeters * radiusMeters;
}

void Minimap::Synchronize(const WorldDocument& document)
{
    if (sourceDocument == &document && sourceRevision == RoadGeometry::latestRevision &&
        waterRevision == TerrainChunk::LatestWaterRevision() && terrainCount == document.Terrain().size() &&
        roadCount == document.roads.size() && junctionCount == document.IntersectionSurfaces().size()) return;
    sourceDocument = &document;
    sourceRevision = RoadGeometry::latestRevision;
    waterRevision = TerrainChunk::LatestWaterRevision(); terrainCount = document.Terrain().size();
    roadCount = document.roads.size(); junctionCount = document.IntersectionSurfaces().size();
    sources.clear(); index.BeginSync();
    const auto add = [&](Source source, size_t count) {
        // Split long roads, railroads and large water bodies into searchable
        // spans instead of walking every triangle on every HUD frame.
        constexpr size_t trianglesPerSpan = 128;
        for (size_t first = 0; first < count; first += trianglesPerSpan)
        {
            source.first = first; source.end = std::min(first + trianglesPerSpan, count);
            Bounds3 bounds;
            for (size_t i = first; i < source.end; ++i)
                for (const auto& vertex : source.Triangle(i)) bounds.Include(vertex);
            index.Upsert({SceneKind::Road, int(sources.size())}, Flatten(bounds));
            sources.push_back(source);
        }
    };
    for (const auto& road : document.roads)
        add({&road, nullptr, road.style.railroad ? Layer::Railroad : Layer::Road}, road.pavementTriangles.size());
    for (const auto& [id, surface] : document.IntersectionSurfaces())
        add({&surface.geometry}, surface.geometry.pavementTriangles.size());
    for (const auto& chunk : document.Terrain())
        if (chunk->HasWater() && chunk->Water().mesh)
            add({nullptr, &chunk->Water(), Layer::Water}, chunk->Water().mesh->indices.size() / 3);
    index.EndSync();
}

void Minimap::SubmitTriangle(const std::array<Vector3, 3>& triangle, Layer layer)
{
    Bounds3 bounds; for (auto p : triangle) bounds.Include(p);
    if (!BoundsNeeded(bounds)) return;
    ViewGeometry::Polygon polygon{}; polygon.count = 3;
    for (size_t i = 0; i < triangle.size(); ++i)
    {
        // Subtract in double precision before projecting, including at distant
        // geographic coordinates. Elevation does not displace the map footprint.
        const auto delta = triangle[i] - position;
        const double ahead = Math::Dot(delta, forward);
        polygon.points[i] = {Math::Dot(delta, right), ahead * std::sin(pitch), cameraDistance + ahead * std::cos(pitch)};
    }
    const double halfWidth = rectangle.w * .5, centerY = rectangle.h * markerHeight;
    const std::array<Vector3, 5> planes{{{0,0,1}, {focal,0,halfWidth}, {-focal,0,halfWidth},
                                       {0,-focal,centerY}, {0,focal,rectangle.h-centerY}}};
    for (size_t i = 0; i < planes.size() && polygon.count >= 3; ++i)
        polygon = ViewGeometry::Clip(polygon, planes[i], i == 0 ? 1.0 : 0.0);
    if (polygon.count < 3) return;
    const auto vertex = [&](Vector3 p) {
        return SDL_Vertex{{float(rectangle.x + halfWidth + p.x*focal/p.z),
                           float(rectangle.y + centerY - p.y*focal/p.z)}, colors[size_t(layer)], {}};
    };
    auto& batch = vertices[size_t(layer)];
    for (int i = 2; i < polygon.count; ++i)
    {
        batch.push_back(vertex(polygon.points[0]));
        batch.push_back(vertex(polygon.points[i-1]));
        batch.push_back(vertex(polygon.points[i]));
    }
}

void Minimap::Submit(const MeshInstance& instance, Layer layer)
{
    if (!instance.mesh || !BoundsNeeded(WorldBounds(instance.mesh->bounds, instance.transform))) return;
    const auto& mesh = *instance.mesh;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
        SubmitTriangle({instance.transform.Point(mesh.vertices[mesh.indices[i]].position),
                        instance.transform.Point(mesh.vertices[mesh.indices[i+1]].position),
                        instance.transform.Point(mesh.vertices[mesh.indices[i+2]].position)}, layer);
}

bool Minimap::Draw(SDL_Renderer* renderer, const WorldDocument& document, const CookedWorld* streaming,
                   Vector3 center, Vector3 facing)
{
    SDL_Rect viewport{};
    if (!SDL_GetRenderViewport(renderer, &viewport)) return false;
    const float scale = GameHUD::Scale(viewport);
    if (scale < .5f) return true;
    rectangle = GameHUD::MapRect(viewport);
    position = center;
    // Use the rendered pawn's actual forward axis, not its Euler Y angle:
    // equivalent X/Y/Z rotations can otherwise reverse the map's heading.
    // Retain the last useful heading while the vehicle points nearly vertically.
    facing = Math::Normalize(facing);
    if (std::hypot(facing.x, facing.z) > .2)
        forward = Math::Normalize({facing.x, 0, facing.z});
    right = {forward.z, 0, -forward.x};
    focal = rectangle.h * 1.1;
    for (auto& batch : vertices) batch.clear();
    if (streaming && streaming->Ready()) streaming->SubmitMinimap(*this);
    else
    {
        Synchronize(document);
        for (auto key : index.Query({position.x,0,position.z}, radiusMeters))
        {
            const auto& source = sources[key.index];
            for (size_t i = source.first; i < source.end; ++i)
                SubmitTriangle(source.Triangle(i), source.layer);
        }
    }
    SDL_BlendMode oldBlend;
    SDL_Rect oldClip{};
    Uint8 oldR, oldG, oldB, oldA;
    const bool clipped = SDL_RenderClipEnabled(renderer);
    if (!SDL_GetRenderDrawBlendMode(renderer, &oldBlend) || !SDL_GetRenderClipRect(renderer, &oldClip) ||
        !SDL_GetRenderDrawColor(renderer, &oldR, &oldG, &oldB, &oldA)) return false;
    SDL_Rect clip{int(std::floor(rectangle.x)), int(std::floor(rectangle.y)),
                  int(std::ceil(rectangle.w)), int(std::ceil(rectangle.h))};
    if (clipped && !SDL_GetRectIntersection(&clip, &oldClip, &clip)) return true;
    const auto draw = [&] {
        // 20% transparent = 80% opaque (204/255).
        if (!SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND) || !SDL_SetRenderClipRect(renderer, &clip) ||
            !SDL_SetRenderDrawColor(renderer, 12,15,20,204) || !SDL_RenderFillRect(renderer, &rectangle)) return false;
        // Water goes below tracks and pavement even when its cell arrives last.
        for (const auto& batch : vertices)
            if (!batch.empty() && !SDL_RenderGeometry(renderer, nullptr, batch.data(), int(batch.size()), nullptr, 0)) return false;
        const float x = rectangle.x + rectangle.w*.5f, y = rectangle.y + rectangle.h*markerHeight;
        const auto arrow = [&](float size, SDL_FColor color) {
            const SDL_Vertex points[]{{{x,y-13*size},color,{}}, {{x-9*size,y+9*size},color,{}},
                                      {{x,y+5*size},color,{}}, {{x+9*size,y+9*size},color,{}}};
            const int indices[]{0,1,2,0,2,3};
            return SDL_RenderGeometry(renderer, nullptr, points, 4, indices, 6);
        };
        return arrow(scale*1.25f, {.04f,.05f,.07f,1}) && arrow(scale, {.94f,.96f,1,1}) &&
               SDL_SetRenderDrawColor(renderer, 92,101,114,180) && SDL_RenderRect(renderer, &rectangle);
    };
    const bool drawn = draw();
    const bool clipRestored = SDL_SetRenderClipRect(renderer, clipped ? &oldClip : nullptr);
    const bool blendRestored = SDL_SetRenderDrawBlendMode(renderer, oldBlend);
    const bool colorRestored = SDL_SetRenderDrawColor(renderer, oldR, oldG, oldB, oldA);
    return drawn && clipRestored && blendRestored && colorRestored;
}
