#include "world_renderer.hpp"
#include "free_camera.hpp"
#include "terrain_chunk.hpp"
#include "sun.hpp"
#include "world_state.hpp"
#include "editor_colors.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

bool WorldRenderer::Begin(SDL_Renderer* target, const FreeCamera& view)
{
    if (!SDL_GetRenderViewport(target, &viewport))
    {
        return false;
    }
    renderer = target;
    camera = &view;
    viewport.x = viewport.y = 0;
    focal = std::min(viewport.w, viewport.h) * 0.5;
    scaleX = viewport.w > 0 ? 2.0 * focal / viewport.w : 0.0;
    scaleY = viewport.h > 0 ? 2.0 * focal / viewport.h : 0.0;
    vertices.clear();
    outline.clear();
    materialRanges.clear();
    sunDirection = Sun::Direction(g_WorldState.GetTimeOfDayMinutes());
    wireframeEnd = 0;
    foregroundStart = std::numeric_limits<std::size_t>::max();
    selectedStart = selectedEnd = outlineStart = 0;
    return true;
}

WorldDepthVertex WorldRenderer::WorldToClip(Vector3 position, SDL_FColor color) const
{
    const Vector3 v = camera->WorldToView(position);
    return {static_cast<float>(v.x * scaleX), static_cast<float>(v.y * scaleY),
            static_cast<float>(worldNearDepth), static_cast<float>(v.z), color};
}

void WorldRenderer::SubmitTriangle(Vector3 a, Vector3 b, Vector3 c, SDL_FColor color)
{
    vertices.push_back(WorldToClip(a, color));
    vertices.push_back(WorldToClip(b, color));
    vertices.push_back(WorldToClip(c, color));
}

void WorldRenderer::SubmitTriangle(const std::array<Vector3, 3>& positions, const std::array<SDL_FColor, 3>& colors)
{
    for (std::size_t i = 0; i < positions.size(); ++i)
    {
        vertices.push_back(WorldToClip(positions[i], colors[i]));
    }
}

void WorldRenderer::SubmitMesh(const MeshInstance& instance, SDL_FColor tint)
{
    if (!instance.mesh)
    {
        return;
    }
    const Mesh& mesh = *instance.mesh;
    projectedMesh.clear();
    projectedMesh.reserve(mesh.vertices.size());
    for (const MeshVertex& v : mesh.vertices)
    {
        const SDL_FColor color{v.color.r * tint.r, v.color.g * tint.g, v.color.b * tint.b, 1};
        projectedMesh.push_back(WorldToClip(instance.transform.Point(v.position), color));
    }
    const bool mirrored = instance.transform.Determinant() < 0;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        vertices.push_back(projectedMesh[mesh.indices[i]]);
        vertices.push_back(projectedMesh[mesh.indices[i + (mirrored ? 2 : 1)]]);
        vertices.push_back(projectedMesh[mesh.indices[i + (mirrored ? 1 : 2)]]);
    }
}

// Project in local meters using the two axes most parallel to the face. This
// gives roads and chunks the same texel density, including on steep slopes.
void WorldRenderer::SubmitMaterialTriangle(const std::array<Vector3, 3>& points,
                                           const std::array<SDL_FColor, 3>& colors, const MeshTransform& frame)
{
    const Vector3 normal = Math::Normalize(Math::Cross(points[1] - points[0], points[2] - points[0]));
    const Vector3 local = frame.Unrotate(normal);
    Vector3 u = frame.x, v = frame.z;
    if (std::abs(local.x) > std::abs(local.y) && std::abs(local.x) >= std::abs(local.z)) u = frame.y;
    else if (std::abs(local.z) > std::abs(local.y)) v = frame.y;
    Vector3 tangent = Math::Normalize(Math::Cross(normal, v));
    if (Math::Dot(tangent, u) < 0) tangent = tangent * -1;
    Vector3 bitangent = Math::Cross(normal, tangent);
    if (Math::Dot(bitangent, v) < 0) bitangent = bitangent * -1;
    const auto toTangent = [&](Vector3 d) -> std::array<float, 3>
    {
        return {float(Math::Dot(d, tangent)), float(Math::Dot(d, bitangent)), float(Math::Dot(d, normal))};
    };
    const auto light = toTangent(sunDirection);
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Vector3 p = points[i];
        auto vertex = WorldToClip(p, colors[i]);
        const Vector3 offset = p - frame.origin;
        vertex.uv = {float(Math::Dot(offset, u) / SurfaceMaterial::tileMeters),
                     float(Math::Dot(offset, v) / SurfaceMaterial::tileMeters)};
        vertex.lighting = {light, toTangent(camera->GetPosition() - p)};
        vertices.push_back(vertex);
    }
}

void WorldRenderer::SubmitTerrain(const TerrainChunk& chunk)
{
    const MeshInstance& instance = chunk.Instance();
    if (chunk.material.empty() || !instance.mesh)
    {
        SubmitMesh(instance);
        return;
    }
    const Mesh& mesh = *instance.mesh;
    const auto start = vertices.size();
    for (size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        const auto& a = mesh.vertices[mesh.indices[i]];
        const auto& b = mesh.vertices[mesh.indices[i + 1]];
        const auto& c = mesh.vertices[mesh.indices[i + 2]];
        const std::array positions{instance.transform.Point(a.position), instance.transform.Point(b.position),
                                   instance.transform.Point(c.position)};
        SubmitMaterialTriangle(positions, {a.color, b.color, c.color}, instance.transform);
    }
    if (vertices.size() > start) materialRanges.push_back({start, vertices.size(), chunk.material});
}

void WorldRenderer::WireframeSubmittedGeometry()
{
    // Reserve UVs in this pass for two barycentric coordinates. Clipping and
    // perspective interpolation then preserve the edges of the original triangle.
    constexpr SDL_FPoint corners[] = {{1, 0}, {0, 1}, {0, 0}};
    wireframeEnd = vertices.size();
    for (size_t i = 0; i < wireframeEnd; ++i)
    {
        vertices[i].uv = corners[i % 3];
        vertices[i].color = EditorColors::wire;
    }
}

bool WorldRenderer::End()
{
    if (vertices.empty() || viewport.w <= 0 || viewport.h <= 0)
    {
        return true;
    }
    outlineStart = std::min(foregroundStart, vertices.size());
    vertices.insert(vertices.begin() + outlineStart, outline.begin(), outline.end());
    foregroundStart = outlineStart + outline.size();
    const SDL_FRect dest{0, 0, static_cast<float>(viewport.w), static_cast<float>(viewport.h)};
    if (!PrepareTarget(renderer, viewport.w, viewport.h)) return false;
    if (!wireframeEnd)
    {
        for (auto& range : materialRanges) range.material = materialCache.Get(range.path, device);
    }
    return (device ? DrawGPU() : DrawSoftware()) &&
           SDL_RenderTexture(renderer, colorTarget, nullptr, &dest);
}
