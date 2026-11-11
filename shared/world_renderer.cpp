#include "world_renderer.hpp"
#include "free_camera.hpp"
#include "terrain_chunk.hpp"
#include "editor_colors.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

bool WorldRenderer::Begin(SDL_Renderer* target, const FreeCamera& view, const WorldLighting* environment)
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
    lightGeometry = environment != nullptr;
    lights = environment ? *environment : WorldLighting{};
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
    SubmitTriangle({a, b, c}, {color, color, color});
}

void WorldRenderer::SubmitTriangle(const std::array<Vector3, 3>& positions, const std::array<SDL_FColor, 3>& colors)
{
    Vector3 normal{};
    if (lightGeometry) normal = Math::Normalize(Math::Cross(positions[1] - positions[0], positions[2] - positions[0]));
    for (std::size_t i = 0; i < positions.size(); ++i)
    {
        auto vertex = WorldToClip(positions[i], colors[i]);
        vertex.normal = normal;
        vertex.lit = lightGeometry;
        vertices.push_back(vertex);
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
    const auto normals = instance.transform.NormalTransform();
    for (const MeshVertex& v : mesh.vertices)
    {
        const SDL_FColor color{v.color.r * tint.r, v.color.g * tint.g, v.color.b * tint.b, 1};
        auto vertex = WorldToClip(instance.transform.Point(v.position), color);
        if (lightGeometry)
        {
            // glTF vertex colors and base color factors are already linear.
            vertex.normal = Math::Normalize(normals.Direction(v.normal));
            vertex.lit = 1;
        }
        projectedMesh.push_back(vertex);
    }
    const bool mirrored = instance.transform.Determinant() < 0;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        vertices.push_back(projectedMesh[mesh.indices[i]]);
        vertices.push_back(projectedMesh[mesh.indices[i + (mirrored ? 2 : 1)]]);
        vertices.push_back(projectedMesh[mesh.indices[i + (mirrored ? 1 : 2)]]);
    }
}

namespace
{
    // Terrain uses a four-meter tile projected onto the dominant local plane.
    std::array<SDL_FPoint, 3> TerrainUVs(const std::array<Vector3, 3>& points, const MeshTransform& frame)
    {
        const Vector3 local = frame.Unrotate(Math::Cross(points[1] - points[0], points[2] - points[0]));
        Vector3 u = frame.x, v = frame.z;
        if (std::abs(local.x) > std::abs(local.y) && std::abs(local.x) >= std::abs(local.z)) u = frame.y;
        else if (std::abs(local.z) > std::abs(local.y)) v = frame.y;
        std::array<SDL_FPoint, 3> uv;
        for (size_t i = 0; i < points.size(); ++i)
        {
            const Vector3 offset = points[i] - frame.origin;
            uv[i] = {float(Math::Dot(offset, u) / SurfaceMaterial::tileMeters),
                     float(Math::Dot(offset, v) / SurfaceMaterial::tileMeters)};
        }
        return uv;
    }
}

void WorldRenderer::SubmitMaterialTriangle(const std::array<Vector3, 3>& points,
                                           const std::array<SDL_FColor, 3>& colors, const std::array<SDL_FPoint, 3>& uv,
                                           const std::array<Vector3, 3>* smoothNormals)
{
    // Derive the normal-map axes from the same UVs all material maps sample.
    const Vector3 e1 = points[1] - points[0], e2 = points[2] - points[0];
    const Vector3 normal = Math::Normalize(Math::Cross(e1, e2));
    const double du1 = double(uv[1].x) - uv[0].x, dv1 = double(uv[1].y) - uv[0].y;
    const double du2 = double(uv[2].x) - uv[0].x, dv2 = double(uv[2].y) - uv[0].y;
    const double determinant = du1 * dv2 - du2 * dv1;
    const float handedness = determinant < 0.0 ? -1.0f : 1.0f;
    const Vector3 tangent = Math::Normalize(determinant == 0.0 ? e1 : (e1 * dv2 - e2 * dv1) * handedness);
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Vector3 p = points[i];
        auto vertex = WorldToClip(p, colors[i]);
        vertex.uv = uv[i];
        vertex.normal = smoothNormals ? (*smoothNormals)[i] : normal;
        vertex.lit = lightGeometry;
        vertex.tangent = tangent;
        vertex.handedness = handedness;
        vertex.view = camera->GetPosition() - p;
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
        // Terrain transforms are rigid, so rotating the authored normals preserves length.
        const std::array normals{instance.transform.Direction(a.normal), instance.transform.Direction(b.normal),
                                 instance.transform.Direction(c.normal)};
        SubmitMaterialTriangle(positions, {a.color, b.color, c.color}, TerrainUVs(positions, instance.transform), &normals);
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
