#include "world_renderer.hpp"
#include "free_camera.hpp"

#include <algorithm>

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
    pavement.clear();
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

bool WorldRenderer::End()
{
    if (vertices.empty() || viewport.w <= 0 || viewport.h <= 0)
    {
        return true;
    }
    outlineStart = vertices.size();
    vertices.insert(vertices.end(), outline.begin(), outline.end());
    const SDL_FRect dest{0, 0, static_cast<float>(viewport.w), static_cast<float>(viewport.h)};
    if (!PrepareTarget(renderer, viewport.w, viewport.h)) return false;
    drawAsphalt = !pavement.empty() && asphalt.Prepare(device);
    return (device ? DrawGPU() : DrawSoftware()) &&
           SDL_RenderTexture(renderer, colorTarget, nullptr, &dest);
}
