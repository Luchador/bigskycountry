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
    selectedRanges.clear();
    outlineStart = 0;
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

bool WorldRenderer::BoundsVisible(const Bounds3& bounds, const MeshTransform* placement) const
{
    if (bounds.Empty()) return true; // Unbounded editor overlays use normal clipping.
    // Reject only if every corner is outside the same plane. Transform corners
    // with the full instance matrix so mirrored and sheared placements work too.
    unsigned outside = 31;
    for (int i = 0; i < 8 && outside; ++i)
    {
        Vector3 p{(i & 1) ? bounds.max.x : bounds.min.x, (i & 2) ? bounds.max.y : bounds.min.y,
                  (i & 4) ? bounds.max.z : bounds.min.z};
        if (placement) p = placement->Point(p);
        const Vector3 v = camera->WorldToView(p);
        // Allow for rounding when world positions are packed into float clip coordinates.
        const double pad = 1e-6 * std::max(1.0, std::abs(v.z));
        const double x = v.x * scaleX, y = v.y * scaleY, farSide = v.z + pad;
        unsigned mask = 0;
        if (farSide < worldNearDepth) mask |= 1;
        if (x < -farSide) mask |= 2;
        if (x > farSide) mask |= 4;
        if (y < -farSide) mask |= 8;
        if (y > farSide) mask |= 16;
        outside &= mask;
    }
    return outside == 0;
}

bool WorldRenderer::SphereVisible(Vector3 center, double radius) const
{
    const Vector3 p = camera->WorldToView(center);
    // Clip-space side planes expressed in view meters, expanded by the sphere radius.
    return p.z + radius >= worldNearDepth &&
           std::abs(p.x * scaleX) <= p.z + radius * std::hypot(1.0, scaleX) &&
           std::abs(p.y * scaleY) <= p.z + radius * std::hypot(1.0, scaleY);
}

void WorldRenderer::SubmitMesh(const MeshInstance& instance, SDL_FColor tint)
{
    if (!instance.mesh || !BoundsVisible(instance.mesh->bounds, &instance.transform))
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

void WorldRenderer::SubmitMaterialMesh(const MeshInstance& instance, const std::string& material, MeshUVs mapping)
{
    if (material.empty() || !instance.mesh)
    {
        SubmitMesh(instance);
        return;
    }
    const Mesh& mesh = *instance.mesh;
    if (!BoundsVisible(mesh.bounds, &instance.transform)) return;
    const auto normalMatrix = instance.transform.NormalTransform();
    const auto worldNormal = [&](Vector3 n) {
        // Terrain placements are rigid. Object scale can be nonuniform or mirrored.
        if (mapping == MeshUVs::TerrainProjection) return instance.transform.Direction(n);
        return Math::Normalize(normalMatrix.Direction(n));
    };
    const bool mirrored = instance.transform.Determinant() < 0;
    const auto start = vertices.size();
    for (size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        const auto& a = mesh.vertices[mesh.indices[i]];
        const auto& b = mesh.vertices[mesh.indices[i + (mirrored ? 2 : 1)]];
        const auto& c = mesh.vertices[mesh.indices[i + (mirrored ? 1 : 2)]];
        const std::array normals{worldNormal(a.normal), worldNormal(b.normal), worldNormal(c.normal)};
        const std::array positions{instance.transform.Point(a.position), instance.transform.Point(b.position),
                                   instance.transform.Point(c.position)};
        const auto uv = mapping == MeshUVs::TerrainProjection ? TerrainUVs(positions, instance.transform) : std::array{a.uv, b.uv, c.uv};
        SubmitMaterialTriangle(positions, {a.color, b.color, c.color}, uv, &normals);
    }
    if (vertices.size() > start) materialRanges.push_back({start, vertices.size(), material});
}

void WorldRenderer::SubmitTerrain(const TerrainChunk& chunk)
{
    const MeshInstance& instance = chunk.Instance();
    if (chunk.material.empty() || !instance.mesh)
    {
        SubmitMesh(instance, chunk.boundary ? TerrainChunk::groundColor : SDL_FColor{1, 1, 1, 1});
        return;
    }
    SubmitMaterialMesh(instance, chunk.material, MeshUVs::TerrainProjection);
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

void WorldRenderer::BuildDrawRanges()
{
    draws.clear();
    struct TranslucentTriangle { float depth; DrawRange draw; };
    std::vector<TranslucentTriangle> transparent;
    std::vector<VertexRange> translucentSpans;
    auto material = materialRanges.begin();
    auto selection = selectedRanges.begin();
    for (size_t first = 0; first < outlineStart; )
    {
        size_t end = outlineStart;
        if (first < wireframeEnd) end = std::min(end, wireframeEnd);
        while (material != materialRanges.end() && material->end <= first) ++material;
        while (selection != selectedRanges.end() && selection->end <= first) ++selection;
        const SurfaceMaterial* surface = nullptr;
        if (!wireframeEnd && material != materialRanges.end())
        {
            const bool inside = first >= material->first;
            end = std::min(end, inside ? material->end : material->first);
            if (inside) surface = material->material;
        }
        const bool selected = selection != selectedRanges.end() && first >= selection->first;
        if (selection != selectedRanges.end()) end = std::min(end, selected ? selection->end : selection->first);
        if (surface && surface->type == MaterialType::Unlit)
        {
            translucentSpans.push_back({first, end});
            for (size_t i = first; i < end; i += 3)
            {
                const float depth = vertices[i].w + vertices[i + 1].w + vertices[i + 2].w;
                transparent.push_back({depth, {i, i + 3, surface, selected}});
            }
        }
        else draws.push_back({first, end, surface, selected});
        first = end;
    }
    if (transparent.empty()) return;
    // Clip W is camera-space depth. Blend far to near after every opaque surface.
    std::stable_sort(transparent.begin(), transparent.end(), [](const auto& a, const auto& b) { return a.depth > b.depth; });

    // Pack sorted triangles into the original transparent spans so a large translucent
    // road or terrain mesh can still be drawn in batches. Opaque and overlay vertices stay in place.
    std::vector<WorldDepthVertex> sortedVertices;
    sortedVertices.reserve(transparent.size() * 3);
    for (const auto& triangle : transparent)
    {
        sortedVertices.insert(sortedVertices.end(), vertices.begin() + triangle.draw.first, vertices.begin() + triangle.draw.end);
    }
    size_t triangle = 0;
    for (const auto& span : translucentSpans)
    {
        std::copy_n(sortedVertices.begin() + triangle * 3, span.end - span.first, vertices.begin() + span.first);
        for (size_t first = span.first; first < span.end; first += 3)
        {
            const auto& draw = transparent[triangle++].draw;
            if (!draws.empty() && draws.back().end == first && draws.back().material == draw.material &&
                draws.back().selected == draw.selected)
                draws.back().end = first + 3;
            else
                draws.push_back({first, first + 3, draw.material, draw.selected});
        }
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
        for (auto& range : materialRanges)
        {
            range.material = materialCache.Get(range.path, device);
            if (range.transitionOnly && range.material && range.material->type != MaterialType::Transition)
                range.material = nullptr;
        }
    }
    BuildDrawRanges();
    return (device ? DrawGPU() : DrawSoftware()) &&
           SDL_RenderTexture(renderer, colorTarget, nullptr, &dest);
}
