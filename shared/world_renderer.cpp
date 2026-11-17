#include "world_renderer.hpp"
#include "camera_view.hpp"
#include "terrain_chunk.hpp"
#include "editor_colors.hpp"
#include "view_geometry.hpp"
#include "spatial_index.hpp"
#include "project_settings.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

bool WorldRenderer::DrawCacheStatistics(SDL_Renderer* target) const
{
    const auto& settings = g_ProjectSettings.Streaming();
    constexpr double mib = StreamingSettings::MiB;
    const auto geometry = ResidentBytes(), textures = TextureBytes();
    const SDL_FRect box{8, 36, 735, 50};
    return SDL_SetRenderDrawColor(target, 24, 26, 30, 255) && SDL_RenderFillRect(target, &box) &&
        SDL_SetRenderDrawColor(target, 230, 232, 235, 255) &&
        SDL_RenderDebugTextFormat(target, 16, 44, "Render cache %.1f/%.0f MiB | Textures %.1f/%.0f MiB (%zu shared)",
            geometry / mib, settings.geometryBytes / mib, textures / mib, settings.assetBytes / (2 * mib), TextureCount()) &&
        SDL_RenderDebugText(target, 16, 58, geometry > settings.geometryBytes || textures > settings.assetBytes / 2 ?
            "Budget pressure: active resources pinned" : "Warm render resources retained within memory budgets") &&
        SDL_RenderDebugTextFormat(target, 16, 72, "Prepared %zu tris | Uploaded %.2f MiB this frame | %zu ready buffers",
            preparedVertices / 3, uploadedBytes / mib, ReadyBuffers());
}

std::size_t WorldRenderer::ReadyBuffers() const
{
    return std::count_if(resident.begin(), resident.end(), [](const auto& e) {
        return e.second->buffer && e.second->uploadedBytes == e.second->bufferBytes;
    });
}

std::size_t WorldRenderer::ResidentBytes() const
{
    std::size_t bytes = 0;
    for (const auto& [key, entry] : resident)
        bytes += entry->bufferBytes + entry->pendingUpload.capacity() * sizeof(WorldDepthVertex);
    return bytes;
}

bool WorldRenderer::Begin(SDL_Renderer* target, const CameraView& view, const WorldLighting* environment, const WorldFog* atmosphere, double distance)
{
    if (!SDL_GetRenderViewport(target, &viewport))
    {
        return false;
    }
    renderer = target;
    camera = &view;
    viewDistance = std::isfinite(distance) && distance > 0 ? distance : 0;
    viewport.x = viewport.y = 0;
    focal = std::min(viewport.w, viewport.h) * 0.5;
    scaleX = viewport.w > 0 ? 2.0 * focal / viewport.w : 0.0;
    scaleY = viewport.h > 0 ? 2.0 * focal / viewport.h : 0.0;
    ++frame;
    preparedVertices = uploadedBytes = preparationNanos = 0;
    residentDraws.clear();
    residentWire = false;
    // Looking away does not unload a warm buffer. Streamed cells explicitly call
    // Forget on unload; authored mesh owners can expire here. Other disposable
    // buffers remain in the bounded LRU cache until memory is needed.
    std::erase_if(resident, [&](const auto& entry) {
        const auto& geometry = *entry.second;
        return (geometry.meshOwned && geometry.mesh.expired()) ||
            (!geometry.buffer && geometry.pendingUpload.empty() && frame - geometry.lastFrame > 120);
    });
    // Editor objects, selections and history are never evicted. Only disposable
    // render buffers are reclaimed, and currently visible geometry stays pinned.
    auto residentBytes = ResidentBytes();
    const auto budget = g_ProjectSettings.Streaming().geometryBytes;
    while (residentBytes > budget)
    {
        auto victim = resident.end();
        for (auto it = resident.begin(); it != resident.end(); ++it)
            if (it->second->lastFrame + 1 < frame && (victim == resident.end() || it->second->lastFrame < victim->second->lastFrame)) victim = it;
        if (victim == resident.end()) break;
        residentBytes -= victim->second->bufferBytes + victim->second->pendingUpload.capacity() * sizeof(WorldDepthVertex);
        resident.erase(victim);
    }
    vertices.clear();
    outline.clear();
    materialRanges.clear();
    lightGeometry = environment != nullptr;
    lights = environment ? *environment : WorldLighting{};
    fog = atmosphere ? std::optional{*atmosphere} : std::nullopt;
    wireframeEnd = 0;
    foregroundStart = std::numeric_limits<std::size_t>::max();
    selectedRanges.clear();
    outlineStart = 0;
    if (viewport.w > 0 && viewport.h > 0 && !PrepareTarget(target, viewport.w, viewport.h)) return false;
    materialCache.BeginFrame(device, g_ProjectSettings.Streaming().assetBytes / 2);
    return true;
}

WorldDepthVertex WorldRenderer::WorldToClip(Vector3 position, SDL_FColor color) const
{
    if (packingOrigin)
    {
        const Vector3 local = position - *packingOrigin;
        WorldDepthVertex vertex{float(local.x), float(local.y), float(local.z), 1, color};
        // Reuse the view channel for the rounding residual of a resident vertex.
        vertex.view = local - Vector3{vertex.x, vertex.y, vertex.z};
        return vertex;
    }
    const Vector3 v = camera->WorldToView(position);
    WorldDepthVertex vertex{static_cast<float>(v.x * scaleX), static_cast<float>(v.y * scaleY),
                            static_cast<float>(worldNearDepth), static_cast<float>(v.z), color};
    // Every world surface needs a camera-relative position for pixel fog, even
    // untextured meshes. Keep the subtraction in double at continental distances.
    vertex.view = camera->GetPosition() - position;
    return vertex;
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
    if (viewDistance > 0 && foregroundStart == std::numeric_limits<size_t>::max() &&
        BoundsDistanceSquared(placement ? WorldBounds(bounds, *placement) : bounds, camera->GetPosition()) > viewDistance * viewDistance)
        return false;
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
    if (viewDistance > 0 && foregroundStart == std::numeric_limits<size_t>::max() &&
        Math::Length(center - camera->GetPosition()) > viewDistance + radius) return false;
    const Vector3 p = camera->WorldToView(center);
    // Clip-space side planes expressed in view meters, expanded by the sphere radius.
    return p.z + radius >= worldNearDepth &&
           std::abs(p.x * scaleX) <= p.z + radius * std::hypot(1.0, scaleX) &&
           std::abs(p.y * scaleY) <= p.z + radius * std::hypot(1.0, scaleY);
}

void WorldRenderer::SubmitMesh(const MeshInstance& instance, SDL_FColor tint)
{
    if (!instance.mesh || (!packingOrigin && !BoundsVisible(instance.mesh->bounds, &instance.transform)))
    {
        return;
    }
    const Mesh& mesh = *instance.mesh;
    if (lightGeometry && mesh.HasMaterials())
    {
        SubmitMaterialMesh(instance, {}, MeshUVs::Authored, tint);
        return;
    }
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
        vertices.push_back(vertex);
    }
}

void WorldRenderer::SubmitMaterialMesh(const MeshInstance& instance, const std::string& material, MeshUVs mapping, SDL_FColor tint)
{
    if (!instance.mesh) return;
    if (material.empty() && !instance.mesh->HasMaterials())
    {
        SubmitMesh(instance, tint);
        return;
    }
    const Mesh& mesh = *instance.mesh;
    if (!packingOrigin && !BoundsVisible(mesh.bounds, &instance.transform)) return;
    const auto normalMatrix = instance.transform.NormalTransform();
    const auto worldNormal = [&](Vector3 n) {
        // Terrain placements are rigid. Object scale can be nonuniform or mirrored.
        if (mapping == MeshUVs::TerrainProjection) return instance.transform.Direction(n);
        return Math::Normalize(normalMatrix.Direction(n));
    };
    const bool mirrored = instance.transform.Determinant() < 0;
    for (size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        const auto& a = mesh.vertices[mesh.indices[i]];
        const auto& b = mesh.vertices[mesh.indices[i + (mirrored ? 2 : 1)]];
        const auto& c = mesh.vertices[mesh.indices[i + (mirrored ? 1 : 2)]];
        const std::array normals{worldNormal(a.normal), worldNormal(b.normal), worldNormal(c.normal)};
        const std::array positions{instance.transform.Point(a.position), instance.transform.Point(b.position),
                                   instance.transform.Point(c.position)};
        const auto uv = mapping == MeshUVs::TerrainProjection ? TerrainUVs(positions, instance.transform) : std::array{a.uv, b.uv, c.uv};
        const auto start = vertices.size();
        const auto color = [&](SDL_FColor c) { return SDL_FColor{c.r * tint.r, c.g * tint.g, c.b * tint.b, c.a}; };
        SubmitMaterialTriangle(positions, {color(a.color), color(b.color), color(c.color)}, uv, &normals);
        const auto& path = material.empty() ? mesh.MaterialAt(i) : material;
        if (!path.empty())
        {
            if (!materialRanges.empty() && materialRanges.back().end == start && materialRanges.back().path == path)
                materialRanges.back().end = vertices.size();
            else materialRanges.push_back({start, vertices.size(), path});
        }
    }
}

bool WorldRenderer::CanRetain(const std::string& material)
{
    if (!device || !lightGeometry || foregroundStart != std::numeric_limits<size_t>::max()) return false;
    const auto* surface = material.empty() ? nullptr : materialCache.Get(material, device);
    // Translucent triangles still need global, camera-dependent sorting.
    return !surface || surface->type != MaterialType::Unlit;
}

void WorldRenderer::CaptureGeometry(ResidentGeometry& geometry, size_t start, size_t firstMaterial)
{
    packingOrigin.reset();
    geometry.pendingUpload.assign(vertices.begin() + start, vertices.end());
    size_t cursor = start;
    for (size_t i = firstMaterial; i < materialRanges.size(); ++i)
    {
        auto& part = materialRanges[i];
        if (cursor < part.first) geometry.parts.push_back({cursor - start, part.first - start, {}});
        cursor = part.end;
        part.first -= start;
        part.end -= start;
        geometry.parts.push_back(std::move(part));
    }
    if (cursor < vertices.size()) geometry.parts.push_back({cursor - start, vertices.size() - start, {}});
    vertices.resize(start);
    materialRanges.resize(firstMaterial);
    geometry.device = device;
    geometry.complete = true;
}

bool WorldRenderer::CanPrepare() const
{
    return preparedVertices < maxPreparedVertices && preparationNanos < maxPreparationNanos;
}

void WorldRenderer::PrepareMesh(ResidentGeometry& geometry, const MeshInstance& instance)
{
    if (geometry.complete || !CanPrepare()) return;
    const auto started = SDL_GetTicksNS();
    const auto& mesh = *instance.mesh;
    const auto normalMatrix = instance.transform.NormalTransform();
    const bool mirrored = instance.transform.Determinant() < 0;
    const bool textured = !geometry.material.empty() || mesh.HasMaterials();
    const auto worldNormal = [&](Vector3 normal) {
        return textured && geometry.mapping == MeshUVs::TerrainProjection ? instance.transform.Direction(normal) :
            Math::Normalize(normalMatrix.Direction(normal));
    };
    // Pack directly into the pending buffer. Preserve the dynamic stream without
    // copying already prepared triangles each time this mesh resumes.
    vertices.swap(geometry.pendingUpload);
    vertices.reserve(mesh.indices.size());
    packingOrigin = geometry.origin;
    while (vertices.size() < mesh.indices.size() && preparedVertices < maxPreparedVertices)
    {
        const auto count = std::min({mesh.indices.size() - vertices.size(),
            maxPreparedVertices - preparedVertices, std::size_t(256 * 3)});
        const auto end = vertices.size() + count;
        for (auto i = vertices.size(); i < end; i += 3)
        {
            const auto& a = mesh.vertices[mesh.indices[i]];
            const auto& b = mesh.vertices[mesh.indices[i + (mirrored ? 2 : 1)]];
            const auto& c = mesh.vertices[mesh.indices[i + (mirrored ? 1 : 2)]];
            const std::array positions{instance.transform.Point(a.position), instance.transform.Point(b.position),
                                       instance.transform.Point(c.position)};
            const std::array normals{worldNormal(a.normal), worldNormal(b.normal), worldNormal(c.normal)};
            if (textured)
            {
                const auto uv = geometry.mapping == MeshUVs::TerrainProjection ? TerrainUVs(positions, instance.transform) :
                    std::array{a.uv, b.uv, c.uv};
                const auto color = [&](SDL_FColor c) {
                    return SDL_FColor{c.r * geometry.tint.r, c.g * geometry.tint.g, c.b * geometry.tint.b, c.a};
                };
                SubmitMaterialTriangle(positions, {color(a.color), color(b.color), color(c.color)}, uv, &normals);
            }
            else
            {
                const std::array colors{a.color, b.color, c.color};
                for (std::size_t j = 0; j < 3; ++j)
                {
                    const auto color = colors[j], tint = geometry.tint;
                    auto vertex = WorldToClip(positions[j], {color.r * tint.r, color.g * tint.g, color.b * tint.b, 1});
                    vertex.normal = normals[j];
                    vertex.lit = 1;
                    vertices.push_back(vertex);
                }
            }
        }
        preparedVertices += count;
        if (preparationNanos + SDL_GetTicksNS() - started >= maxPreparationNanos) break;
    }
    packingOrigin.reset();
    vertices.swap(geometry.pendingUpload);
    preparationNanos += SDL_GetTicksNS() - started;
    geometry.complete = geometry.pendingUpload.size() == mesh.indices.size();
    if (geometry.complete && !geometry.pendingUpload.empty())
    {
        if (!geometry.material.empty() || !mesh.HasMaterials())
            geometry.parts.push_back({0, geometry.pendingUpload.size(), geometry.material});
        else for (const auto& part : mesh.parts)
        {
            const auto& path = mesh.MaterialAt(part.firstIndex);
            if (!geometry.parts.empty() && geometry.parts.back().end == part.firstIndex && geometry.parts.back().path == path)
                geometry.parts.back().end += part.indexCount;
            else geometry.parts.push_back({part.firstIndex, part.firstIndex + part.indexCount, path});
        }
    }
}

void WorldRenderer::SubmitWorldMesh(const MeshInstance& instance, const std::string& material,
                                    MeshUVs mapping, SDL_FColor tint)
{
    if (!instance.mesh || !BoundsVisible(instance.mesh->bounds, &instance.transform)) return;
    if (material.empty() && instance.mesh->HasMaterials()) mapping = MeshUVs::Authored;
    const auto emit = [&] {
        if (material.empty()) SubmitMesh(instance, tint);
        else SubmitMaterialMesh(instance, material, mapping, tint);
    };
    bool retain = CanRetain(material);
    if (retain && material.empty() && instance.mesh->materials)
        for (const auto& slot : instance.mesh->materials->slots) retain &= CanRetain(slot.material);
    if (!retain)
    {
        emit();
        return;
    }
    const auto& bounds = instance.mesh->bounds;
    auto& entry = resident[&instance];
    const bool sameMesh = entry && entry->mesh.lock() == instance.mesh;
    const auto revision = instance.mesh->materials ? instance.mesh->materials->revision : 0;
    if (sameMesh && entry->transform != instance.transform)
    {
        // Stream moving placements. Once stationary, bake the new transform once.
        entry->lastFrame = frame;
        entry->transform = instance.transform;
        entry->moving = true;
        emit();
        return;
    }
    if (!sameMesh || entry->moving || entry->revision != revision ||
        entry->material != material || entry->mapping != mapping ||
        entry->tint.r != tint.r || entry->tint.g != tint.g || entry->tint.b != tint.b)
    {
        entry = std::make_shared<ResidentGeometry>();
        entry->device = device;
        entry->meshOwned = true;
        entry->mesh = instance.mesh;
        entry->revision = revision;
        entry->transform = instance.transform;
        entry->material = material;
        entry->mapping = mapping;
        entry->tint = tint;
        entry->origin = bounds.Empty() ? instance.transform.origin : instance.transform.Point((bounds.min + bounds.max) * 0.5);
    }
    PrepareMesh(*entry, instance);
    entry->lastFrame = frame;
    if (!entry->parts.empty()) residentDraws.push_back({entry, vertices.size(), false});
}

void WorldRenderer::AppendSelectionVertex(SDL_FPoint point, double depth, double lift)
{
    const double x = (2.0 * point.x / viewport.w - 1.0) * depth;
    const double y = (1.0 - 2.0 * point.y / viewport.h) * depth;
    const double strokeDepth = std::max(worldNearDepth, depth - lift);
    const double biased = std::min(depth, worldNearDepth * 1.0001 * depth / strokeDepth);
    outline.push_back({static_cast<float>(x), static_cast<float>(y),
                       static_cast<float>(biased), static_cast<float>(depth), EditorColors::orange});
}

void WorldRenderer::SubmitTerrain(const TerrainChunk& chunk, bool selected)
{
    const auto& instance = chunk.Instance();
    if (selected && (!instance.mesh || !BoundsVisible(instance.mesh->bounds, &instance.transform))) return;
    const auto start = vertices.size(), firstResident = residentDraws.size();
    SubmitWorldMesh(chunk.Instance(), chunk.material, MeshUVs::TerrainProjection,
                    chunk.boundary ? TerrainChunk::groundColor : SDL_FColor{1, 1, 1, 1});
    if (chunk.HasWater()) SubmitWorldMesh(chunk.Water(), LakeDefinition::waterMaterial);
    if (!selected || viewport.w <= 0 || viewport.h <= 0) return;
    // Both cached GPU batches and streamed/software triangles participate in
    // the same depth/stencil selection mask used by roads and intersections.
    for (auto i = firstResident; i < residentDraws.size(); ++i) residentDraws[i].selected = true;
    if (start < vertices.size())
    {
        if (!selectedRanges.empty() && selectedRanges.back().end == start) selectedRanges.back().end = vertices.size();
        else selectedRanges.push_back({start, vertices.size()});
    }
    const auto sides = ViewGeometry::SidePlanes(focal, viewport);
    for (const auto& edge : chunk.Outline())
    {
        const auto a = camera->WorldToView(instance.transform.Point(edge.a));
        const auto b = camera->WorldToView(instance.transform.Point(edge.b));
        const auto stroke = ViewGeometry::ProjectStroke(a, b, sides, focal, viewport, worldNearDepth, 1.5f);
        if (!stroke) continue;
        const auto normal = camera->WorldDirectionToView(instance.transform.Direction(edge.normal));
        const double plane = Math::Dot(normal, a);
        for (int i : {0, 1, 2, 0, 2, 3})
        {
            const auto p = stroke->corners[i];
            const Vector3 ray{(p.x - viewport.w * 0.5) / focal, (viewport.h * 0.5 - p.y) / focal, 1};
            const double denominator = Math::Dot(normal, ray);
            const double surfaceDepth = denominator != 0 ? plane / denominator : 0;
            // The inward half of a thick line must follow its adjacent face.
            // A constant endpoint depth hides the far rim even on flat terrain.
            const double depth = std::isfinite(surfaceDepth) && surfaceDepth >= worldNearDepth ? surfaceDepth :
                                 (i < 2 ? stroke->startDepth : stroke->endDepth);
            AppendSelectionVertex(p, depth);
        }
    }
}

void WorldRenderer::WireframeSubmittedGeometry()
{
    // Reserve UVs in this pass for two barycentric coordinates. Clipping and
    // perspective interpolation then preserve the edges of the original triangle.
    constexpr SDL_FPoint corners[] = {{1, 0}, {0, 1}, {0, 0}};
    residentWire = true;
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
    if ((vertices.empty() && residentDraws.empty()) || viewport.w <= 0 || viewport.h <= 0)
    {
        return true;
    }
    outlineStart = std::min(foregroundStart, vertices.size());
    vertices.insert(vertices.begin() + outlineStart, outline.begin(), outline.end());
    foregroundStart = outlineStart + outline.size();
    const SDL_FRect dest{0, 0, static_cast<float>(viewport.w), static_cast<float>(viewport.h)};
    if (!wireframeEnd)
    {
        for (auto& range : materialRanges)
        {
            range.material = materialCache.Get(range.path, device);
            if (range.transitionOnly && range.material && range.material->type != MaterialType::Transition)
                range.material = nullptr;
        }
    }
    for (const auto& batch : residentDraws)
    {
        for (auto& part : batch.geometry->parts)
        {
            part.material = !residentWire && !part.path.empty() ? materialCache.Get(part.path, device) : nullptr;
            if (part.transitionOnly && part.material && part.material->type != MaterialType::Transition)
                part.material = nullptr;
        }
    }
    BuildDrawRanges();
    return (device ? DrawGPU() : DrawSoftware()) &&
           SDL_RenderTexture(renderer, colorTarget, nullptr, &dest);
}
