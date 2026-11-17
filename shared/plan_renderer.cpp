#include "plan_renderer.hpp"
#include "project_settings.hpp"
#include "terrain_chunk.hpp"
#include "view_geometry.hpp"
#include "spatial_index.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

#ifdef BIGSKY_WORLD_GPU
#include "plan_vert_dxil.hpp"
#include "plan_vert_spirv.hpp"
#include "plan_frag_dxil.hpp"
#include "plan_frag_spirv.hpp"
#endif

struct PlanRenderer::Impl
{
    struct Vertex { float x, z, lowX, lowZ; SDL_FColor color; };
    struct Batch
    {
        SDL_GPUDevice* device = nullptr;
        SDL_GPUBuffer* buffer = nullptr;
        ~Batch() { if (buffer) SDL_ReleaseGPUBuffer(device, buffer); }
        Vector3 origin;
        std::vector<Vertex> vertices; // Released after the GPU upload; retained for the SDL fallback.
        std::size_t count = 0;
        std::uint64_t projectedView = 0;
        std::vector<SDL_Vertex> projected;
        std::vector<int> indices;
        void Add(Vector3 position, SDL_FColor color)
        {
            const auto p = position - origin;
            const float x = float(p.x), z = float(p.z);
            vertices.push_back({x, z, float(p.x - x), float(p.z - z), color});
        }
    };
    using Part = std::unique_ptr<Batch>;
    struct Track { Part rails; std::vector<Part> ties; bool visible = false; };
    struct RoadEntry
    {
        std::uint64_t revision = 0, lastFrame = 0;
        Part pavement, strips, paint;
        std::vector<Track> track;
        Vector3 detailEye{};
        double detailDistance = -1;
    };
    struct TerrainEntry
    {
        std::weak_ptr<const Mesh> mesh;
        MeshTransform transform;
        std::uint64_t lastFrame = 0;
        Part surface;
    };
    std::unordered_map<const RoadGeometry*, RoadEntry> roads;
    std::unordered_map<const TerrainChunk*, TerrainEntry> terrain;
    std::vector<Batch*> draws;
    std::uint64_t frame = 0, view = 0;
    Vector3 center{};
    double scale = 0;
    SDL_Rect viewport{};
    SDL_Renderer* renderer = nullptr;
    SDL_GPUDevice* device = nullptr;
    SDL_GPUGraphicsPipeline* pipeline = nullptr;
    SDL_GPUTransferBuffer* upload = nullptr;
    Uint32 uploadBytes = 0;
    SDL_Texture* target = nullptr;
    Statistics stats;

    ~Impl()
    {
        if (target) SDL_DestroyTexture(target);
        if (upload) SDL_ReleaseGPUTransferBuffer(device, upload);
        if (pipeline) SDL_ReleaseGPUGraphicsPipeline(device, pipeline);
    }
    bool Visible(const Bounds3& bounds) const
    {
        if (bounds.Empty()) return true;
        const double halfX = viewport.w * 0.5 / scale, halfZ = viewport.h * 0.5 / scale;
        return bounds.min.x <= center.x + halfX && bounds.max.x >= center.x - halfX &&
               bounds.min.z <= center.z + halfZ && bounds.max.z >= center.z - halfZ;
    }
    bool Visible(Vector3 p, double radius) const
    {
        return std::abs(p.x - center.x) <= viewport.w * 0.5 / scale + radius &&
               std::abs(p.z - center.z) <= viewport.h * 0.5 / scale + radius;
    }
    template<class Emit> void Use(Part& part, Vector3 origin, Emit emit)
    {
        if (!part)
        {
            part = std::make_unique<Batch>();
            part->device = device;
            part->origin = origin;
            emit(*part);
            part->count = part->vertices.size();
            ++stats.rebuiltBatches;
        }
        if (part->count) draws.push_back(part.get());
    }
    bool PrepareGPU();
    bool DrawGPU();
    bool DrawSDL();
};

PlanRenderer::PlanRenderer() : impl(std::make_unique<Impl>()) {}
PlanRenderer::~PlanRenderer() = default;
const PlanRenderer::Statistics& PlanRenderer::Stats() const { return impl->stats; }

void PlanRenderer::Clear()
{
    impl = std::make_unique<Impl>();
}

bool PlanRenderer::Begin(SDL_Renderer* renderer, Vector3 center, double scale, SDL_Rect viewport)
{
    auto& c = *impl;
    c.stats = {};
    c.draws.clear();
    ++c.frame;
    if (c.center.x != center.x || c.center.z != center.z || c.scale != scale ||
        c.viewport.w != viewport.w || c.viewport.h != viewport.h) ++c.view;
    c.center = center;
    c.scale = scale;
    c.viewport = {0, 0, viewport.w, viewport.h};
    if (!c.renderer)
    {
        c.renderer = renderer;
#ifdef BIGSKY_WORLD_GPU
        if (SDL_strcmp(SDL_GetRendererName(renderer), "gpu") == 0)
        {
            c.device = SDL_GetGPURendererDevice(renderer);
            if (c.device && !c.PrepareGPU())
            {
                SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Using cached SDL plan geometry: %s", SDL_GetError());
                c.device = nullptr;
            }
        }
#endif
    }
    // Do not keep hidden/deleted geometry forever. Edits replace entries immediately.
    std::erase_if(c.roads, [&](const auto& item) { return c.frame - item.second.lastFrame > 120; });
    std::erase_if(c.terrain, [&](const auto& item) { return c.frame - item.second.lastFrame > 120; });
    return c.renderer == renderer && scale > 0 && std::isfinite(scale);
}

void PlanRenderer::Road(const RoadGeometry& road, RoadLayer layer)
{
    auto& c = *impl;
    if (!c.Visible(road.bounds) || layer == RoadLayer::Outline) return;
    auto& entry = c.roads[&road];
    if (entry.revision != road.revision || (!road.revision && entry.lastFrame != c.frame))
    {
        entry = {};
        entry.revision = road.revision;
    }
    entry.lastFrame = c.frame;
    const Vector3 origin = road.bounds.Empty() ? Vector3{} : (road.bounds.min + road.bounds.max) * 0.5;
    const auto colored = [](Impl::Batch& batch, const auto& triangles) {
        for (const auto& t : triangles) for (const auto& v : t) batch.Add(v.position, v.color);
    };
    if (layer == RoadLayer::Stripes)
    {
        c.Use(entry.paint, origin, [&](auto& batch) {
            for (const auto& t : road.yellowStripes) for (auto p : t) batch.Add(p, {0.96f, 0.85f, 0.12f, 1});
            for (const auto& t : road.whiteStripes) for (auto p : t) batch.Add(p, {0.9f, 0.9f, 0.9f, 1});
        });
        return;
    }
    c.Use(entry.pavement, origin, [&](auto& batch) {
        for (const auto& t : road.pavementTriangles) for (const auto& v : t) batch.Add(v.position, {0.43f, 0.43f, 0.43f, 1});
        colored(batch, road.edgeTriangles);
    });
    // Preserve the existing detail-distance and subpixel tie culling. Each run
    // is expanded only once when first visible, rather than on every frame.
    entry.track.resize(road.railroadSegments.size());
    const Vector3 eye = c.center + Vector3{0, 0.5 * std::min(c.viewport.w, c.viewport.h) / c.scale, 0};
    const double distance = g_ProjectSettings.RailroadDetailDistance();
    const bool changedDetailView = entry.detailEye != eye || entry.detailDistance != distance;
    for (std::size_t i = 0; i < road.railroadSegments.size(); ++i)
    {
        const auto& segment = road.railroadSegments[i];
        auto& track = entry.track[i];
        if (changedDetailView) track.visible = segment.WithinDistance(eye, distance);
        if (!track.visible) continue;
        track.ties.resize(segment.ties.size());
        if (c.scale * RailroadSettings::tieSpacing >= 0.5)
        {
            for (std::size_t j = 0; j < segment.ties.size(); ++j)
            {
                const auto& run = segment.ties[j];
                if (!c.Visible(run.Center(), run.Radius())) continue;
                c.Use(track.ties[j], run.Center(), [&](auto& batch) {
                    const auto& mesh = *RailroadTieMesh();
                    for (int k = 0; k < run.count; ++k)
                    {
                        const auto tie = run.At(k);
                        for (int corner : {0, 1, 2, 0, 2, 3})
                            batch.Add(tie.Point(mesh.vertices[corner].position), {0.36f, 0.27f, 0.20f, 1});
                    }
                });
            }
        }
        c.Use(track.rails, origin, [&](auto& batch) { colored(batch, segment.rails); });
    }
    entry.detailEye = eye;
    entry.detailDistance = distance;
    c.Use(entry.strips, origin, [&](auto& batch) {
        for (const auto& [path, triangles] : road.transitionStrips) colored(batch, triangles);
    });
}

void PlanRenderer::Terrain(const TerrainChunk& chunk)
{
    const auto& instance = chunk.HasWater() ? chunk.Water() : chunk.Instance();
    if (!instance.mesh) return;
    auto& c = *impl;
    const auto worldBounds = WorldBounds(instance.mesh->bounds, instance.transform);
    if (!c.Visible(worldBounds)) return;
    auto& entry = c.terrain[&chunk];
    if (entry.mesh.lock() != instance.mesh || entry.transform != instance.transform)
    {
        entry = {};
        entry.mesh = instance.mesh;
        entry.transform = instance.transform;
    }
    entry.lastFrame = c.frame;
    const Vector3 origin = (worldBounds.min + worldBounds.max) * 0.5;
    c.Use(entry.surface, origin, [&](auto& batch) {
        batch.vertices.reserve(instance.mesh->indices.size());
        for (auto index : instance.mesh->indices)
            batch.Add(instance.transform.Point(instance.mesh->vertices[index].position),
                      chunk.HasWater() ? SDL_FColor{0.12f, 0.37f, 0.45f, 1} : TerrainChunk::groundColor);
    });
}

bool PlanRenderer::Impl::DrawSDL()
{
    const auto planes = ViewGeometry::SidePlanes(scale, viewport);
    for (auto* batch : draws)
    {
        if (batch->projectedView != view)
        {
            batch->projected.clear();
            const auto offset = batch->origin - center;
            using V = RoadGeometry::EdgeVertex;
            for (std::size_t i = 0; i < batch->vertices.size(); i += 3)
            {
                ViewGeometry::VertexPolygon<V> p{};
                for (std::size_t j = i; j < i + 3; ++j)
                {
                    const auto& v = batch->vertices[j];
                    p.points[p.count++] = {{(double(v.x) + offset.x) + v.lowX, (double(v.z) + offset.z) + v.lowZ, 1}, v.color};
                }
                for (auto normal : planes)
                {
                    if (p.count < 3) break;
                    p = ViewGeometry::ClipPolygon(p, [&](const V& v) { return Math::Dot(v.position, normal); },
                        [](const V& a, const V& b, double t) {
                            const float f = float(t);
                            return V{a.position + (b.position - a.position) * t,
                                {std::lerp(a.color.r, b.color.r, f), std::lerp(a.color.g, b.color.g, f),
                                 std::lerp(a.color.b, b.color.b, f), std::lerp(a.color.a, b.color.a, f)}};
                        });
                }
                for (int j = 2; j < p.count; ++j) for (int k : {0, j - 1, j})
                    batch->projected.push_back({ViewGeometry::ScreenPoint(p.points[k].position, scale, viewport), p.points[k].color, {}});
            }
            if (batch->projected.size() > std::numeric_limits<int>::max()) return SDL_SetError("Plan geometry exceeds SDL vertex limit");
            // Separate corner indices prevent SDL's software rectangle optimizer
            // from joining terrain triangles across a diagonal.
            batch->indices.resize(batch->projected.size());
            std::iota(batch->indices.begin(), batch->indices.end(), 0);
            batch->projectedView = view;
            stats.projectedVertices += batch->vertices.size();
        }
        if (batch->projected.empty()) continue;
        const int count = static_cast<int>(batch->projected.size());
        if (!SDL_RenderGeometry(renderer, nullptr, batch->projected.data(), count, batch->indices.data(), count)) return false;
        ++stats.drawCalls;
    }
    return true;
}

#ifdef BIGSKY_WORLD_GPU
bool PlanRenderer::Impl::PrepareGPU()
{
    const auto shader = [&](SDL_GPUShaderStage stage, const char* entry, std::span<const Uint8> dxil, std::span<const Uint8> spirv) {
        SDL_GPUShaderCreateInfo info{};
        info.stage = stage;
        info.entrypoint = entry;
        info.num_uniform_buffers = stage == SDL_GPU_SHADERSTAGE_VERTEX ? 1 : 0;
        const bool vulkan = SDL_GetGPUShaderFormats(device) & SDL_GPU_SHADERFORMAT_SPIRV;
        info.format = vulkan ? SDL_GPU_SHADERFORMAT_SPIRV : SDL_GPU_SHADERFORMAT_DXIL;
        const auto bytes = vulkan ? spirv : dxil;
        info.code = bytes.data();
        info.code_size = bytes.size();
        return SDL_CreateGPUShader(device, &info);
    };
    auto* vs = shader(SDL_GPU_SHADERSTAGE_VERTEX, "VSMain", plan_vert_dxil, plan_vert_spirv);
    if (!vs) return false;
    auto* ps = shader(SDL_GPU_SHADERSTAGE_FRAGMENT, "PSMain", plan_frag_dxil, plan_frag_spirv);
    if (!ps) { SDL_ReleaseGPUShader(device, vs); return false; }
    const SDL_GPUVertexBufferDescription layout{0, sizeof(Vertex), SDL_GPU_VERTEXINPUTRATE_VERTEX, 0};
    const SDL_GPUVertexAttribute attributes[] = {{0, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, 0},
                                                {1, 0, SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, offsetof(Vertex, color)}};
    SDL_GPUColorTargetDescription color{};
    color.format = SDL_GetGPUTextureFormatFromPixelFormat(SDL_PIXELFORMAT_RGBA32);
    SDL_GPUGraphicsPipelineCreateInfo info{};
    info.vertex_shader = vs;
    info.fragment_shader = ps;
    info.vertex_input_state = {&layout, 1, attributes, SDL_arraysize(attributes)};
    info.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    info.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
    info.target_info.color_target_descriptions = &color;
    info.target_info.num_color_targets = 1;
    pipeline = SDL_CreateGPUGraphicsPipeline(device, &info);
    SDL_ReleaseGPUShader(device, vs);
    SDL_ReleaseGPUShader(device, ps);
    return pipeline != nullptr;
}

bool PlanRenderer::Impl::DrawGPU()
{
    // Submit SDL's previous use of the target before resizing/cycling it or
    // issuing independent GPU commands. No readback or CPU/GPU wait is needed.
    if (!SDL_FlushRenderer(renderer)) return false;
    if (!target || target->w != viewport.w || target->h != viewport.h)
    {
        auto* next = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_TARGET, viewport.w, viewport.h);
        if (!next) return false;
        if (!SDL_SetTextureBlendMode(next, SDL_BLENDMODE_BLEND) || !SDL_SetTextureScaleMode(next, SDL_SCALEMODE_NEAREST))
        {
            SDL_DestroyTexture(next);
            return false;
        }
        if (target) SDL_DestroyTexture(target);
        target = next;
    }
    struct Upload { Batch* batch; Uint32 offset, size; };
    std::vector<Upload> pending;
    std::unordered_set<Batch*> queued;
    Uint32 total = 0;
    for (auto* batch : draws)
    {
        if (batch->vertices.empty()) continue;
        if (!queued.insert(batch).second) continue;
        if (batch->vertices.size() > (std::numeric_limits<Uint32>::max() - total) / sizeof(Vertex))
            return SDL_SetError("Plan upload exceeds GPU buffer limit");
        const auto size = static_cast<Uint32>(batch->vertices.size() * sizeof(Vertex));
        const SDL_GPUBufferCreateInfo info{SDL_GPU_BUFFERUSAGE_VERTEX, size, 0};
        if (!batch->buffer) batch->buffer = SDL_CreateGPUBuffer(device, &info);
        if (!batch->buffer) return false;
        pending.push_back({batch, total, size});
        total += size;
    }
    if (total > uploadBytes)
    {
        const SDL_GPUTransferBufferCreateInfo info{SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, total, 0};
        auto* next = SDL_CreateGPUTransferBuffer(device, &info);
        if (!next) return false;
        if (upload) SDL_ReleaseGPUTransferBuffer(device, upload);
        upload = next;
        uploadBytes = total;
    }
    if (total)
    {
        auto* data = static_cast<Uint8*>(SDL_MapGPUTransferBuffer(device, upload, true));
        if (!data) return false;
        for (const auto& item : pending) std::memcpy(data + item.offset, item.batch->vertices.data(), item.size);
        SDL_UnmapGPUTransferBuffer(device, upload);
    }
    SDL_GPUColorTargetInfo color{};
    color.texture = static_cast<SDL_GPUTexture*>(SDL_GetPointerProperty(SDL_GetTextureProperties(target), SDL_PROP_TEXTURE_GPU_TEXTURE_POINTER, nullptr));
    color.load_op = SDL_GPU_LOADOP_CLEAR;
    color.store_op = SDL_GPU_STOREOP_STORE;
    color.cycle = true;
    if (!color.texture) return SDL_SetError("Plan target has no GPU texture");
    auto* commands = SDL_AcquireGPUCommandBuffer(device);
    if (!commands) return false;
    if (total)
    {
        auto* copy = SDL_BeginGPUCopyPass(commands);
        if (!copy) { SDL_CancelGPUCommandBuffer(commands); return false; }
        for (const auto& item : pending)
        {
            const SDL_GPUTransferBufferLocation from{upload, item.offset};
            const SDL_GPUBufferRegion to{item.batch->buffer, 0, item.size};
            SDL_UploadToGPUBuffer(copy, &from, &to, false);
        }
        SDL_EndGPUCopyPass(copy);
    }
    auto* pass = SDL_BeginGPURenderPass(commands, &color, 1, nullptr);
    if (!pass) { SDL_CancelGPUCommandBuffer(commands); return false; }
    SDL_BindGPUGraphicsPipeline(pass, pipeline);
    for (const auto* batch : draws)
    {
        const auto offset = batch->origin - center;
        const float x = float(offset.x), z = float(offset.z);
        const float camera[8]{x, z, float(offset.x - x), float(offset.z - z),
                              float(2 * scale / viewport.w), float(2 * scale / viewport.h), 0, 0};
        SDL_PushGPUVertexUniformData(commands, 0, camera, sizeof(camera));
        const SDL_GPUBufferBinding binding{batch->buffer, 0};
        SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
        SDL_DrawGPUPrimitives(pass, static_cast<Uint32>(batch->count), 1, 0, 0);
        ++stats.drawCalls;
    }
    SDL_EndGPURenderPass(pass);
    if (!SDL_SubmitGPUCommandBuffer(commands)) return false;
    for (const auto& item : pending) std::vector<Vertex>().swap(item.batch->vertices);
    stats.uploadedBytes = total;
    const SDL_FRect dest{0, 0, float(viewport.w), float(viewport.h)};
    return SDL_RenderTexture(renderer, target, nullptr, &dest);
}
#else
bool PlanRenderer::Impl::PrepareGPU() { return false; }
bool PlanRenderer::Impl::DrawGPU() { return false; }
#endif

bool PlanRenderer::End()
{
    if (impl->draws.empty() || impl->viewport.w <= 0 || impl->viewport.h <= 0) return true;
    return impl->device ? impl->DrawGPU() : impl->DrawSDL();
}
