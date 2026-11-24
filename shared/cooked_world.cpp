#include "cooked_world.hpp"
#include "mesh_ao.hpp"
#include "world_renderer.hpp"
#include "world_collision.hpp"
#include "camera_view.hpp"
#include "project_settings.hpp"
#include "spatial_index.hpp"
#include "marker.hpp"
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <set>
#include <limits>

namespace
{
using namespace WorldPackage;
std::size_t MeshBytes(const std::shared_ptr<const Mesh> &mesh)
{
    if (!mesh) return 0;
    auto bytes = sizeof(Mesh) + mesh->vertices.capacity() * sizeof(MeshVertex) + mesh->vertexColors.capacity() * sizeof(SDL_FColor) +
        mesh->indices.capacity() * sizeof(std::uint32_t) + mesh->parts.capacity() * sizeof(MeshPart);
    bytes += mesh->assetPath.capacity() + mesh->lods.capacity() * sizeof(Mesh::LOD);
    for (const auto& lod : mesh->lods)
    {
        const auto& level = *lod.mesh;
        bytes += sizeof(Mesh) + level.assetPath.capacity() + level.vertices.capacity() * sizeof(MeshVertex) + level.vertexColors.capacity() * sizeof(SDL_FColor) +
            level.indices.capacity() * sizeof(std::uint32_t) + level.parts.capacity() * sizeof(MeshPart);
    }
    if (mesh->ambientOcclusion)
    {
        bytes += sizeof(MeshAOState) + mesh->ambientOcclusion->file.capacity();
        if (const auto ao = MeshAO::Current(*mesh))
        {
            bytes += sizeof(MeshAOData) + ao->alpha.capacity() * sizeof(std::vector<float>);
            for (const auto& level : ao->alpha) bytes += level.capacity() * sizeof(float);
        }
    }
    if (mesh->collision)
    {
        bytes += sizeof(MeshCollisionState) + mesh->collision->file.capacity();
        const auto collision = MeshCollision::Current(*mesh);
        bytes += sizeof(MeshCollisionData) + collision->shapes.capacity() * sizeof(CollisionShape);
    }
    if (mesh->materials)
    {
        bytes += sizeof(MeshMaterials) + mesh->materials->slots.capacity() * sizeof(MeshMaterialSlot) + mesh->materials->file.capacity();
        for (const auto& slot : mesh->materials->slots) bytes += slot.name.capacity() + slot.material.capacity();
    }
    return bytes;
}
std::size_t CellBytes(const Cell &cell)
{
    std::size_t bytes = sizeof(Cell) + cell.batches.capacity() * sizeof(Batch) + cell.ties.capacity() * sizeof(Ties);
    for (const auto &b : cell.batches) bytes += MeshBytes(b.nearMesh.mesh) + b.material.capacity();
    return bytes;
}
double Nearest(const Bounds3 &bounds, std::span<const Vector3> points)
{
    double result = std::numeric_limits<double>::infinity();
    for (auto p : points) result = std::min(result, BoundsDistanceSquared(bounds, p));
    return result;
}
constexpr std::uint64_t modelCollisionBit = std::uint64_t(1) << 63;
} // namespace

struct CookedWorld::Impl
{
    Reader reader;
    SpatialIndex cellsIndex, objectsIndex, objectCollisionIndex;
    Statistics stats;
    bool ready = false, attached = false;
    std::string error;
    struct Render
    {
        PayloadKind lod;
        std::unique_ptr<Cell> cell;
        std::size_t bytes;
    };
    struct Instance
    {
        MeshInstance mesh;
        std::unique_ptr<Marker> marker;
    };
    std::map<std::size_t, Render> renders;
    std::map<std::size_t, std::shared_ptr<const WorldCollision>> colliders, instanceColliders;
    std::map<std::string, std::shared_ptr<const Mesh>> models;
    std::map<std::string, std::size_t> modelSizes; // Admission estimates survive eviction, preventing reload churn.
    std::map<std::pair<std::size_t, PayloadKind>, std::size_t> cellSizes;
    std::map<std::size_t, Instance> instances;
    struct Want
    {
        double distance;
        PayloadKind lod;
        bool protect;
    };
    std::map<std::size_t, Want> wantRender;
    std::set<std::size_t> wantCollision, wantObjects, collideObjects;
    std::set<std::pair<int, std::size_t>> failed;
    std::set<std::string> failedModels;
    enum class Kind
    {
        Near,
        Far,
        Collision,
        Model,
        InstanceCollision
    };
    struct Request
    {
        Kind kind;
        std::size_t index;
        std::shared_ptr<const Mesh> mesh; // Only an instance-collision job retains its shared asset.
        auto Key() const { return std::pair{static_cast<int>(kind), index}; }
    };
    struct Result
    {
        Request request;
        std::unique_ptr<Cell> cell;
        std::shared_ptr<const Mesh> mesh;
        std::shared_ptr<const WorldCollision> collision;
        std::string error;
    };
    std::mutex mutex;
    std::condition_variable condition;
    std::optional<Request> job;
    std::optional<Result> result;
    std::optional<Request> pending;
    bool stop = false;
    std::thread worker;
    ~Impl()
    {
        {
            std::lock_guard lock(mutex);
            stop = true;
            job.reset();
        }
        condition.notify_one();
        if (worker.joinable()) worker.join();
    }
    void Run()
    {
        for (;;)
        {
            Request request;
            {
                std::unique_lock lock(mutex);
                condition.wait(lock, [&] { return stop || job.has_value(); });
                if (stop) return;
                request = std::move(*job);
                job.reset();
            }
            Result done;
            done.request = request;
            try
            {
                bool ok = true;
                if (request.kind == Kind::Near || request.kind == Kind::Far)
                {
                    done.cell = std::make_unique<Cell>();
                    ok = reader.ReadRender(request.index,
                                           request.kind == Kind::Near ? PayloadKind::Near : PayloadKind::Distant,
                                           *done.cell);
                }
                else if (request.kind == Kind::Model)
                {
                    const auto &path = reader.Index().entities[request.index].model;
                    // Main-thread cache deduplicates requests; the worker does not retain hidden strong references.
                    MeshAssets assets(AssetRoot::Cooked);
                    done.mesh = assets.Load(path);
                    ok = done.mesh != nullptr;
                }
                else
                {
                    MeshInstance mesh;
                    if (request.kind == Kind::Collision)
                        ok = reader.ReadCollision(request.index, mesh);
                    else
                        mesh = {request.mesh, reader.Index().entities[request.index].transform};
                    if (ok)
                    {
                        auto collider = std::make_shared<WorldCollision>();
                        collider->AddMesh(mesh);
                        collider->Finish();
                        done.collision = std::move(collider);
                    }
                }
                if (!ok) done.error = SDL_GetError();
            }
            catch (const std::exception &e)
            {
                done.error = e.what();
            }
            {
                std::lock_guard lock(mutex);
                if (stop) return;
                result = std::move(done);
            }
        }
    }
    bool Wanted(const Request &r) const
    {
        switch (r.kind)
        {
        case Kind::Near:
        case Kind::Far: {
            auto it = wantRender.find(r.index);
            return it != wantRender.end() &&
                   it->second.lod == (r.kind == Kind::Near ? PayloadKind::Near : PayloadKind::Distant);
        }
        case Kind::Collision:
            return wantCollision.contains(r.index);
        case Kind::Model:
            for (auto i : wantObjects)
                if (reader.Index().entities[i].model == reader.Index().entities[r.index].model) return true;
            return false;
        case Kind::InstanceCollision:
            return collideObjects.contains(r.index);
        }
        return false;
    }
    void Forget(const Render &r, WorldRenderer *renderer)
    {
        if (!renderer) return;
        for (const auto &batch : r.cell->batches) renderer->Forget(&batch.nearMesh);
        for (const auto &tie : r.cell->ties) renderer->Forget(&tie.run);
    }
    void RemoveInstance(std::size_t id, WorldRenderer *renderer)
    {
        const auto it = instances.find(id);
        if (it == instances.end()) return;
        if (renderer) renderer->Forget(&it->second.mesh);
        instances.erase(it);
    }
    void Measure()
    {
        stats.nearCells = stats.distantCells = stats.geometryBytes = stats.modelBytes = stats.collisionBytes = 0;
        for (const auto &[i, r] : renders)
        {
            (r.lod == PayloadKind::Near ? stats.nearCells : stats.distantCells)++;
            stats.geometryBytes += r.bytes;
        }
        for (const auto &[path, mesh] : models) stats.modelBytes += MeshBytes(mesh);
        for (const auto *group : {&colliders, &instanceColliders})
            for (const auto &[i, c] : *group) stats.collisionBytes += c->MemoryBytes();
        stats.collisionChunks = colliders.size() + instanceColliders.size();
        stats.modelAssets = models.size();
        stats.modelInstances = instances.size();
        stats.queued = pending.has_value();
    }
};

CookedWorld::CookedWorld() : impl(std::make_unique<Impl>()) {}
CookedWorld::~CookedWorld() = default;
bool CookedWorld::Ready() const
{
    return impl->ready;
}
const CookedWorld::Statistics &CookedWorld::Stats() const
{
    return impl->stats;
}
const std::string &CookedWorld::Error() const
{
    return impl->error;
}
const std::vector<Spawn> &CookedWorld::Spawns() const
{
    return impl->reader.Index().spawns;
}
const TrafficNetwork& CookedWorld::Traffic() const
{
    return impl->reader.Index().traffic;
}
bool CookedWorld::Load(const WorldDocument &source)
{
    return LoadPath(source.Path());
}
bool CookedWorld::LoadPath(const std::string &source)
{
    // No worker survives a catalog replacement, and no old completion can publish into it.
    impl = std::make_unique<Impl>();
    WorldDocument pathOnly(AssetRoot::Cooked);
    if (!pathOnly.OpenPath(source.c_str())) return false;
    auto &s = *impl;
    if (!s.reader.Open(DefaultPath(pathOnly)) || !s.reader.Matches(source, AssetRoot::Cooked)) return false;
    s.cellsIndex.BeginSync();
    s.objectsIndex.BeginSync();
    s.objectCollisionIndex.BeginSync();
    const auto &index = s.reader.Index();
    for (std::size_t i = 0; i < index.cells.size(); ++i)
        s.cellsIndex.Upsert({SceneKind::Terrain, static_cast<int>(i)}, index.cells[i].bounds);
    for (std::size_t i = 0; i < index.entities.size(); ++i)
    {
        const auto &e = index.entities[i];
        if ((e.id.kind == SceneKind::Object || e.id.kind == SceneKind::Mesh) && !e.model.empty())
        {
            auto bounds = e.type == ObjectType::Marker ? Marker::VisibilityBounds(e.bounds, e.markerText, e.pose.scale)
                                                       : e.bounds;
            s.objectsIndex.Upsert({SceneKind::Mesh, static_cast<int>(i)}, bounds);
            if (e.hasCollision)
                s.objectCollisionIndex.Upsert({SceneKind::Mesh, static_cast<int>(i)}, e.bounds);
        }
    }
    s.cellsIndex.EndSync();
    s.objectsIndex.EndSync();
    s.objectCollisionIndex.EndSync();
    s.stats.totalCells = index.cells.size();
    s.ready = true;
    s.worker = std::thread([&s] { s.Run(); });
    return true;
}
void CookedWorld::Update(Vector3 eye, std::span<const Vector3> sources, WorldCollision &collision,
                         WorldRenderer *renderer, double radius, const StreamingSettings &settings)
{
    if (!Ready()) return;
    auto &s = *impl;
    const auto &index = s.reader.Index();
    if (!s.attached)
    {
        collision.ClearChunks();
        s.attached = true;
    }
    s.wantRender.clear();
    s.wantCollision.clear();
    s.wantObjects.clear();
    s.collideObjects.clear();
    const double nearDistance = std::max(distantMeters, g_ProjectSettings.RailroadDetailDistance());
    for (auto key : s.cellsIndex.Query(eye, radius + settings.preloadMeters))
    {
        const auto i = static_cast<std::size_t>(key.index);
        const double d = BoundsDistanceSquared(index.cells[i].bounds, eye);
        auto lod = d <= nearDistance * nearDistance ? PayloadKind::Near : PayloadKind::Distant;
        // LOD hysteresis stops repeated uploads around the 2 km boundary.
        if (const auto old = s.renders.find(i); old != s.renders.end() && old->second.lod == PayloadKind::Near &&
                                                d < (nearDistance + 128) * (nearDistance + 128))
            lod = PayloadKind::Near;
        s.wantRender[i] = {d, lod, d < 128 * 128};
    }
    for (auto key : s.objectsIndex.Query(eye, radius + settings.preloadMeters)) s.wantObjects.insert(key.index);
    for (auto position : sources)
    {
        for (auto key : s.cellsIndex.Query(position, settings.collisionMeters)) s.wantCollision.insert(key.index);
        for (auto key : s.objectCollisionIndex.Query(position, settings.collisionMeters))
        {
            s.wantObjects.insert(key.index);
            s.collideObjects.insert(key.index);
        }
    }
    // Publish at most one completed job each frame; the queue has a single bounded in-flight slot.
    std::optional<Impl::Result> done;
    {
        std::lock_guard lock(s.mutex);
        if (s.result)
        {
            done = std::move(s.result);
            s.result.reset();
            s.pending.reset();
        }
    }
    if (done)
    {
        const auto &r = done->request;
        if (!s.Wanted(r))
            ++s.stats.discarded;
        else if (!done->error.empty())
        {
            ++s.stats.failed;
            s.failed.insert(r.Key());
            s.error = done->error;
            if (r.kind == Impl::Kind::Model) s.failedModels.insert(index.entities[r.index].model);
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "World streaming: %s", s.error.c_str());
        }
        else
        {
            ++s.stats.completed;
            if (done->cell)
            {
                if (const auto old = s.renders.find(r.index); old != s.renders.end()) s.Forget(old->second, renderer);
                const auto bytes = CellBytes(*done->cell);
                s.cellSizes[{r.index, r.kind == Impl::Kind::Near ? PayloadKind::Near : PayloadKind::Distant}] = bytes;
                s.renders.insert_or_assign(
                    r.index, Impl::Render{r.kind == Impl::Kind::Near ? PayloadKind::Near : PayloadKind::Distant,
                                          std::move(done->cell), bytes});
            }
            else if (done->mesh)
            {
                const auto &path = index.entities[r.index].model;
                s.modelSizes[path] = MeshBytes(done->mesh);
                s.models[path] = std::move(done->mesh);
            }
            else if (done->collision)
            {
                auto &group = r.kind == Impl::Kind::Collision ? s.colliders : s.instanceColliders;
                group[r.index] = done->collision;
                collision.Attach(r.index | (r.kind == Impl::Kind::Collision ? 0 : modelCollisionBit),
                                 std::move(done->collision));
            }
        }
    }
    // Unload outside the hysteresis range. No graph recursion: neighboring roads do not pin a continent.
    const double keep = radius + settings.unloadMarginMeters;
    for (auto it = s.renders.begin(); it != s.renders.end();)
        if (BoundsDistanceSquared(index.cells[it->first].bounds, eye) > keep * keep)
        {
            s.Forget(it->second, renderer);
            it = s.renders.erase(it);
            ++s.stats.evicted;
        }
        else
            ++it;
    const double collisionKeep = settings.collisionMeters + 128;
    auto pruneCollision = [&](auto &group, bool model) {
        for (auto it = group.begin(); it != group.end();)
        {
            const auto &bounds = model ? index.entities[it->first].bounds : index.cells[it->first].bounds;
            if (Nearest(bounds, sources) > collisionKeep * collisionKeep)
            {
                collision.Detach(it->first | (model ? modelCollisionBit : 0));
                it = group.erase(it);
                ++s.stats.evicted;
            }
            else
                ++it;
        }
    };
    pruneCollision(s.colliders, false);
    pruneCollision(s.instanceColliders, true);
    for (auto it = s.instances.begin(); it != s.instances.end();)
    {
        const auto id = it++->first;
        if (!s.collideObjects.contains(id) &&
            BoundsDistanceSquared(s.objectsIndex.Bounds({SceneKind::Mesh, static_cast<int>(id)}), eye) > keep * keep)
            s.RemoveInstance(id, renderer);
    }
    // Instances share exactly one decoded model per path while it is resident.
    for (auto i : s.wantObjects)
    {
        if (s.instances.contains(i)) continue;
        const auto &e = index.entities[i];
        const auto model = s.models.find(e.model);
        if (model == s.models.end()) continue;
        Impl::Instance instance;
        instance.mesh = {model->second, e.transform};
        if (e.type == ObjectType::Marker && e.id.kind == SceneKind::Object)
        {
            auto marker = std::make_unique<Marker>();
            if (!marker->SetTransform(e.pose) || !marker->SetText(e.markerText))
            {
                s.error = SDL_GetError();
                continue;
            }
            marker->SetMesh(model->second);
            marker->hasCollision = e.hasCollision;
            marker->textColor = e.markerColor;
            marker->material = e.material;
            instance.marker = std::move(marker);
        }
        s.instances.emplace(i, std::move(instance));
    }
    s.Measure();
    // Farthest first under pressure. Nearby geometry and physics dependencies are soft-budget pins.
    while (s.stats.geometryBytes > settings.geometryBytes)
    {
        auto victim = s.renders.end();
        double farthest = -1;
        for (auto it = s.renders.begin(); it != s.renders.end(); ++it)
        {
            const double d = BoundsDistanceSquared(index.cells[it->first].bounds, eye);
            if (d > 128 * 128 && d > farthest)
            {
                victim = it;
                farthest = d;
            }
        }
        if (victim == s.renders.end()) break;
        s.stats.geometryBytes -= victim->second.bytes;
        s.Forget(victim->second, renderer);
        s.renders.erase(victim);
        ++s.stats.evicted;
    }
    while (s.stats.collisionBytes > settings.collisionBytes)
    {
        bool removed = false;
        for (auto *group : {&s.colliders, &s.instanceColliders})
        {
            const bool model = group == &s.instanceColliders;
            for (auto it = group->begin(); it != group->end();)
                if (!(model ? s.collideObjects : s.wantCollision).contains(it->first))
                {
                    s.stats.collisionBytes -= it->second->MemoryBytes();
                    collision.Detach(it->first | (model ? modelCollisionBit : 0));
                    it = group->erase(it);
                    removed = true;
                    ++s.stats.evicted;
                }
                else
                    ++it;
        }
        if (!removed) break;
    }
    for (auto it = s.models.begin(); it != s.models.end();)
    {
        const auto &path = it->first;
        bool needed = false;
        for (const auto &[id, instance] : s.instances)
            if (index.entities[id].model == path)
            {
                needed = true;
                break;
            }
        if (!needed)
        {
            it = s.models.erase(it);
            ++s.stats.evicted;
        }
        else
            ++it;
    }
    s.Measure();
    while (s.stats.modelBytes > settings.assetBytes / 2)
    {
        auto victim = s.models.end();
        double farthest = -1;
        for (auto it = s.models.begin(); it != s.models.end(); ++it)
        {
            double distance = std::numeric_limits<double>::infinity();
            bool pinned = false;
            for (const auto &[i, instance] : s.instances)
                if (index.entities[i].model == it->first)
                {
                    distance = std::min(distance, BoundsDistanceSquared(index.entities[i].bounds, eye));
                    pinned |= s.collideObjects.contains(i);
                }
            if (!pinned && distance > 128 * 128 && distance > farthest)
            {
                farthest = distance;
                victim = it;
            }
        }
        if (victim == s.models.end()) break;
        for (auto it = s.instances.begin(); it != s.instances.end();)
        {
            auto i = it++->first;
            if (index.entities[i].model == victim->first) s.RemoveInstance(i, renderer);
        }
        s.stats.modelBytes -= MeshBytes(victim->second);
        s.models.erase(victim);
        ++s.stats.evicted;
    }
    // Select only one next job. Old-camera work is discarded before publication, not queued indefinitely.
    std::optional<Impl::Request> next;
    double best = std::numeric_limits<double>::infinity();
    auto consider = [&](Impl::Kind kind, std::size_t i, double priority, std::shared_ptr<const Mesh> mesh = {}) {
        Impl::Request r{kind, i, std::move(mesh)};
        if (!s.failed.contains(r.Key()) && priority < best)
        {
            next = std::move(r);
            best = priority;
        }
    };
    if (!s.pending)
    {
        // Sorted prefix costs avoid scanning the entire resident set for every
        // candidate when the visibility radius reaches many thousands of cells.
        using Costs = std::vector<std::pair<double, std::size_t>>;
        auto prefix = [](Costs costs) {
            std::sort(costs.begin(), costs.end());
            std::size_t total = 0;
            for (auto &cost : costs)
            {
                total += cost.second;
                cost.second = total;
            }
            return costs;
        };
        auto atMost = [](const Costs &costs, double distance) {
            const auto end = std::upper_bound(costs.begin(), costs.end(), distance,
                                              [](double d, const auto &cost) { return d < cost.first; });
            return end == costs.begin() ? std::size_t(0) : std::prev(end)->second;
        };
        Costs geometryCosts, modelCosts;
        for (const auto &[i, resident] : s.renders)
            geometryCosts.emplace_back(BoundsDistanceSquared(index.cells[i].bounds, eye), resident.bytes);
        std::map<std::string, double> modelDistances;
        for (const auto &[i, instance] : s.instances)
        {
            const auto &path = index.entities[i].model;
            const double d = s.collideObjects.contains(i) ? -1 : BoundsDistanceSquared(index.entities[i].bounds, eye);
            auto [it, inserted] = modelDistances.emplace(path, d);
            if (!inserted) it->second = std::min(it->second, d);
        }
        for (const auto &[path, d] : modelDistances) modelCosts.emplace_back(d, MeshBytes(s.models.at(path)));
        geometryCosts = prefix(std::move(geometryCosts));
        modelCosts = prefix(std::move(modelCosts));
        for (auto i : s.wantCollision)
            if (!s.colliders.contains(i)) consider(Impl::Kind::Collision, i, Nearest(index.cells[i].bounds, sources));
        for (auto i : s.collideObjects)
        {
            const auto &path = index.entities[i].model;
            const auto model = s.models.find(path);
            if (s.failedModels.contains(path)) continue;
            const double d = Nearest(index.entities[i].bounds, sources);
            if (model == s.models.end())
                consider(Impl::Kind::Model, i, d);
            else if (!s.instanceColliders.contains(i))
                consider(Impl::Kind::InstanceCollision, i, d, model->second);
        }
        for (const auto &[i, want] : s.wantRender)
        {
            const auto old = s.renders.find(i);
            if (old != s.renders.end() && old->second.lod == want.lod) continue;
            const auto known = s.cellSizes.find({i, want.lod});
            const auto estimate = known == s.cellSizes.end()
                                      ? index.cells[i].payloads[static_cast<std::size_t>(want.lod)].bytes * 2
                                      : known->second;
            // Do not thrash far cells after the working set fills its budget.
            const std::size_t closer =
                atMost(geometryCosts, want.distance) - (old == s.renders.end() ? 0 : old->second.bytes);
            if (!want.protect && (estimate > settings.geometryBytes || closer > settings.geometryBytes - estimate))
                continue;
            consider(want.lod == PayloadKind::Near ? Impl::Kind::Near : Impl::Kind::Far, i, 1e12 + want.distance);
        }
        for (auto i : s.wantObjects)
        {
            const auto &path = index.entities[i].model;
            if (s.models.contains(path) || s.failedModels.contains(path)) continue;
            const double d = BoundsDistanceSquared(index.entities[i].bounds, eye);
            if (d > 128 * 128 && !s.collideObjects.contains(i))
            {
                const auto known = s.modelSizes.find(path);
                const std::size_t estimate = known == s.modelSizes.end() ? 1 : known->second;
                const auto closer = atMost(modelCosts, d);
                const auto budget = settings.assetBytes / 2;
                if (estimate > budget || closer > budget - estimate) continue;
            }
            consider(Impl::Kind::Model, i, 1e12 + d);
        }
        if (next)
        {
            s.pending = *next;
            {
                std::lock_guard lock(s.mutex);
                s.job = std::move(next);
            }
            s.condition.notify_one();
        }
    }
    s.Measure();
    s.stats.pressure = s.stats.geometryBytes > settings.geometryBytes || s.stats.modelBytes > settings.assetBytes / 2 ||
                       s.stats.collisionBytes > settings.collisionBytes;
}

bool CookedWorld::CollisionReady(Vector3 position, double radius) const
{
    if (!Ready()) return true;
    auto &s = *impl;
    for (auto key : s.cellsIndex.Query(position, radius))
        if (!s.colliders.contains(key.index)) return false;
    for (auto key : s.objectCollisionIndex.Query(position, radius))
        if (!s.instanceColliders.contains(key.index)) return false;
    return true;
}
bool CookedWorld::MayHaveGroundBelow(Vector3 position) const
{
    if (!Ready()) return false;
    const auto below = [&](const Bounds3& bounds)
    {
        // Include the pawn's footprint, not just its center. A false positive
        // merely postpones recovery until it falls below these bounds.
        return !bounds.Empty() && bounds.min.y <= position.y &&
               position.x >= bounds.min.x - 3 && position.x <= bounds.max.x + 3 &&
               position.z >= bounds.min.z - 3 && position.z <= bounds.max.z + 3;
    };
    // Nearby collision is protected by CollisionReady, but a high airborne
    // pawn can be far above its landing cell. Missing residency is not a hole.
    for (const auto& cell : impl->reader.Index().cells) if (below(cell.bounds)) return true;
    for (const auto& entity : impl->reader.Index().entities)
        if (entity.hasCollision && !entity.model.empty() && below(entity.bounds)) return true;
    return false;
}
void CookedWorld::Submit(WorldRenderer &renderer, const CameraView &camera, double radius)
{
    if (!Ready()) return;
    auto &s = *impl;
    const auto eye = camera.GetPosition();
    const double railDistance = g_ProjectSettings.RailroadDetailDistance();
    for (const auto &[i, render] : s.renders)
    {
        const auto &bounds = s.reader.Index().cells[i].bounds;
        if (BoundsDistanceSquared(bounds, eye) > radius * radius || !renderer.BoundsNeeded(bounds)) continue;
        for (const auto &batch : render.cell->batches)
        {
            const auto &mesh = batch.nearMesh;
            if (batch.layer == Layer::Rail && BoundsDistanceSquared(WorldBounds(mesh.mesh->bounds, mesh.transform),
                                                                    eye) > railDistance * railDistance)
                continue;
            renderer.SubmitWorldMesh(mesh, batch.material);
        }
        for (const auto &ties : render.cell->ties) renderer.SubmitRailroadTies(ties.run, 1);
    }
    for (const auto &[i, instance] : s.instances)
        renderer.SubmitWorldMesh(instance.mesh, s.reader.Index().entities[i].material);
}
void CookedWorld::SubmitLabels(WorldRenderer &renderer, const CameraView &camera)
{
    for (const auto &[i, instance] : impl->instances)
        if (instance.marker) instance.marker->SubmitLabel(renderer, camera);
}
bool CookedWorld::DrawStatistics(SDL_Renderer *renderer, float y) const
{
    SDL_Rect viewport{};
    if (!SDL_GetRenderViewport(renderer, &viewport)) return false;
    const auto &s = Stats();
    if (y < 0) y = std::max(48.f, float(viewport.h) - 185.f);
    const SDL_FRect box{8, y, std::min(650.f, float(viewport.w) - 16), 118};
    constexpr double mb = StreamingSettings::MiB;
    return SDL_SetRenderDrawColor(renderer, 24, 26, 30, 255) && SDL_RenderFillRect(renderer, &box) &&
           SDL_SetRenderDrawColor(renderer, 230, 232, 235, 255) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 8, "Streaming (F3) | %zu/%zu cells | %zu near / %zu distant",
                                     s.nearCells + s.distantCells, s.totalCells, s.nearCells, s.distantCells) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 24,
                                     "Geometry %.1f MiB | Models %.1f MiB (%zu shared / %zu instances)",
                                     s.geometryBytes / mb, s.modelBytes / mb, s.modelAssets, s.modelInstances) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 40, "Collision %.1f MiB (%zu chunks) | In flight %zu",
                                     s.collisionBytes / mb, s.collisionChunks, s.queued) &&
           SDL_RenderDebugTextFormat(renderer, 16, y + 56, "Loaded %zu | Evicted %zu | Stale %zu | Errors %zu",
                                     s.completed, s.evicted, s.discarded, s.failed) &&
           SDL_RenderDebugText(renderer, 16, y + 72,
                               s.pressure ? "Budget pressure: active dependencies protected"
                                          : "Asynchronous cells and shared assets; collision pinned near pawns") &&
           SDL_RenderDebugText(renderer, 16, y + 88,
                               Error().empty()
                                   ? "Near / distant / collision payloads load independently"
                                   : "Streaming error: recook the world; physics waits for missing collision");
}
