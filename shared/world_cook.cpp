#include "world_package.hpp"
#include "world_lighting.hpp"
#include "spatial_index.hpp"
#include "marker.hpp"
#include "vehicle.hpp"
#include <meshoptimizer.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace WorldPackage
{
namespace
{
    CellKey Key(Vector3 p)
    {
        const double x = std::floor(p.x / cellMeters), z = std::floor(p.z / cellMeters);
        if (!std::isfinite(x) || !std::isfinite(z) || std::abs(x) > INT32_MAX - 1 || std::abs(z) > INT32_MAX - 1)
            throw std::runtime_error("Geometry lies outside the cooked cell coordinate range");
        return {static_cast<std::int32_t>(x), static_cast<std::int32_t>(z)};
    }
    MeshVertex Lerp(MeshVertex a, MeshVertex b, double t)
    {
        MeshVertex v; v.position = a.position + (b.position - a.position) * t;
        v.normal = a.normal + (b.normal - a.normal) * t;
        const float f = float(t);
        v.uv = {std::lerp(a.uv.x, b.uv.x, f), std::lerp(a.uv.y, b.uv.y, f)};
        v.color = {std::lerp(a.color.r, b.color.r, f), std::lerp(a.color.g, b.color.g, f), std::lerp(a.color.b, b.color.b, f), std::lerp(a.color.a, b.color.a, f)};
        return v;
    }
    std::vector<MeshVertex> Clip(std::vector<MeshVertex> polygon, bool x, double plane, bool positive)
    {
        std::vector<MeshVertex> out;
        if (polygon.empty()) return out;
        auto coord = [&](const MeshVertex& v) { return x ? v.position.x : v.position.z; };
        auto inside = [&](const MeshVertex& v) { return positive ? coord(v) >= plane : coord(v) <= plane; };
        auto a = polygon.back();
        for (auto b : polygon)
        {
            if (inside(a) != inside(b))
            {
                // Canonical interpolation order produces identical seam vertices on either side.
                auto lo = a, hi = b; if (coord(lo) > coord(hi)) std::swap(lo, hi);
                auto v = Lerp(lo, hi, (plane - coord(lo)) / (coord(hi) - coord(lo)));
                (x ? v.position.x : v.position.z) = plane;
                out.push_back(v);
            }
            if (inside(b)) out.push_back(b);
            a = b;
        }
        return out;
    }
    void Weld(Mesh& mesh)
    {
        std::map<std::array<double, 12>, std::uint32_t> unique;
        std::vector<MeshVertex> vertices;
        for (auto& index : mesh.indices)
        {
            const auto v = mesh.vertices.at(index);
            const std::array key{v.position.x, v.position.y, v.position.z, v.normal.x, v.normal.y, v.normal.z,
                                double(v.uv.x), double(v.uv.y), double(v.color.r), double(v.color.g), double(v.color.b), double(v.color.a)};
            auto [it, inserted] = unique.emplace(key, static_cast<std::uint32_t>(vertices.size()));
            if (inserted) vertices.push_back(v);
            index = it->second;
        }
        mesh.vertices = std::move(vertices); mesh.UpdateBounds();
    }
}
void ClipTriangle(std::map<CellKey, Cell>& cells, EntityId owner, Layer layer, const std::string& material,
                  const std::array<MeshVertex, 3>& triangle)
{
    Bounds3 bounds; for (const auto& v : triangle) bounds.Include(v.position);
    const auto lo = Key(bounds.min), hi = Key(bounds.max);
    if ((std::int64_t(hi.x) - lo.x + 1) * (std::int64_t(hi.z) - lo.z + 1) > 65536)
        throw std::runtime_error("A triangle spans too many world cells; subdivide the source mesh");
    for (auto x = lo.x; x <= hi.x; ++x) for (auto z = lo.z; z <= hi.z; ++z)
    {
        CellKey key{x, z}; const auto origin = key.Origin();
        std::vector<MeshVertex> polygon(triangle.begin(), triangle.end());
        polygon = Clip(std::move(polygon), true, origin.x, true);
        polygon = Clip(std::move(polygon), true, origin.x + cellMeters, false);
        polygon = Clip(std::move(polygon), false, origin.z, true);
        polygon = Clip(std::move(polygon), false, origin.z + cellMeters, false);
        for (std::size_t i = 2; i < polygon.size(); ++i)
        {
            const auto cross = Math::Cross(polygon[i - 1].position - polygon[0].position, polygon[i].position - polygon[0].position);
            if (Math::Dot(cross, cross) < 1e-20) continue;
            auto& cell = cells[key]; cell.key = key;
            auto it = std::find_if(cell.batches.begin(), cell.batches.end(), [&](const Batch& b) { return b.owner == owner && b.layer == layer && b.material == material; });
            if (it == cell.batches.end())
            {
                cell.batches.push_back({owner, layer, material, {std::make_shared<Mesh>(), MeshTransform{origin}}, {}});
                it = std::prev(cell.batches.end());
            }
            auto mesh = std::const_pointer_cast<Mesh>(it->nearMesh.mesh);
            for (auto j : {std::size_t(0), i - 1, i})
            {
                auto v = polygon[j]; v.position = v.position - origin;
                mesh->indices.push_back(static_cast<std::uint32_t>(mesh->vertices.size())); mesh->vertices.push_back(v);
            }
        }
    }
}
std::shared_ptr<Mesh> Simplify(const Mesh& mesh)
{
    auto result = std::make_shared<Mesh>(mesh);
    if (mesh.indices.size() < 12) return result;
    std::vector<std::array<float, 3>> positions;
    std::vector<std::array<float, 9>> attributes;
    std::vector<unsigned char> locked;
    positions.reserve(mesh.vertices.size()); attributes.reserve(mesh.vertices.size());
    for (const auto& v : mesh.vertices)
    {
        positions.push_back({float(v.position.x), float(v.position.y), float(v.position.z)});
        attributes.push_back({float(v.normal.x), float(v.normal.y), float(v.normal.z), v.uv.x, v.uv.y, v.color.r, v.color.g, v.color.b, v.color.a});
        // Explicit cell-plane locks supplement topological border locks (also protects holes/material seams).
        locked.push_back(v.position.x == 0 || v.position.x == cellMeters || v.position.z == 0 || v.position.z == cellMeters);
    }
    constexpr float weights[]{0.5f, 0.5f, 0.5f, 0.01f, 0.01f, 0.5f, 0.5f, 0.5f, 0.5f};
    const auto count = meshopt_simplifyWithAttributes(result->indices.data(), mesh.indices.data(), mesh.indices.size(),
        positions.front().data(), positions.size(), sizeof(positions.front()), attributes.front().data(), sizeof(attributes.front()),
        weights, std::size(weights), locked.data(), (mesh.indices.size() / 12) * 3, 0.20f,
        meshopt_SimplifyLockBorder | meshopt_SimplifyErrorAbsolute, nullptr);
    result->indices.resize(count);
    Weld(*result); // Compact unused vertices; original vertex positions remain unchanged.
    return result;
}
bool Cook(WorldDocument& world, const std::string& file)
{
    try
    {
        // Persist legacy IDs before fingerprinting. Never cook unsaved lake drafts or invalid references.
        if (std::filesystem::weakly_canonical(std::filesystem::path(std::u8string(file.begin(), file.end()))) ==
            std::filesystem::weakly_canonical(std::filesystem::path(std::u8string(world.Path().begin(), world.Path().end()))))
            return SDL_SetError("Cooked output must differ from the authored world file");
        if (!world.Save()) return false;
        world.UpdateIntersections();
        Manifest manifest; manifest.sourceHash = FileHash(world.Path());
        if (!manifest.sourceHash) return SDL_SetError("Cannot fingerprint the saved world");
        std::map<CellKey, Cell> cells;
        std::map<EntityId, std::size_t> entities;
        std::map<std::pair<AssetKind, std::string>, std::size_t> assets;
        const auto root = world.models.Location();
        auto asset = [&](AssetKind kind, const std::string& path) {
            if (path.empty() || assets.contains({kind, path})) return;
            Asset a; a.kind = kind; a.path = path; a.hash = AssetHash(kind, path, root);
            if (kind == AssetKind::Material)
            {
                Material m;
                if (m.Load(AssetPaths::Root(root) + "textures/" + path))
                    for (std::size_t i = 0; i < materialSlotCount; ++i) if (m.Uses(static_cast<MaterialSlot>(i))) a.textures[i] = m.textures[i];
            }
            assets.emplace(std::make_pair(kind, path), manifest.assets.size()); manifest.assets.push_back(std::move(a));
        };
        auto entity = [&](SceneKey key, Bounds3 bounds, std::string name, const MeshInstance* instance, std::string material) {
            Entity e; e.id = world.StableId(key); e.folder = world.hierarchy.Parent(key); e.bounds = bounds; e.name = std::move(name); e.material = material;
            if (e.bounds.Empty()) e.bounds.Include({});
            if (instance) { e.transform = instance->transform; if (instance->mesh) e.model = instance->mesh->assetPath; }
            asset(AssetKind::Model, e.model); asset(AssetKind::Material, material);
            if (material.empty() && instance && instance->mesh && instance->mesh->materials)
                for (const auto& slot : instance->mesh->materials->slots) asset(AssetKind::Material, slot.material);
            entities[e.id] = manifest.entities.size(); manifest.entities.push_back(std::move(e));
        };
        auto emit = [&](EntityId id, Layer layer, const std::string& material, std::array<MeshVertex, 3> t, bool flat = true) {
            if (flat) { const auto normal = Math::Normalize(Math::Cross(t[1].position - t[0].position, t[2].position - t[0].position)); for (auto& v : t) v.normal = normal; }
            ClipTriangle(cells, id, layer, material, t);
        };
        auto roadMesh = [&](EntityId id, const RoadGeometry& road, const std::string& material) {
            for (const auto& t : road.pavementTriangles)
            {
                std::array<MeshVertex, 3> v; for (int i = 0; i < 3; ++i) v[i] = {t[i].position, {}, t[i].uv, Srgb::ToLinear(SDL_FColor{.43f,.43f,.43f,1})}; emit(id, Layer::Surface, material, v);
            }
            auto colored = [&](const auto& triangles, Layer layer) {
                for (const auto& t : triangles) { std::array<MeshVertex, 3> v; for (int i = 0; i < 3; ++i) v[i] = {t[i].position, {}, {}, Srgb::ToLinear(t[i].color)}; emit(id, layer, {}, v); }
            };
            colored(road.edgeTriangles, Layer::Surface); colored(road.bridgeTriangles, Layer::Surface);
            for (const auto& [path, strip] : road.transitionStrips)
            {
                asset(AssetKind::Material, path);
                Material m; const bool valid = !path.empty() && m.Load(AssetPaths::Root(root) + "textures/" + path) && m.type == MaterialType::Transition;
                for (const auto& t : strip) { std::array<MeshVertex, 3> v; for (int i = 0; i < 3; ++i) v[i] = {t[i].position, {}, t[i].uv, t[i].color}; emit(id, Layer::Transition, valid ? path : "", v); }
            }
            auto paint = [&](const auto& triangles, SDL_FColor color) {
                for (const auto& t : triangles) { std::array<MeshVertex, 3> v; for (int i = 0; i < 3; ++i) v[i] = {t[i], {}, {}, Srgb::ToLinear(color)}; emit(id, Layer::Paint, {}, v); }
            };
            paint(road.whiteStripes, {.9f,.9f,.9f,1}); paint(road.yellowStripes, {.96f,.85f,.12f,1});
            for (const auto& segment : road.railroadSegments)
            {
                colored(segment.rails, Layer::Rail);
                for (const auto& run : segment.ties)
                {
                    // Ties are shared rigid instances, not cut triangles. Keep a run intact;
                    // its conservative bounds can extend into neighboring cells.
                    const auto key = Key(run.Center());
                    auto& cell = cells[key]; cell.key = key;
                    cell.ties.push_back({id, run});
                }
            }
        };
        for (const auto& road : world.roads)
        {
            if (road.style.railroad) continue;
            const auto samples = road.laneCenterlines.empty() ? road.SegmentSamples(0) : std::span<const Vector3>(road.laneCenterlines.front());
            if (samples.size() < 3) continue;
            const auto i = samples.size() / 2;
            const auto tangent = samples[i + 1] - samples[i - 1];
            manifest.spawns.push_back({samples[i], std::atan2(tangent.x, tangent.z)});
        }
        for (std::size_t i = 0; i < world.roads.size(); ++i)
        {
            const auto& road = world.roads[i]; const SceneKey key{SceneKind::Road, static_cast<int>(i)};
            const auto names = road.NameList();
            entity(key, road.bounds, names, nullptr, road.style.material); roadMesh(world.StableId(key), road, road.style.material);
        }
        for (const auto& [id, junction] : world.intersections)
        {
            auto it = world.IntersectionSurfaces().find(id); Bounds3 bounds;
            if (it != world.IntersectionSurfaces().end()) bounds = it->second.geometry.bounds;
            if (bounds.Empty()) bounds.Include(junction.position);
            entity({SceneKind::Intersection, id}, bounds, {}, nullptr, junction.material);
            if (it != world.IntersectionSurfaces().end()) roadMesh({SceneKind::Intersection, static_cast<std::uint64_t>(id)}, it->second.geometry, junction.material);
        }
        auto meshBounds = [](const MeshInstance& m) { return m.mesh ? WorldBounds(m.mesh->bounds, m.transform) : Bounds3{}; };
        for (std::size_t i = 0; i < world.meshes.size(); ++i) entity({SceneKind::Mesh, int(i)}, meshBounds(world.meshes[i]), {}, &world.meshes[i], {});
        for (std::size_t i = 0; i < world.Objects().size(); ++i)
        {
            const auto& object = *world.Objects()[i]; const auto* marker = dynamic_cast<const Marker*>(&object);
            entity({SceneKind::Object, int(i)}, meshBounds(object.Instance()), {}, &object.Instance(), marker ? marker->material : "");
            auto& e = manifest.entities.back(); e.type = object.Type(); e.pose = object.GetTransform();
            e.health = object.health; e.invincible = object.invincible;
            if (const auto* vehicle = dynamic_cast<const Vehicle*>(&object)) e.acceleration = vehicle->acceleration;
            if (marker) { e.markerText = marker->Text(); e.markerColor = marker->textColor; }
        }
        for (std::size_t i = 0; i < world.Terrain().size(); ++i)
        {
            const auto& chunk = *world.Terrain()[i]; const SceneKey key{SceneKind::Terrain, int(i)};
            entity(key, meshBounds(chunk.Instance()), chunk.Lake() ? "Lake" : "Terrain", &chunk.Instance(), chunk.material);
            auto bake = [&](const MeshInstance& instance, bool terrain, const std::string& material) {
                if (!instance.mesh) return;
                asset(AssetKind::Material, material);
                const auto& mesh = *instance.mesh; const auto normal = instance.transform.NormalTransform();
                for (std::size_t j = 0; j < mesh.indices.size(); j += 3)
                {
                    const auto& surface = material.empty() ? mesh.MaterialAt(j) : material;
                    std::array<MeshVertex, 3> v;
                    for (int k = 0; k < 3; ++k)
                    {
                        v[k] = mesh.vertices.at(mesh.indices[j + k]); v[k].position = instance.transform.Point(v[k].position);
                        v[k].normal = Math::Normalize(normal.Direction(v[k].normal));
                        if (terrain && chunk.boundary && material.empty())
                        { v[k].color.r *= TerrainChunk::groundColor.r; v[k].color.g *= TerrainChunk::groundColor.g; v[k].color.b *= TerrainChunk::groundColor.b; }
                    }
                    if (terrain && (!material.empty() || !mesh.HasMaterials()))
                    {
                        const auto local = instance.transform.Unrotate(Math::Cross(v[1].position - v[0].position, v[2].position - v[0].position));
                        auto u = instance.transform.x, w = instance.transform.z;
                        if (std::abs(local.x) > std::abs(local.y) && std::abs(local.x) >= std::abs(local.z)) u = instance.transform.y;
                        else if (std::abs(local.z) > std::abs(local.y)) w = instance.transform.y;
                        for (auto& vertex : v) { auto p = vertex.position - instance.transform.origin; vertex.uv = {float(Math::Dot(p, u) / 4), float(Math::Dot(p, w) / 4)}; }
                    }
                    emit(world.StableId(key), terrain ? Layer::Surface : Layer::Water, surface, v, false);
                }
            };
            bake(chunk.Instance(), true, chunk.material);
            if (chunk.Lake()) bake(chunk.Water(), false, LakeDefinition::waterMaterial);
        }
        for (const auto& e : manifest.entities)
            if ((e.id.kind == SceneKind::Terrain || e.id.kind == SceneKind::Mesh) && !e.bounds.Empty())
                manifest.spawns.push_back({(e.bounds.min + e.bounds.max) * 0.5, 0});
        if (manifest.spawns.empty()) manifest.spawns.push_back({});
        // Dependency edges are metadata, not recursive render/load requirements. Cycles are expected.
        auto link = [&](EntityId a, EntityId b) {
            if (a == b || !entities.contains(a) || !entities.contains(b)) return;
            manifest.entities[entities.at(a)].dependencies.push_back(b);
            manifest.entities[entities.at(b)].dependencies.push_back(a);
        };
        std::map<int, EntityId> stations, sockets;
        for (const auto& road : world.roads) for (const auto& p : road.points)
        { if (p.rampStation) stations[p.rampStation] = {SceneKind::Road, std::uint64_t(road.id)}; if (p.connectorSocket) sockets[p.connectorSocket] = {SceneKind::Road, std::uint64_t(road.id)}; }
        for (const auto& road : world.roads)
        {
            EntityId id{SceneKind::Road, std::uint64_t(road.id)};
            for (const auto& p : road.points)
            {
                if (p.anchor) link(id, {SceneKind::Intersection, std::uint64_t(p.anchor->intersectionId)});
                if (p.ramp && stations.contains(p.ramp->station)) link(id, stations.at(p.ramp->station));
                if (p.connectorTarget && sockets.contains(p.connectorTarget)) link(id, sockets.at(p.connectorTarget));
                if (p.bridge) for (int lower : p.bridge->lowerRoads) link(id, {SceneKind::Road, std::uint64_t(lower)});
            }
            for (const auto& crossing : road.crossings) link(id, {SceneKind::Road, std::uint64_t(crossing.railroadId)});
        }
        for (std::size_t i = 0; i < world.Terrain().size(); ++i)
            if (const auto& boundary = world.Terrain()[i]->boundary)
            {
                auto id = world.StableId({SceneKind::Terrain, int(i)});
                for (int road : boundary->roads) link(id, {SceneKind::Road, std::uint64_t(road)});
                for (int junction : boundary->intersections) link(id, {SceneKind::Intersection, std::uint64_t(junction)});
            }
        for (auto& e : manifest.entities) { auto& d = e.dependencies; std::sort(d.begin(), d.end()); d.erase(std::unique(d.begin(), d.end()), d.end()); }
        std::vector<Cell> packed; packed.reserve(cells.size());
        for (auto& [key, cell] : cells)
        {
            for (auto& b : cell.batches)
            {
                auto mesh = std::const_pointer_cast<Mesh>(b.nearMesh.mesh); Weld(*mesh);
                if (b.layer != Layer::Paint && b.layer != Layer::Rail) b.distantMesh = {Simplify(*mesh), b.nearMesh.transform};
            }
            packed.push_back(std::move(cell));
        }
        return Write(file, std::move(manifest), packed);
    }
    catch (const std::exception& e) { return SDL_SetError("World cook failed: %s", e.what()); }
}
}
