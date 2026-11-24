#include "world_package.hpp"
#include "spatial_index.hpp"
#include <SDL3/SDL.h>
#include <bit>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>

namespace WorldPackage
{
namespace
{
    std::filesystem::path Utf8(const std::string& text) { return std::filesystem::path(std::u8string(text.begin(), text.end())); }
    constexpr std::uint64_t maxManifestBytes = 128 * 1024 * 1024;
    constexpr std::uint64_t maxCellBytes = 512 * 1024 * 1024;
    std::uint64_t Hash(std::span<const char> data)
    {
        std::uint64_t hash = 14695981039346656037ull;
        for (unsigned char c : data) { hash ^= c; hash *= 1099511628211ull; }
        return hash;
    }
    struct Writer
    {
        std::vector<char> data;
        void U64(std::uint64_t v) { for (int i = 0; i < 8; ++i) { data.push_back(static_cast<char>(v & 255)); v >>= 8; } }
        void U32(std::uint32_t v) { for (int i = 0; i < 4; ++i) { data.push_back(static_cast<char>(v & 255)); v >>= 8; } }
        void F64(double v) { U64(std::bit_cast<std::uint64_t>(v)); }
        void F32(float v) { U32(std::bit_cast<std::uint32_t>(v)); }
        void Count(std::size_t v) { if (v > UINT32_MAX) throw std::runtime_error("Too many world records"); U32(static_cast<std::uint32_t>(v)); }
        void String(const std::string& s) { Count(s.size()); data.insert(data.end(), s.begin(), s.end()); }
        void Vec(Vector3 v) { F64(v.x); F64(v.y); F64(v.z); }
        void Bounds(Bounds3 b) { Vec(b.min); Vec(b.max); }
        void Id(EntityId id) { U32(static_cast<std::uint32_t>(id.kind)); U64(id.value); }
        void Transform(const MeshTransform& t) { Vec(t.origin); Vec(t.x); Vec(t.y); Vec(t.z); }
        void MeshData(const MeshInstance& m)
        {
            const Mesh empty;
            const Mesh& mesh = m.mesh ? *m.mesh : empty;
            Count(mesh.vertices.size()); Count(mesh.indices.size());
            for (const auto& v : mesh.vertices)
            {
                Vec(v.position);
                F32(float(v.normal.x)); F32(float(v.normal.y)); F32(float(v.normal.z));
                F32(v.uv.x); F32(v.uv.y);
                F32(v.color.r); F32(v.color.g); F32(v.color.b); F32(v.color.a);
            }
            for (auto i : mesh.indices) U32(i);
        }
    };
    struct Input
    {
        std::span<const char> data;
        void Need(std::size_t n) const { if (n > data.size()) throw std::runtime_error("Truncated cooked world record"); }
        std::uint64_t U64() { Need(8); std::uint64_t v = 0; for (int i = 0; i < 8; ++i) v |= std::uint64_t(static_cast<unsigned char>(data[i])) << (8 * i); data = data.subspan(8); return v; }
        std::uint32_t U32() { Need(4); std::uint32_t v = 0; for (int i = 0; i < 4; ++i) v |= std::uint32_t(static_cast<unsigned char>(data[i])) << (8 * i); data = data.subspan(4); return v; }
        double F64() { auto v = std::bit_cast<double>(U64()); if (!std::isfinite(v)) throw std::runtime_error("Non-finite world coordinate"); return v; }
        float F32() { auto v = std::bit_cast<float>(U32()); if (!std::isfinite(v)) throw std::runtime_error("Non-finite mesh attribute"); return v; }
        std::uint32_t Count(std::size_t minimumBytes = 1) { auto n = U32(); if (n > data.size() / minimumBytes) throw std::runtime_error("Invalid cooked record count"); return n; }
        std::string String() { auto n = Count(); if (n > 65536) throw std::runtime_error("Cooked string too long"); std::string s(data.data(), n); data = data.subspan(n); if (s.find('\0') != s.npos) throw std::runtime_error("Invalid cooked string"); return s; }
        Vector3 Vec() { return {F64(), F64(), F64()}; }
        Bounds3 Bounds() { Bounds3 b; b.min = Vec(); b.max = Vec(); if (b.Empty() || b.min.y > b.max.y || b.min.z > b.max.z) throw std::runtime_error("Invalid cooked bounds"); return b; }
        EntityId Id() { auto k = U32(); auto v = U64(); if (k < unsigned(SceneKind::Road) || k > unsigned(SceneKind::Intersection) || !v) throw std::runtime_error("Invalid entity ID"); return {static_cast<SceneKind>(k), v}; }
        MeshTransform Transform() { MeshTransform t; t.origin = Vec(); t.x = Vec(); t.y = Vec(); t.z = Vec(); if (!t.Valid()) throw std::runtime_error("Invalid cooked transform"); return t; }
        MeshInstance MeshData(Vector3 origin)
        {
            auto nv = U32(), ni = U32();
            if (ni % 3 || std::uint64_t(nv) * 60 + std::uint64_t(ni) * 4 > data.size()) throw std::runtime_error("Invalid cooked mesh size");
            auto m = std::make_shared<Mesh>(); m->vertices.reserve(nv); m->indices.reserve(ni);
            for (std::uint32_t i = 0; i < nv; ++i)
            {
                MeshVertex v; v.position = Vec(); v.normal = {F32(), F32(), F32()};
                v.uv = {F32(), F32()}; v.color = {F32(), F32(), F32(), F32()}; m->vertices.push_back(v);
            }
            for (std::uint32_t i = 0; i < ni; ++i) { auto index = U32(); if (index >= nv) throw std::runtime_error("Invalid mesh index"); m->indices.push_back(index); }
            m->UpdateBounds();
            return {std::move(m), MeshTransform{origin}};
        }
        void End() const { if (!data.empty()) throw std::runtime_error("Trailing cooked record data"); }
    };
    Writer EncodeCell(const Cell& cell, PayloadKind lod)
    {
        Writer w;
        std::vector<const Batch*> batches;
        for (const auto& b : cell.batches)
        {
            const auto& mesh = lod == PayloadKind::Near ? b.nearMesh : b.distantMesh;
            if (mesh.mesh && !mesh.mesh->indices.empty()) batches.push_back(&b);
        }
        w.Count(batches.size());
        for (const auto* b : batches)
        {
            w.Id(b->owner); w.U32(unsigned(b->layer)); w.String(b->material);
            w.MeshData(lod == PayloadKind::Near ? b->nearMesh : b->distantMesh);
        }
        w.Count(lod == PayloadKind::Near ? cell.ties.size() : 0);
        if (lod == PayloadKind::Near)
            for (const auto& ties : cell.ties) { w.Id(ties.owner); w.Transform(ties.run.first); w.Transform(ties.run.last); w.U32(ties.run.count); }
        return w;
    }
    Writer EncodeCollision(const Cell& cell)
    {
        std::vector<Vector3> positions;
        std::vector<std::uint32_t> indices;
        auto append = [&](const MeshInstance& instance) {
            if (!instance.mesh) return;
            if (positions.size() + instance.mesh->vertices.size() > UINT32_MAX) throw std::runtime_error("Collision chunk too large");
            const auto base = static_cast<std::uint32_t>(positions.size());
            for (const auto& v : instance.mesh->vertices) positions.push_back(instance.transform.Point(v.position) - cell.key.Origin());
            for (auto i : instance.mesh->indices) indices.push_back(base + i);
        };
        for (const auto& b : cell.batches)
            if (b.layer != Layer::Paint && b.layer != Layer::Water) append(b.nearMesh);
        for (const auto& ties : cell.ties)
            for (int i = 0; i < ties.run.count; ++i) append({RailroadTieMesh(), ties.run.At(i)});
        Writer w; w.Count(positions.size()); w.Count(indices.size());
        for (auto p : positions) w.Vec(p);
        for (auto i : indices) w.U32(i);
        return w;
    }
    Writer EncodeManifest(const Manifest& m)
    {
        Writer w; w.U64(m.sourceHash); w.F64(cellMeters); w.F64(distantMeters);
        w.Count(m.spawns.size());
        for (const auto& spawn : m.spawns) { w.Vec(spawn.position); w.F64(spawn.yaw); w.U32(spawn.authored); }
        w.Count(m.entities.size());
        for (const auto& e : m.entities)
        {
            w.Id(e.id); w.U32(e.folder); w.String(e.name); w.String(e.model); w.String(e.material); w.Transform(e.transform); w.Bounds(e.bounds);
            w.U32(static_cast<unsigned>(e.type));
            w.Vec(e.pose.position); w.Vec(e.pose.rotation); w.Vec(e.pose.scale);
            w.F32(e.health); w.F32(e.acceleration); w.U32(e.invincible);
            w.U32(e.hasCollision);
            w.String(e.markerText); for (auto color : e.markerColor) w.U32(color);
            w.Count(e.dependencies.size()); for (auto dep : e.dependencies) w.Id(dep);
        }
        w.Count(m.assets.size());
        for (const auto& a : m.assets) { w.U32(unsigned(a.kind)); w.String(a.path); w.U64(a.hash); for (const auto& s : a.textures) w.String(s); }
        w.Count(m.cells.size());
        for (const auto& c : m.cells)
        {
            w.U32(std::bit_cast<std::uint32_t>(c.key.x)); w.U32(std::bit_cast<std::uint32_t>(c.key.z)); w.Bounds(c.bounds);
            for (const auto& p : c.payloads) { w.U64(p.offset); w.U64(p.bytes); w.U64(p.hash); }
            w.U64(c.nearTriangles); w.U64(c.distantTriangles);
            w.Count(c.owners.size()); for (auto id : c.owners) w.Id(id);
        }
        w.Count(m.traffic.lanes.size());
        for (const auto& lane : m.traffic.lanes)
        {
            w.U64(lane.id); w.U32(lane.roadId); w.U32(lane.direction > 0 ? 1 : 0); w.U32(lane.ordinal);
            w.F64(lane.width); w.F64(lane.speedLimit);
            w.Count(lane.points.size()); for (auto point : lane.points) w.Vec(point);
            w.Count(lane.widths.size()); for (auto width : lane.widths) w.F64(width);
        }
        w.Count(m.traffic.movements.size());
        for (const auto& move : m.traffic.movements)
        {
            w.U64(move.id); w.U64(move.from); w.U64(move.to);
            w.U32(move.junction); w.U32(move.fromSide); w.U32(move.toSide);
            w.U32(static_cast<unsigned>(move.control)); w.U32(static_cast<unsigned>(move.turn));
            w.F64(move.path.speedLimit);
            w.Count(move.path.points.size()); for (auto p : move.path.points) w.Vec(p);
        }
        w.Count(m.traffic.links.size());
        for (const auto& link : m.traffic.links)
        {
            w.U64(link.from); w.U64(link.to);
            w.F64(link.fromDistance); w.F64(link.toDistance);
            w.U32(static_cast<unsigned>(link.kind)); w.F64(link.path.speedLimit);
            w.Count(link.path.points.size()); for (auto p : link.path.points) w.Vec(p);
        }
        return w;
    }
    std::vector<char> Read(std::ifstream& in, std::uint64_t bytes, std::uint64_t limit)
    {
        if (bytes > limit) throw std::runtime_error("Cooked record exceeds size limit");
        std::vector<char> data(static_cast<std::size_t>(bytes));
        if (!in.read(data.data(), static_cast<std::streamsize>(bytes))) throw std::runtime_error("Incomplete cooked world file");
        return data;
    }
}
std::uint64_t FileHash(const std::string& file)
{
    std::ifstream in(Utf8(file), std::ios::binary);
    if (!in) return 0;
    std::uint64_t hash = 14695981039346656037ull;
    std::array<char, 65536> data;
    while (in) { in.read(data.data(), data.size()); for (std::streamsize i = 0; i < in.gcount(); ++i) { hash ^= static_cast<unsigned char>(data[i]); hash *= 1099511628211ull; } }
    return in.eof() ? hash : 0;
}
std::uint64_t AssetHash(AssetKind kind, const std::string& path, AssetRoot root)
{
    const auto file = AssetPaths::Root(root) + (kind == AssetKind::Model ? "models/" : "textures/") + path;
    auto hash = FileHash(kind == AssetKind::Material ? Material::ResolveFile(file) : file);
    if (kind == AssetKind::Model)
    {
        for (const auto* extension : {".bmesh", ".bao", ".bcollision"})
        {
            const auto sidecar = FileHash(file + extension);
            if (sidecar) hash = (hash ^ sidecar) * 1099511628211ull;
        }
    }
    return hash;
}
std::string DefaultPath(const WorldDocument& world)
{
    const auto name = Utf8(world.Path()).filename().u8string();
    return AssetPaths::Root(AssetRoot::Cooked) + "worlds/" + std::string(name.begin(), name.end()) + ".bscworld";
}
bool Write(const std::string& file, Manifest manifest, std::span<const Cell> cells)
{
    std::string temporary;
    try
    {
        const auto parent = Utf8(file).parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        SDL_Time now; if (!SDL_GetCurrentTime(&now)) return false;
        temporary = file + "." + std::to_string(now) + ".tmp";
        // Payload spool avoids holding a second copy of all cooked geometry.
        const auto spool = temporary + ".cells";
        struct Cleanup { std::string file; ~Cleanup() { SDL_RemovePath(file.c_str()); } } cleanup{spool};
        std::ofstream data(Utf8(spool), std::ios::binary);
        std::uint64_t offset = 0;
        for (const auto& cell : cells)
        {
            CellRecord record; record.key = cell.key;
            for (auto kind : {PayloadKind::Near, PayloadKind::Distant, PayloadKind::Collision})
            {
                auto encoded = kind == PayloadKind::Collision ? EncodeCollision(cell) : EncodeCell(cell, kind);
                if (encoded.data.size() > maxCellBytes) throw std::runtime_error("Cell exceeds 512 MiB; reduce authored density");
                auto& p = record.payloads[static_cast<std::size_t>(kind)];
                p.offset = offset; p.bytes = encoded.data.size(); p.hash = Hash(encoded.data); offset += p.bytes;
                if (!data.write(encoded.data.data(), encoded.data.size())) throw std::runtime_error("Cannot write chunk payload");
            }
            std::set<EntityId> owners;
            for (const auto& b : cell.batches)
            {
                owners.insert(b.owner);
                const auto bounds = WorldBounds(b.nearMesh.mesh->bounds, b.nearMesh.transform);
                record.bounds.Include(bounds.min); record.bounds.Include(bounds.max);
                record.nearTriangles += b.nearMesh.mesh->indices.size() / 3;
                if (b.distantMesh.mesh) record.distantTriangles += b.distantMesh.mesh->indices.size() / 3;
            }
            for (const auto& ties : cell.ties)
            {
                owners.insert(ties.owner); const double r = ties.run.Radius();
                record.bounds.Include(ties.run.Center() - Vector3{r, r, r}); record.bounds.Include(ties.run.Center() + Vector3{r, r, r});
            }
            record.owners.assign(owners.begin(), owners.end()); manifest.cells.push_back(record);
        }
        data.close(); if (!data) throw std::runtime_error("Cannot finish chunk payload");
        if (!manifest.traffic.Validate()) throw std::runtime_error(SDL_GetError());
        auto index = EncodeManifest(manifest);
        if (index.data.size() > maxManifestBytes) throw std::runtime_error("World manifest exceeds limit");
        Writer header; header.data = {'B','S','C','C','H','N','K','\0'}; header.U32(version); header.U64(index.data.size()); header.U64(Hash(index.data));
        std::ofstream out(Utf8(temporary), std::ios::binary);
        out.write(header.data.data(), header.data.size()); out.write(index.data.data(), index.data.size());
        std::ifstream payload(Utf8(spool), std::ios::binary);
        std::array<char, 65536> block;
        while (payload) { payload.read(block.data(), block.size()); out.write(block.data(), payload.gcount()); }
        out.close();
        if (!payload.eof() || !out || !SDL_RenamePath(temporary.c_str(), file.c_str())) throw std::runtime_error("Cannot publish cooked world");
        return true;
    }
    catch (const std::exception& e) { if (!temporary.empty()) SDL_RemovePath(temporary.c_str()); return SDL_SetError("World cook: %s", e.what()); }
}
bool Reader::Open(const std::string& file)
{
    path.clear(); manifest = {};
    try
    {
        std::ifstream in(Utf8(file), std::ios::binary | std::ios::ate);
        if (!in) return SDL_SetError("Cooked world is unavailable");
        const auto size = in.tellg(); in.seekg(0);
        auto header = Read(in, 28, 28); Input h{header};
        if (std::string(header.data(), 8) != std::string("BSCCHNK\0", 8)) throw std::runtime_error("Invalid cooked world magic");
        h.data = h.data.subspan(8);
        const auto fileVersion = h.U32();
        if (fileVersion < 2 || fileVersion > version) throw std::runtime_error("Unsupported cooked world version");
        const auto bytes = h.U64(), checksum = h.U64();
        if (size < 28 || bytes > static_cast<std::uint64_t>(size) - 28) throw std::runtime_error("Invalid manifest size");
        auto data = Read(in, bytes, maxManifestBytes); if (Hash(data) != checksum) throw std::runtime_error("Manifest checksum mismatch");
        Input r{data}; Manifest m; m.sourceHash = r.U64();
        if (r.F64() != cellMeters || r.F64() != distantMeters) throw std::runtime_error("Unsupported world cook recipe");
        for (auto n = r.Count(fileVersion >= 3 ? 36 : 32); n; --n)
        {
            Spawn spawn{r.Vec(), r.F64()};
            if (fileVersion >= 3)
            {
                const auto authored = r.U32();
                if (authored > 1) throw std::runtime_error("Invalid spawn point flag");
                spawn.authored = authored != 0;
            }
            m.spawns.push_back(spawn);
        }
        if (std::ranges::any_of(m.spawns, [](const auto& s) { return s.authored; }) && m.spawns.size() != 1)
            throw std::runtime_error("An authored spawn point must be the only startup location");
        std::set<EntityId> ids;
        for (auto n = r.Count(280); n; --n)
        {
            Entity e; e.id = r.Id(); e.folder = static_cast<int>(r.U32()); e.name = r.String(); e.model = r.String(); e.material = r.String(); e.transform = r.Transform(); e.bounds = r.Bounds();
            const auto type = r.U32(); if (type > unsigned(ObjectType::Marker)) throw std::runtime_error("Invalid object type");
            e.type = static_cast<ObjectType>(type);
            e.pose.position = r.Vec(); e.pose.rotation = r.Vec(); e.pose.scale = r.Vec();
            if (!e.pose.ToMeshTransform().Valid()) throw std::runtime_error("Invalid object pose");
            e.health = r.F32(); e.acceleration = r.F32(); const auto invincible = r.U32();
            if (e.health < 0 || invincible > 1) throw std::runtime_error("Invalid object properties"); e.invincible = invincible != 0;
            if (fileVersion >= 4)
            {
                const auto hasCollision = r.U32();
                if (hasCollision > 1) throw std::runtime_error("Invalid object collision flag");
                e.hasCollision = hasCollision != 0;
            }
            e.markerText = r.String();
            for (auto& channel : e.markerColor) { auto c = r.U32(); if (c > 255) throw std::runtime_error("Invalid marker color"); channel = static_cast<Uint8>(c); }
            for (auto d = r.Count(12); d; --d) e.dependencies.push_back(r.Id());
            if (!ids.insert(e.id).second) throw std::runtime_error("Duplicate entity ID");
            m.entities.push_back(std::move(e));
        }
        for (const auto& e : m.entities) for (auto d : e.dependencies) if (!ids.contains(d)) throw std::runtime_error("Missing entity dependency");
        std::set<std::pair<AssetKind, std::string>> assetKeys;
        for (auto n = r.Count(fileVersion >= 9 ? 56 : 52); n; --n)
        {
            Asset a; const auto k = r.U32(); if (k > 1) throw std::runtime_error("Invalid asset kind"); a.kind = static_cast<AssetKind>(k); a.path = r.String(); a.hash = r.U64();
            // Asset paths must stay inside their roots.
            if (a.path.empty() || a.path.find("..") != a.path.npos || a.path.find(':') != a.path.npos || a.path.front() == '/' || a.path.find('\\') != a.path.npos || !assetKeys.emplace(a.kind, a.path).second) throw std::runtime_error("Invalid dependency path");
            const std::size_t slots = fileVersion >= 9 ? materialSlotCount : 9;
            for (std::size_t i = 0; i < slots; ++i)
            {
                auto& s = a.textures[i];
                s = r.String();
                if (!s.empty() && !Material::ValidTexturePath(s)) throw std::runtime_error("Invalid texture dependency");
            }
            m.assets.push_back(std::move(a));
        }
        std::set<CellKey> keys; std::uint64_t end = 0;
        for (auto n = r.Count(148); n; --n)
        {
            CellRecord c; c.key.x = std::bit_cast<std::int32_t>(r.U32()); c.key.z = std::bit_cast<std::int32_t>(r.U32()); c.bounds = r.Bounds();
            for (auto& p : c.payloads)
            {
                p.offset = r.U64(); p.bytes = r.U64(); p.hash = r.U64();
                if (p.offset != end || p.bytes > maxCellBytes || end > static_cast<std::uint64_t>(size) - 28 - bytes || p.bytes > static_cast<std::uint64_t>(size) - 28 - bytes - end)
                    throw std::runtime_error("Invalid cell directory");
                end += p.bytes;
            }
            c.nearTriangles = r.U64(); c.distantTriangles = r.U64();
            for (auto d = r.Count(12); d; --d) { auto id = r.Id(); if (!ids.contains(id)) throw std::runtime_error("Unknown cell owner"); c.owners.push_back(id); }
            if (!keys.insert(c.key).second) throw std::runtime_error("Duplicate cell address");
            m.cells.push_back(std::move(c));
        }
        if (fileVersion >= 5)
        {
            for (auto n = r.Count(88); n; --n)
            {
                TrafficLane lane;
                lane.id = r.U64();
                const auto road = r.U32(), forward = r.U32(), ordinal = r.U32();
                if (road > INT32_MAX || forward > 1 || ordinal >= RoadStyle::maxLanes ||
                    !ids.contains({SceneKind::Road, road})) throw std::runtime_error("Invalid traffic lane owner");
                lane.roadId = static_cast<int>(road); lane.direction = forward ? 1 : -1; lane.ordinal = static_cast<int>(ordinal);
                lane.width = r.F64(); lane.speedLimit = r.F64();
                for (auto count = r.Count(24); count; --count) lane.points.push_back(r.Vec());
                if (fileVersion >= 8)
                    for (auto count = r.Count(8); count; --count) lane.widths.push_back(r.F64());
                m.traffic.lanes.push_back(std::move(lane));
            }
            if (fileVersion >= 6)
                for (auto n = r.Count(104); n; --n)
                {
                    TrafficMovement move;
                    move.id = r.U64(); move.from = r.U64(); move.to = r.U64();
                    const auto junction = r.U32(), from = r.U32(), to = r.U32(), control = r.U32(), turn = r.U32();
                    if (junction > INT32_MAX || from >= 12 || to >= 12 || control > 2 || turn > 2 ||
                        !ids.contains({SceneKind::Intersection, junction})) throw std::runtime_error("Invalid traffic movement owner");
                    move.junction = static_cast<int>(junction); move.fromSide = static_cast<int>(from); move.toSide = static_cast<int>(to);
                    move.control = static_cast<TrafficControl>(control); move.turn = static_cast<TrafficTurn>(turn);
                    move.path.speedLimit = r.F64();
                    for (auto count = r.Count(24); count; --count) move.path.points.push_back(r.Vec());
                    m.traffic.movements.push_back(std::move(move));
                }
            if (fileVersion >= 7)
                for (auto n = r.Count(96); n; --n)
                {
                    TrafficLink link;
                    link.from = r.U64(); link.to = r.U64();
                    link.fromDistance = r.F64(); link.toDistance = r.F64();
                    const auto kind = r.U32();
                    if (kind > 2) throw std::runtime_error("Invalid highway connection kind");
                    link.kind = static_cast<TrafficLinkKind>(kind); link.path.speedLimit = r.F64();
                    for (auto count = r.Count(24); count; --count) link.path.points.push_back(r.Vec());
                    m.traffic.links.push_back(std::move(link));
                }
            if (!m.traffic.Validate()) throw std::runtime_error(SDL_GetError());
        }
        r.End(); if (28 + bytes + end != static_cast<std::uint64_t>(size)) throw std::runtime_error("Unexpected cooked world length");
        manifest = std::move(m); payloadStart = 28 + bytes; path = file; return true;
    }
    catch (const std::exception& e) { return SDL_SetError("Cooked world: %s", e.what()); }
}
bool Reader::Matches(const std::string& sourceWorld, AssetRoot root) const
{
    if (path.empty() || FileHash(sourceWorld) != manifest.sourceHash) return SDL_SetError("Cooked world is stale (world changed)");
    for (const auto& a : manifest.assets)
        if (AssetHash(a.kind, a.path, root) != a.hash) return SDL_SetError("Cooked world is stale (asset changed: %s)", a.path.c_str());
    return true;
}
bool Reader::ReadRender(std::size_t index, PayloadKind lod, Cell& cell) const
{
    try
    {
        if (path.empty() || index >= manifest.cells.size()) throw std::runtime_error("Invalid cell request");
        if (lod != PayloadKind::Near && lod != PayloadKind::Distant) throw std::runtime_error("Invalid render LOD");
        const auto& c = manifest.cells[index];
        const auto& p = c.payloads[static_cast<std::size_t>(lod)];
        std::ifstream in(Utf8(path), std::ios::binary); in.seekg(static_cast<std::streamoff>(payloadStart + p.offset));
        auto data = Read(in, p.bytes, maxCellBytes); if (Hash(data) != p.hash) throw std::runtime_error("Cell checksum mismatch");
        Input r{data}; Cell result; result.key = c.key;
        auto owner = [&] { auto id = r.Id(); if (std::find(c.owners.begin(), c.owners.end(), id) == c.owners.end()) throw std::runtime_error("Invalid batch owner"); return id; };
        for (auto n = r.Count(28); n; --n)
        {
            Batch b; b.owner = owner(); auto layer = r.U32(); if (layer > unsigned(Layer::Transition)) throw std::runtime_error("Invalid mesh layer");
            b.layer = static_cast<Layer>(layer); b.material = r.String(); if (!b.material.empty() && !Material::ValidAssetPath(b.material)) throw std::runtime_error("Invalid material path");
            b.nearMesh = r.MeshData(c.key.Origin()); result.batches.push_back(std::move(b));
        }
        for (auto n = r.Count(208); n; --n)
        {
            Ties ties; ties.owner = owner(); ties.run.first = r.Transform(); ties.run.last = r.Transform(); ties.run.count = static_cast<int>(r.U32());
            if (ties.run.count < 1 || ties.run.count > RailroadTieRun::capacity) throw std::runtime_error("Invalid tie run");
            result.ties.push_back(ties);
        }
        r.End(); cell = std::move(result); return true;
    }
    catch (const std::exception& e) { return SDL_SetError("Cooked cell: %s", e.what()); }
}
bool Reader::ReadCell(std::size_t index, Cell& cell) const
{
    Cell near, distant;
    if (!ReadRender(index, PayloadKind::Near, near) || !ReadRender(index, PayloadKind::Distant, distant)) return false;
    for (auto& b : near.batches)
    {
        const auto it = std::find_if(distant.batches.begin(), distant.batches.end(), [&](const auto& d) {
            return b.owner == d.owner && b.layer == d.layer && b.material == d.material;
        });
        b.distantMesh = it == distant.batches.end() ? MeshInstance{std::make_shared<Mesh>(), b.nearMesh.transform} : it->nearMesh;
    }
    cell = std::move(near); return true;
}
bool Reader::ReadCollision(std::size_t index, MeshInstance& mesh) const
{
    try
    {
        if (path.empty() || index >= manifest.cells.size()) throw std::runtime_error("Invalid collision cell request");
        const auto& c = manifest.cells[index]; const auto& p = c.payloads[2];
        std::ifstream in(Utf8(path), std::ios::binary); in.seekg(static_cast<std::streamoff>(payloadStart + p.offset));
        auto data = Read(in, p.bytes, maxCellBytes); if (Hash(data) != p.hash) throw std::runtime_error("Collision checksum mismatch");
        Input r{data}; const auto nv = r.U32(), ni = r.U32();
        if (ni % 3 || std::uint64_t(nv) * 24 + std::uint64_t(ni) * 4 != r.data.size()) throw std::runtime_error("Invalid collision size");
        auto result = std::make_shared<Mesh>(); result->vertices.reserve(nv); result->indices.reserve(ni);
        for (std::uint32_t i = 0; i < nv; ++i) result->vertices.push_back({r.Vec(), {}});
        for (std::uint32_t i = 0; i < ni; ++i) { auto v = r.U32(); if (v >= nv) throw std::runtime_error("Invalid collision index"); result->indices.push_back(v); }
        result->UpdateBounds(); mesh = {std::move(result), MeshTransform{c.key.Origin()}}; return true;
    }
    catch (const std::exception& e) { return SDL_SetError("Cooked collision: %s", e.what()); }
}

}
