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
    Writer EncodeCell(const Cell& cell)
    {
        Writer w; w.Count(cell.batches.size());
        for (const auto& b : cell.batches) { w.Id(b.owner); w.U32(unsigned(b.layer)); w.String(b.material); w.MeshData(b.nearMesh); w.MeshData(b.distantMesh); }
        w.Count(cell.ties.size());
        for (const auto& ties : cell.ties) { w.Id(ties.owner); w.Transform(ties.run.first); w.Transform(ties.run.last); w.U32(ties.run.count); }
        return w;
    }
    Writer EncodeManifest(const Manifest& m)
    {
        Writer w; w.U64(m.sourceHash); w.F64(cellMeters); w.F64(distantMeters);
        w.Count(m.entities.size());
        for (const auto& e : m.entities)
        {
            w.Id(e.id); w.U32(e.folder); w.String(e.name); w.String(e.model); w.String(e.material); w.Transform(e.transform); w.Bounds(e.bounds);
            w.Count(e.dependencies.size()); for (auto dep : e.dependencies) w.Id(dep);
        }
        w.Count(m.assets.size());
        for (const auto& a : m.assets) { w.U32(unsigned(a.kind)); w.String(a.path); w.U64(a.hash); for (const auto& s : a.textures) w.String(s); }
        w.Count(m.cells.size());
        for (const auto& c : m.cells)
        {
            w.U32(std::bit_cast<std::uint32_t>(c.key.x)); w.U32(std::bit_cast<std::uint32_t>(c.key.z)); w.Bounds(c.bounds);
            w.U64(c.offset); w.U64(c.bytes); w.U64(c.hash); w.U64(c.nearTriangles); w.U64(c.distantTriangles);
            w.Count(c.owners.size()); for (auto id : c.owners) w.Id(id);
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
    return FileHash(kind == AssetKind::Material ? Material::ResolveFile(file) : file);
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
            CellRecord record; record.key = cell.key; record.offset = offset;
            auto encoded = EncodeCell(cell);
            if (encoded.data.size() > maxCellBytes) throw std::runtime_error("Cell exceeds 512 MiB; reduce authored density");
            record.bytes = encoded.data.size(); record.hash = Hash(encoded.data); offset += record.bytes;
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
            if (!data.write(encoded.data.data(), encoded.data.size())) throw std::runtime_error("Cannot write chunk payload");
        }
        data.close(); if (!data) throw std::runtime_error("Cannot finish chunk payload");
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
        h.data = h.data.subspan(8); if (h.U32() != version) throw std::runtime_error("Unsupported cooked world version");
        const auto bytes = h.U64(), checksum = h.U64();
        if (size < 28 || bytes > static_cast<std::uint64_t>(size) - 28) throw std::runtime_error("Invalid manifest size");
        auto data = Read(in, bytes, maxManifestBytes); if (Hash(data) != checksum) throw std::runtime_error("Manifest checksum mismatch");
        Input r{data}; Manifest m; m.sourceHash = r.U64();
        if (r.F64() != cellMeters || r.F64() != distantMeters) throw std::runtime_error("Unsupported world cook recipe");
        std::set<EntityId> ids;
        for (auto n = r.Count(176); n; --n)
        {
            Entity e; e.id = r.Id(); e.folder = static_cast<int>(r.U32()); e.name = r.String(); e.model = r.String(); e.material = r.String(); e.transform = r.Transform(); e.bounds = r.Bounds();
            for (auto d = r.Count(12); d; --d) e.dependencies.push_back(r.Id());
            if (!ids.insert(e.id).second) throw std::runtime_error("Duplicate entity ID");
            m.entities.push_back(std::move(e));
        }
        for (const auto& e : m.entities) for (auto d : e.dependencies) if (!ids.contains(d)) throw std::runtime_error("Missing entity dependency");
        std::set<std::pair<AssetKind, std::string>> assetKeys;
        for (auto n = r.Count(52); n; --n)
        {
            Asset a; const auto k = r.U32(); if (k > 1) throw std::runtime_error("Invalid asset kind"); a.kind = static_cast<AssetKind>(k); a.path = r.String(); a.hash = r.U64();
            // Asset paths must stay inside their roots.
            if (a.path.empty() || a.path.find("..") != a.path.npos || a.path.find(':') != a.path.npos || a.path.front() == '/' || a.path.find('\\') != a.path.npos || !assetKeys.emplace(a.kind, a.path).second) throw std::runtime_error("Invalid dependency path");
            for (auto& s : a.textures) { s = r.String(); if (!s.empty() && !Material::ValidTexturePath(s)) throw std::runtime_error("Invalid texture dependency"); }
            m.assets.push_back(std::move(a));
        }
        std::set<CellKey> keys; std::uint64_t end = 0;
        for (auto n = r.Count(100); n; --n)
        {
            CellRecord c; c.key.x = std::bit_cast<std::int32_t>(r.U32()); c.key.z = std::bit_cast<std::int32_t>(r.U32()); c.bounds = r.Bounds();
            c.offset = r.U64(); c.bytes = r.U64(); c.hash = r.U64(); c.nearTriangles = r.U64(); c.distantTriangles = r.U64();
            for (auto d = r.Count(12); d; --d) { auto id = r.Id(); if (!ids.contains(id)) throw std::runtime_error("Unknown cell owner"); c.owners.push_back(id); }
            if (!keys.insert(c.key).second || c.offset != end || c.bytes > maxCellBytes || end > static_cast<std::uint64_t>(size) - 28 - bytes || c.bytes > static_cast<std::uint64_t>(size) - 28 - bytes - end) throw std::runtime_error("Invalid cell directory");
            end += c.bytes; m.cells.push_back(std::move(c));
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
bool Reader::ReadCell(std::size_t index, Cell& cell) const
{
    try
    {
        if (path.empty() || index >= manifest.cells.size()) throw std::runtime_error("Invalid cell request");
        const auto& c = manifest.cells[index];
        std::ifstream in(Utf8(path), std::ios::binary); in.seekg(static_cast<std::streamoff>(payloadStart + c.offset));
        auto data = Read(in, c.bytes, maxCellBytes); if (Hash(data) != c.hash) throw std::runtime_error("Cell checksum mismatch");
        Input r{data}; Cell result; result.key = c.key;
        auto owner = [&] { auto id = r.Id(); if (std::find(c.owners.begin(), c.owners.end(), id) == c.owners.end()) throw std::runtime_error("Invalid batch owner"); return id; };
        for (auto n = r.Count(36); n; --n)
        {
            Batch b; b.owner = owner(); auto layer = r.U32(); if (layer > unsigned(Layer::Transition)) throw std::runtime_error("Invalid mesh layer");
            b.layer = static_cast<Layer>(layer); b.material = r.String(); if (!b.material.empty() && !Material::ValidAssetPath(b.material)) throw std::runtime_error("Invalid material path");
            b.nearMesh = r.MeshData(c.key.Origin()); b.distantMesh = r.MeshData(c.key.Origin()); result.batches.push_back(std::move(b));
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
}
