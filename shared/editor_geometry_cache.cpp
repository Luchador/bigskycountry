#include "editor_geometry_cache.hpp"
#include "editor_geometry_cache_build.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace
{
    constexpr std::uint64_t maxEntryBytes = 256ull * 1024 * 1024;
    constexpr std::uint32_t schema = 1;
    constexpr std::size_t headerBytes = 40;
    constexpr std::uint64_t magic = 0x314f4547435342ull; // BSCGEO1, little endian.

    // IEEE CRC-32. Precompute the byte table: the SDL implementation computes
    // eight polynomial steps per byte, which is costly for large terrain meshes.
    constexpr auto crcTable = []
    {
        std::array<std::uint32_t, 256> table{};
        for (std::uint32_t i = 0; i < table.size(); ++i)
        {
            auto value = i;
            for (int bit = 0; bit < 8; ++bit) value = (value >> 1) ^ ((value & 1) ? 0xedb88320u : 0u);
            table[i] = value;
        }
        return table;
    }();
    std::uint32_t Checksum(std::span<const char> data)
    {
        std::uint32_t crc = UINT32_MAX;
        for (unsigned char byte : data) crc = crcTable[(crc ^ byte) & 255] ^ (crc >> 8);
        return ~crc;
    }

    struct Writer
    {
        std::vector<char> data;
        void U32(std::uint32_t v) { for (int i = 0; i < 4; ++i) { data.push_back(static_cast<char>(v & 255)); v >>= 8; } }
        void U64(std::uint64_t v) { for (int i = 0; i < 8; ++i) { data.push_back(static_cast<char>(v & 255)); v >>= 8; } }
        void F64(double v) { U64(std::bit_cast<std::uint64_t>(v)); }
        void F32(float v) { U32(std::bit_cast<std::uint32_t>(v)); }
        void Vec(Vector3 v) { F64(v.x); F64(v.y); F64(v.z); }
        void Count(std::size_t n)
        {
            if (n > UINT32_MAX) throw std::runtime_error("Cache count too large");
            U32(static_cast<std::uint32_t>(n));
        }
        template<typename T, typename Write> void List(const T& values, Write write)
        { Count(values.size()); for (const auto& value : values) write(value); }
        void Points(const std::vector<Vector3>& points) { List(points, [&](auto p) { Vec(p); }); }
        void MeshData(const Mesh& mesh)
        {
            // Only generated terrain/water meshes belong in this cache. Material
            // assignments and placement transforms stay in the world document.
            if (!mesh.assetPath.empty() || mesh.materials || !mesh.parts.empty())
                throw std::runtime_error("Imported mesh is not editor cache geometry");
            Count(mesh.vertices.size()); Count(mesh.indices.size());
            for (const auto& v : mesh.vertices)
            {
                Vec(v.position); Vec(v.normal);
                F32(v.uv.x); F32(v.uv.y);
                F32(v.color.r); F32(v.color.g); F32(v.color.b); F32(v.color.a);
            }
            for (auto i : mesh.indices) U32(i);
        }
        void WaterData(const LakeGeometry& geometry)
        {
            MeshData(*geometry.bed); MeshData(*geometry.water);
            List(geometry.shoreline, [&](const auto& s)
            {
                Vec(s.position); U32(static_cast<std::uint32_t>(s.segment));
                F64(s.t); U32(static_cast<std::uint32_t>(s.bank + 1));
            });
        }
    };

    struct Reader
    {
        std::span<const char> data;
        void Need(std::uint64_t bytes) const
        { if (bytes > data.size()) throw std::runtime_error("Truncated editor geometry cache"); }
        std::uint32_t U32()
        {
            Need(4); std::uint32_t v = 0;
            for (int i = 0; i < 4; ++i) v |= std::uint32_t(static_cast<unsigned char>(data[i])) << (8 * i);
            data = data.subspan(4); return v;
        }
        std::uint64_t U64()
        {
            const auto lo = U32(), hi = U32(); return lo | (std::uint64_t(hi) << 32);
        }
        double F64()
        {
            const auto v = std::bit_cast<double>(U64());
            if (!std::isfinite(v)) throw std::runtime_error("Invalid cached coordinate");
            return v;
        }
        float F32()
        {
            const auto v = std::bit_cast<float>(U32());
            if (!std::isfinite(v)) throw std::runtime_error("Invalid cached attribute");
            return v;
        }
        Vector3 Vec() { return {F64(), F64(), F64()}; }
        std::shared_ptr<const Mesh> MeshData()
        {
            const auto nv = U32(), ni = U32();
            if (!nv || !ni || ni % 3) throw std::runtime_error("Invalid cached mesh counts");
            Need(std::uint64_t(nv) * 72 + std::uint64_t(ni) * 4);
            auto mesh = std::make_shared<Mesh>(); mesh->vertices.reserve(nv); mesh->indices.reserve(ni);
            for (std::uint32_t i = 0; i < nv; ++i)
            {
                MeshVertex v; v.position = Vec(); v.normal = Vec();
                v.uv = {F32(), F32()}; v.color = {F32(), F32(), F32(), F32()};
                mesh->vertices.push_back(v);
            }
            for (std::uint32_t i = 0; i < ni; ++i)
            {
                const auto index = U32();
                if (index >= nv) throw std::runtime_error("Invalid cached mesh index");
                mesh->indices.push_back(index);
            }
            mesh->UpdateBounds(); return mesh;
        }
        LakeGeometry WaterData()
        {
            LakeGeometry geometry; geometry.bed = MeshData(); geometry.water = MeshData();
            const auto n = U32(); Need(std::uint64_t(n) * 40);
            if (n < 4) throw std::runtime_error("Invalid cached shoreline");
            geometry.shoreline.reserve(n);
            for (std::uint32_t i = 0; i < n; ++i)
            {
                LakeSample s; s.position = Vec(); const auto segment = U32(); s.t = F64(); const auto bank = U32();
                if (segment > INT32_MAX || bank > 2 || s.t < 0 || s.t > 1)
                    throw std::runtime_error("Invalid cached shoreline sample");
                s.segment = static_cast<int>(segment); s.bank = static_cast<int>(bank) - 1;
                geometry.shoreline.push_back(s);
            }
            if (geometry.shoreline.front().position != geometry.shoreline.back().position)
                throw std::runtime_error("Open cached shoreline");
            return geometry;
        }
        void End() const { if (!data.empty()) throw std::runtime_error("Trailing cached data"); }
    };

    // Two independent 64-bit accumulators, over explicit scalar encodings (never
    // pointers, padding or transient revision numbers). Checksums detect damaged
    // payloads separately; this is a cache key, not a security primitive.
    std::array<std::uint64_t, 2> KeyFor(const Writer& input)
    {
        std::array<std::uint64_t, 2> key{14695981039346656037ull, 0x9e3779b97f4a7c15ull};
        const auto add = [&](unsigned char b)
        {
            key[0] = (key[0] ^ b) * 1099511628211ull;
            key[1] = std::rotl(key[1] ^ b, 7) * 0x9e3779b185ebca87ull;
        };
        for (unsigned char b : std::string_view(editorGeometrySignature)) add(b);
        for (unsigned char b : input.data) add(b);
        return key;
    }

    void Footprint(Writer& w, const RoadGeometry& g)
    {
        w.Points(g.outline);
        w.List(g.holes, [&](const auto& hole) { w.Points(hole); });
        const auto slab = [&](const RoadGeometry::CrossingSlab& s)
        {
            w.Points(s.outline);
            w.List(s.walls, [&](const auto& wall) { for (auto p : wall) w.Vec(p); });
        };
        w.List(g.groundFootprints, slab); w.List(g.crossingSlabs, slab);
    }
}

EditorGeometryCache::EditorGeometryCache(std::filesystem::path directory, std::uint64_t budgetBytes)
    : directory(std::move(directory)), budget(budgetBytes)
{
    Prune();
}

std::shared_ptr<EditorGeometryCache> EditorGeometryCache::CreateDefault()
{
    return std::make_shared<EditorGeometryCache>(std::filesystem::path(editorGeometryDirectory));
}

std::filesystem::path EditorGeometryCache::File(Key key) const
{
    std::ostringstream name;
    name << std::hex << std::setfill('0') << std::setw(16) << key[0] << std::setw(16) << key[1] << ".bgeom";
    return directory / name.str();
}

std::optional<std::vector<char>> EditorGeometryCache::Read(Key key)
{
    ++statistics.misses;
    if (directory.empty() || !budget) return {};
    const auto file = File(key);
    try
    {
        std::ifstream in(file, std::ios::binary | std::ios::ate);
        if (!in) return {};
        const auto length = in.tellg();
        if (length < static_cast<std::streamoff>(headerBytes) || length > static_cast<std::streamoff>(maxEntryBytes))
            throw std::runtime_error("Invalid cache file size");
        std::vector<char> data(static_cast<std::size_t>(length)); in.seekg(0);
        if (!in.read(data.data(), length)) throw std::runtime_error("Incomplete cache read");
        Reader r{data};
        if (r.U64() != magic || r.U32() != schema || r.U64() != key[0] || r.U64() != key[1])
            throw std::runtime_error("Stale cache header");
        const auto bytes = r.U64(); const auto checksum = r.U32();
        if (bytes != r.data.size() || checksum != Checksum(r.data))
            throw std::runtime_error("Damaged cache payload");
        data.erase(data.begin(), data.begin() + headerBytes);
        return data;
    }
    catch (const std::exception&) { Reject(key); return {}; }
}

void EditorGeometryCache::Reject(Key key)
{
    ++statistics.errors;
    std::error_code error;
    std::filesystem::remove(File(key), error);
}

void EditorGeometryCache::Write(Key key, std::span<const char> payload)
{
    if (directory.empty() || !budget || payload.size() + headerBytes > std::min(budget, maxEntryBytes)) return;
    std::filesystem::path temporary;
    try
    {
        std::error_code error;
        std::filesystem::create_directories(directory, error);
        if (error) { ++statistics.errors; return; }
        static std::atomic<std::uint64_t> sequence{0};
        temporary = File(key);
        temporary += "." + std::to_string(SDL_GetPerformanceCounter()) + "." + std::to_string(++sequence) + ".tmp";
        Writer header; header.U64(magic); header.U32(schema); header.U64(key[0]); header.U64(key[1]);
        header.U64(payload.size()); header.U32(Checksum(payload));
        std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
        out.write(header.data.data(), static_cast<std::streamsize>(header.data.size()));
        out.write(payload.data(), static_cast<std::streamsize>(payload.size())); out.close();
        if (!out) throw std::runtime_error("Cannot write geometry cache");
        // A reader never sees a partially written entry. Concurrent editors may
        // produce the same key; either complete result is suitable.
        std::filesystem::rename(temporary, File(key), error);
        if (error) throw std::runtime_error("Cannot install geometry cache entry");
        ++statistics.writes; bytesWritten += payload.size() + headerBytes;
        if (bytesWritten >= std::min<std::uint64_t>(budget, 64ull * 1024 * 1024)) { Prune(); bytesWritten = 0; }
    }
    catch (const std::exception&)
    {
        ++statistics.errors;
        std::error_code error; if (!temporary.empty()) std::filesystem::remove(temporary, error);
    }
}

void EditorGeometryCache::Prune()
{
    if (directory.empty() || !budget) return;
    std::error_code error;
    struct Entry { std::filesystem::path file; std::filesystem::file_time_type time; std::uint64_t bytes; };
    std::vector<Entry> entries; std::uint64_t total = 0;
    for (std::filesystem::directory_iterator it(directory, error), end; !error && it != end; it.increment(error))
    {
        const auto& file = *it;
        if (file.path().extension() != ".bgeom" || !file.is_regular_file(error) || error) continue;
        const auto bytes = file.file_size(error); if (error) break;
        const auto time = file.last_write_time(error); if (error) break;
        entries.push_back({file.path(), time, bytes}); total += bytes;
    }
    std::ranges::sort(entries, {}, &Entry::time);
    for (const auto& entry : entries)
    {
        if (total <= budget) break;
        if (std::filesystem::remove(entry.file, error)) total -= entry.bytes;
        error.clear();
    }
}

std::optional<LakeGeometry> EditorGeometryCache::Water(const WaterDefinition& definition)
{
    Writer input; input.U32(2); input.U32(static_cast<std::uint32_t>(definition.index()));
    const auto points = [&](const auto& values)
    { input.List(values, [&](const auto& p) { input.Vec(p.position); input.Vec(p.incoming); input.Vec(p.outgoing); }); };
    std::visit([&](const auto& source)
    {
        input.F64(source.depth);
        if constexpr (std::is_same_v<std::decay_t<decltype(source)>, LakeDefinition>) points(source.points);
        else for (const auto& bank : source.banks) points(bank);
    }, definition);
    const auto key = KeyFor(input);
    if (auto data = Read(key))
    {
        try
        {
            Reader r{*data}; auto geometry = r.WaterData(); r.End();
            ++statistics.hits; --statistics.misses;
            std::error_code error; std::filesystem::last_write_time(File(key), std::filesystem::file_time_type::clock::now(), error);
            return geometry;
        }
        catch (const std::exception&) { Reject(key); }
    }
    auto geometry = std::visit([](const auto& source)
    {
        if constexpr (std::is_same_v<std::decay_t<decltype(source)>, LakeDefinition>) return LakeGeometry::Build(source);
        else return RiverGeometry::Build(source);
    }, definition);
    if (geometry)
    {
        try { Writer payload; payload.WaterData(*geometry); Write(key, payload.data); }
        catch (const std::exception&) { ++statistics.errors; }
    }
    return geometry;
}

std::optional<LakeGeometry> EditorGeometryCache::Lake(const LakeDefinition& definition) { return Water(definition); }
std::optional<LakeGeometry> EditorGeometryCache::River(const RiverDefinition& definition) { return Water(definition); }

std::shared_ptr<const Mesh> EditorGeometryCache::Fill(std::span<const RoadGeometry* const> footprints,
    std::span<const std::array<Vector3, 2>> links, std::span<const std::vector<Vector3>> exclusions,
    std::span<const std::array<Vector3, 2>> junctionSeams, TerrainFillScope scope,
    std::span<const RoadGeometry* const> supportingFootprints)
{
    Writer input; input.U32(1); input.U32(static_cast<std::uint32_t>(scope));
    input.U32(!supportingFootprints.empty());
    input.List(footprints, [&](const auto* g)
    {
        input.U32(std::ranges::find(supportingFootprints, g) != supportingFootprints.end()); Footprint(input, *g);
    });
    const auto segment = [&](const auto& edge) { for (auto p : edge) input.Vec(p); };
    input.List(links, segment); input.List(exclusions, [&](const auto& rim) { input.Points(rim); });
    input.List(junctionSeams, segment);
    const auto key = KeyFor(input);
    if (auto data = Read(key))
    {
        try
        {
            Reader r{*data}; auto mesh = r.MeshData(); r.End();
            ++statistics.hits; --statistics.misses;
            std::error_code error; std::filesystem::last_write_time(File(key), std::filesystem::file_time_type::clock::now(), error);
            return mesh;
        }
        catch (const std::exception&) { Reject(key); }
    }
    auto mesh = BuildTerrainFill(footprints, links, exclusions, junctionSeams, scope, supportingFootprints);
    if (mesh)
    {
        try { Writer payload; payload.MeshData(*mesh); Write(key, payload.data); }
        catch (const std::exception&) { ++statistics.errors; }
    }
    return mesh;
}
