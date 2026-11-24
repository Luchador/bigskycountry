#include "mesh_ao.hpp"
#include <algorithm>
#include <bit>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <string_view>
#include <unordered_map>

namespace
{
    std::mutex cacheMutex;
    std::unordered_map<std::string, std::weak_ptr<MeshAOState>> cache;
    constexpr Uint32 version = 1;
    constexpr Uint64 maxBytes = 512ull * 1024 * 1024;

    std::uint64_t Fingerprint(const Mesh& mesh)
    {
        std::uint64_t hash = 14695981039346656037ull;
        const auto word = [&](std::uint64_t value) {
            for (int i = 0; i < 8; ++i) { hash ^= (value >> (i * 8)) & 255; hash *= 1099511628211ull; }
        };
        word(mesh.lods.size() + 1);
        for (std::size_t l = 0; l <= mesh.lods.size(); ++l)
        {
            const auto& level = mesh.Level(l);
            word(level.vertices.size()); word(level.indices.size());
            for (const auto& v : level.vertices)
            {
                for (double n : {v.position.x, v.position.y, v.position.z, v.normal.x, v.normal.y, v.normal.z})
                    word(std::bit_cast<std::uint64_t>(n == 0 ? 0.0 : n));
                word(std::bit_cast<Uint32>(v.uv.x)); word(std::bit_cast<Uint32>(v.uv.y));
            }
            for (auto index : level.indices) word(index);
        }
        return hash;
    }
    bool Valid(const Mesh& mesh, const MeshAOData& data)
    {
        if (data.cleared) return data.alpha.empty();
        if (data.alpha.size() != mesh.lods.size() + 1) return false;
        Uint64 bytes = 28;
        for (std::size_t l = 0; l < data.alpha.size(); ++l)
        {
            bytes += 8 + data.alpha[l].size() * 4ull;
            if (bytes > maxBytes) return false;
            if (data.alpha[l].size() != mesh.Level(l).vertices.size() ||
                !std::ranges::all_of(data.alpha[l], [](float a) { return std::isfinite(a) && a >= 0 && a <= 1; })) return false;
        }
        return true;
    }
    struct CloseIO { void operator()(SDL_IOStream* io) const { if (io) SDL_CloseIO(io); } };
}

MeshAO::Snapshot MeshAO::Current(const Mesh& mesh)
{ return mesh.ambientOcclusion ? mesh.ambientOcclusion->data.load() : nullptr; }

float MeshAO::Alpha(const Mesh& mesh, std::size_t vertex, const Snapshot& data)
{
    if (data)
    {
        if (data->cleared) return 1;
        if (mesh.aoLevel < data->alpha.size() && vertex < data->alpha[mesh.aoLevel].size())
            return data->alpha[mesh.aoLevel][vertex];
    }
    return mesh.vertexColors.size() == mesh.vertices.size() ? std::clamp(mesh.vertexColors[vertex].a, 0.f, 1.f) : 1.f;
}

bool MeshAO::Bind(Mesh& mesh, AssetRoot root)
{
    const auto file = AssetPaths::Root(root) + "models/" + mesh.assetPath + ".bao";
    // Avoid hashing large source meshes more than once per import.
    const auto fingerprint = Fingerprint(mesh);
    std::lock_guard lock(cacheMutex);
    std::erase_if(cache, [](const auto& entry) { return entry.second.expired(); });
    auto state = cache[file].lock();
    if (!state || state->fingerprint != fingerprint)
    {
        state = std::make_shared<MeshAOState>(); state->file = file; state->fingerprint = fingerprint;
        std::error_code error;
        const auto path = std::filesystem::path(std::u8string(file.begin(), file.end()));
        const bool exists = std::filesystem::exists(path, error);
        if (error) return SDL_SetError("Cannot inspect mesh AO: %s", error.message().c_str());
        if (exists)
        {
            std::unique_ptr<SDL_IOStream, CloseIO> input(SDL_IOFromFile(file.c_str(), "rb"));
            if (!input) return false;
            const auto size = SDL_GetIOSize(input.get());
            char magic[8]; Uint32 v = 0, flags = 0, levels = 0; Uint64 signature = 0;
            const bool header = size >= 28 && size <= Sint64(maxBytes) && SDL_ReadIO(input.get(), magic, 8) == 8 &&
                std::string_view(magic, 8) == std::string_view("BSCAO\0\0\0", 8);
            if (!header || !SDL_ReadU32LE(input.get(), &v) || v != version || !SDL_ReadU64LE(input.get(), &signature) ||
                !SDL_ReadU32LE(input.get(), &flags) || flags > 1 || !SDL_ReadU32LE(input.get(), &levels) || !levels || levels > Mesh::maxLODCount)
                return SDL_SetError("Invalid mesh AO file: %s", file.c_str());
            if (signature == fingerprint && levels == mesh.lods.size() + 1)
            {
                auto data = std::make_shared<MeshAOData>(); data->cleared = flags == 1;
                if (!data->cleared)
                {
                    data->alpha.resize(levels);
                    for (std::size_t l = 0; l < levels; ++l)
                    {
                        Uint64 count = 0;
                        if (!SDL_ReadU64LE(input.get(), &count) || count != mesh.Level(l).vertices.size() || count > maxBytes / 4 ||
                            count * 4 > Uint64(std::max<Sint64>(0, size - SDL_TellIO(input.get()))))
                            return SDL_SetError("Invalid mesh AO vertex count: %s", file.c_str());
                        auto& values = data->alpha[l]; values.resize(count);
                        for (auto& a : values) { Uint32 bits; if (!SDL_ReadU32LE(input.get(), &bits)) return false; a = std::bit_cast<float>(bits); }
                    }
                }
                if (SDL_TellIO(input.get()) != size || !Valid(mesh, *data)) return SDL_SetError("Invalid mesh AO data: %s", file.c_str());
                state->data.store(std::move(data));
            }
            else SDL_Log("Ignoring stale AO after model geometry changed: %s", file.c_str());
        }
        cache[file] = state;
    }
    mesh.ambientOcclusion = state; mesh.aoLevel = 0;
    for (std::size_t l = 0; l < mesh.lods.size(); ++l)
    { mesh.lods[l].mesh->ambientOcclusion = state; mesh.lods[l].mesh->aoLevel = l + 1; }
    return true;
}

bool MeshAO::Save(const Mesh& mesh, Snapshot data)
{
    if (!mesh.ambientOcclusion || mesh.aoLevel != 0 || !mesh.materials || mesh.materials->root != AssetRoot::Project || !data || !Valid(mesh, *data))
        return SDL_SetError("Invalid editable mesh AO data");
    const auto& state = mesh.ambientOcclusion;
    SDL_Time time; if (!SDL_GetCurrentTime(&time)) return false;
    const auto temporary = state->file + "." + std::to_string(time) + ".tmp";
    auto* out = SDL_IOFromFile(temporary.c_str(), "wbx");
    if (!out) return false;
    bool ok = SDL_WriteIO(out, "BSCAO\0\0\0", 8) == 8 && SDL_WriteU32LE(out, version) &&
        SDL_WriteU64LE(out, state->fingerprint) && SDL_WriteU32LE(out, data->cleared ? 1 : 0) &&
        SDL_WriteU32LE(out, Uint32(mesh.lods.size() + 1));
    for (const auto& level : data->alpha)
    {
        ok = ok && SDL_WriteU64LE(out, level.size());
        for (float a : level) { if (!ok) break; ok = SDL_WriteU32LE(out, std::bit_cast<Uint32>(a)); }
    }
    ok = SDL_CloseIO(out) && ok;
    if (!ok || !SDL_RenamePath(temporary.c_str(), state->file.c_str()))
    {
        const std::string reason = SDL_GetError(); SDL_RemovePath(temporary.c_str());
        return SDL_SetError("Cannot save mesh AO: %s", reason.c_str());
    }
    state->data.store(std::move(data));
    return true;
}

bool MeshAO::Clear(const Mesh& mesh)
{
    auto data = std::make_shared<MeshAOData>(); data->cleared = true;
    return Save(mesh, std::move(data));
}
