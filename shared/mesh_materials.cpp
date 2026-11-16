#include "mesh_materials.hpp"
#include "material.hpp"
#include <algorithm>
#include <iomanip>
#include <filesystem>
#include <locale>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace
{
    std::mutex cacheMutex;
    std::unordered_map<std::string, std::weak_ptr<MeshMaterials>> cache;
    std::string File(AssetRoot root, const std::string& path)
    { return AssetPaths::Root(root) + "models/" + path + ".bmesh"; }
}

const std::string& Mesh::MaterialAt(std::size_t index) const
{
    static const std::string empty;
    if (!materials || parts.empty()) return empty;
    const auto it = std::upper_bound(parts.begin(), parts.end(), index,
        [](std::size_t i, const MeshPart& p) { return i < p.firstIndex; });
    if (it == parts.begin()) return empty;
    const auto& part = *std::prev(it);
    return index < part.firstIndex + part.indexCount && part.slot < materials->slots.size() ?
        materials->slots[part.slot].material : empty;
}

bool Mesh::HasMaterials() const
{
    return materials && std::ranges::any_of(materials->slots,
        [](const MeshMaterialSlot& slot) { return !slot.material.empty(); });
}

bool MeshMaterialIO::Load(const std::string& file, std::vector<MeshMaterialSlot>& slots)
{
    std::error_code error;
    const bool exists = std::filesystem::exists(std::filesystem::path(std::u8string(file.begin(), file.end())), error);
    if (error) return SDL_SetError("Cannot inspect mesh material settings: %s", error.message().c_str());
    if (!exists) return true;
    std::size_t size = 0;
    std::unique_ptr<char, decltype(&SDL_free)> data(static_cast<char*>(SDL_LoadFile(file.c_str(), &size)), SDL_free);
    if (!data) return false;
    if (size > 4 * 1024 * 1024) return SDL_SetError("Mesh material settings are too large");
    std::istringstream input(std::string(data.get(), size));
    input.imbue(std::locale::classic());
    std::string magic; int version = 0; std::size_t count = 0;
    if (!(input >> magic >> version >> count) || magic != "BSC_MESH_MATERIALS" || version != 1 || count > 65536)
        return SDL_SetError("Invalid mesh material settings: %s", file.c_str());
    auto next = slots;
    for (std::size_t i = 0; i < count; ++i)
    {
        std::string name, material;
        if (!(input >> std::quoted(name) >> std::quoted(material)) ||
            (!material.empty() && !Material::ValidAssetPath(material)))
            return SDL_SetError("Invalid mesh material slot: %s", file.c_str());
        // Keep assignments after a re-export that reorders uniquely named slots.
        if (i < next.size() && next[i].name == name) next[i].material = material;
        else if (std::ranges::count(next, name, &MeshMaterialSlot::name) == 1)
            std::ranges::find(next, name, &MeshMaterialSlot::name)->material = material;
    }
    input >> std::ws;
    if (!input.eof()) return SDL_SetError("Trailing mesh material data: %s", file.c_str());
    slots = std::move(next);
    return true;
}

bool MeshMaterialIO::Save(const std::string& file, const std::vector<MeshMaterialSlot>& slots)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "BSC_MESH_MATERIALS 1\n" << slots.size() << '\n';
    for (const auto& slot : slots)
    {
        if (!slot.material.empty() && !Material::ValidAssetPath(slot.material)) return SDL_SetError("Invalid material path");
        output << std::quoted(slot.name) << ' ' << std::quoted(slot.material) << '\n';
    }
    SDL_Time time;
    if (!SDL_GetCurrentTime(&time)) return false;
    const auto temporary = file + "." + std::to_string(time) + ".tmp";
    auto* stream = SDL_IOFromFile(temporary.c_str(), "wbx");
    if (!stream) return false;
    const auto text = output.str();
    const bool written = SDL_WriteIO(stream, text.data(), text.size()) == text.size();
    if (!SDL_CloseIO(stream) || !written || !SDL_RenamePath(temporary.c_str(), file.c_str()))
    {
        const std::string reason = SDL_GetError();
        SDL_RemovePath(temporary.c_str());
        return SDL_SetError("Cannot save mesh materials: %s", reason.c_str());
    }
    return true;
}

bool MeshMaterialIO::Bind(Mesh& mesh, AssetRoot root)
{
    if (!mesh.materials) return true;
    const auto file = File(root, mesh.assetPath);
    std::lock_guard lock(cacheMutex);
    std::erase_if(cache, [](const auto& entry) { return entry.second.expired(); });
    if (auto existing = cache[file].lock())
    {
        const auto& a = existing->slots;
        const auto& b = mesh.materials->slots;
        if (a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(),
            [](const auto& x, const auto& y) { return x.name == y.name; }))
        { mesh.materials = std::move(existing); return true; }
    }
    if (!Load(file, mesh.materials->slots)) return false;
    mesh.materials->file = file;
    mesh.materials->root = root;
    cache[file] = mesh.materials;
    return true;
}

bool MeshMaterialIO::Assign(const Mesh& mesh, std::size_t slot, const std::string& material)
{
    if (!mesh.materials || mesh.materials->file.empty() || slot >= mesh.materials->slots.size() || !Material::ValidTexturePath(mesh.assetPath))
        return SDL_SetError("Invalid mesh material slot");
    if (!material.empty())
    {
        Material definition;
        if (!Material::ValidAssetPath(material) || !definition.Load(AssetPaths::Root(mesh.materials->root) + "textures/" + material))
            return SDL_SetError("Cannot load assigned material: %s", material.c_str());
    }
    auto next = mesh.materials->slots;
    next[slot].material = material;
    if (!Save(mesh.materials->file, next)) return false;
    mesh.materials->slots = std::move(next);
    ++mesh.materials->revision;
    return true;
}
