#include "material.hpp"

#include <SDL3/SDL.h>
#include <cJSON.h>
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string_view>

namespace
{
    unsigned materialRevision = 0, imageRevision = 0;
    using Json = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;
    constexpr std::array<const char*, materialSlotCount> keys{"baseColor", "normal", "orm", "emissive", "opacity"};

    Json ReadDocument(const std::string& file)
    {
        size_t size = 0;
        std::unique_ptr<void, decltype(&SDL_free)> bytes(SDL_LoadFile(file.c_str(), &size), SDL_free);
        Json json(nullptr, cJSON_Delete);
        if (!bytes)
            return json;
        if (!std::memchr(bytes.get(), 0, size))
            json.reset(cJSON_ParseWithLengthOpts(static_cast<const char*>(bytes.get()), size + 1, nullptr, true));
        const auto* version = cJSON_GetObjectItemCaseSensitive(json.get(), "version");
        if (!cJSON_IsObject(json.get()) || !cJSON_IsNumber(version) || version->valuedouble != 1)
        {
            SDL_SetError("Invalid or unsupported material: %s", file.c_str());
            json.reset();
        }
        return json;
    }
}

bool Material::ValidTexturePath(const std::string& path)
{
    if (path.empty() || path.find_first_of("\\:") != path.npos || path.find('\0') != path.npos)
        return false;
    std::string_view rest(path);
    while (true)
    {
        const auto slash = rest.find('/');
        const auto part = rest.substr(0, slash);
        if (part.empty() || part == "." || part == "..")
            return false;
        if (slash == rest.npos)
            return true;
        rest.remove_prefix(slash + 1);
    }
}

bool Material::ValidAssetPath(const std::string& path)
{
    return ValidTexturePath(path) && path.size() > 5 &&
           SDL_strcasecmp(path.c_str() + path.size() - 5, ".bmat") == 0;
}

std::string Material::TextureRoot(bool authoring)
{
#ifdef BIGSKY_PROJECT_ASSETS
    std::error_code error;
    if (authoring && std::filesystem::is_directory(BIGSKY_PROJECT_ASSETS, error))
    {
        const auto root = std::filesystem::path(BIGSKY_PROJECT_ASSETS).generic_u8string();
        return std::string(root.begin(), root.end()) + "/textures/";
    }
#endif
    const char* base = SDL_GetBasePath();
    return base ? std::string(base) + "assets/textures/" : std::string{};
}

std::string Material::Resolve(const std::string& root, const std::string& relative)
{
    const auto file = root + relative;
    SDL_PathInfo info{};
    if (!root.empty() && SDL_GetPathInfo(file.c_str(), &info) && info.type == SDL_PATHTYPE_FILE)
        return file;
    return TextureRoot() + relative;
}

unsigned Material::Revision() { return materialRevision; }
unsigned Material::ImageRevision() { return imageRevision; }
void Material::RefreshAssets() { ++materialRevision; ++imageRevision; }

bool Material::Load(const std::string& file)
{
    auto json = ReadDocument(file);
    if (!json)
        return false;
    Material material;
    if (const auto* type = cJSON_GetObjectItemCaseSensitive(json.get(), "type"))
    {
        const char* value = cJSON_GetStringValue(type);
        if (!value)
            return SDL_SetError("Invalid material type");
        const auto match = std::find_if(materialTypes.begin(), materialTypes.end(),
                                        [value](const auto& entry) { return SDL_strcmp(value, entry.key) == 0; });
        if (match == materialTypes.end())
            return SDL_SetError("Unsupported material type: %s", value);
        material.type = static_cast<MaterialType>(match - materialTypes.begin());
    }
    const auto* maps = cJSON_GetObjectItemCaseSensitive(json.get(), "textures");
    if (maps && !cJSON_IsObject(maps))
        return SDL_SetError("Invalid material texture list");
    for (std::size_t i = 0; i < keys.size(); ++i)
    {
        if (const auto* entry = cJSON_GetObjectItemCaseSensitive(maps, keys[i]))
        {
            const char* value = cJSON_GetStringValue(entry);
            if (!value || !ValidTexturePath(value))
                return SDL_SetError("Invalid %s texture path", keys[i]);
            material.textures[i] = value;
        }
    }
    *this = std::move(material);
    return true;
}

bool Material::Save(const std::string& file) const
{
    auto json = ReadDocument(file);
    if (!json)
        return false;
    cJSON_DeleteItemFromObjectCaseSensitive(json.get(), "type");
    if (!cJSON_AddStringToObject(json.get(), "type", materialTypes.at(static_cast<std::size_t>(type)).key))
        return SDL_OutOfMemory();
    auto* maps = cJSON_GetObjectItemCaseSensitive(json.get(), "textures");
    if (!maps)
        maps = cJSON_AddObjectToObject(json.get(), "textures");
    if (!cJSON_IsObject(maps))
        return SDL_SetError("Cannot update material texture list");
    for (std::size_t i = 0; i < keys.size(); ++i)
    {
        cJSON_DeleteItemFromObjectCaseSensitive(maps, keys[i]);
        if (textures[i].empty())
            continue;
        if (!ValidTexturePath(textures[i]))
            return SDL_SetError("Invalid texture path");
        if (!cJSON_AddStringToObject(maps, keys[i], textures[i].c_str()))
            return SDL_OutOfMemory();
    }
    std::unique_ptr<char, decltype(&cJSON_free)> data(cJSON_Print(json.get()), cJSON_free);
    if (!data)
        return SDL_OutOfMemory();
    SDL_Time now;
    if (!SDL_GetCurrentTime(&now))
        return false;
    const auto temp = file + "." + std::to_string(now) + ".tmp";
    SDL_IOStream* output = SDL_IOFromFile(temp.c_str(), "wbx");
    if (!output)
        return false;
    const auto size = SDL_strlen(data.get());
    const bool written = SDL_WriteIO(output, data.get(), size) == size;
    if (!SDL_CloseIO(output) || !written || !SDL_RenamePath(temp.c_str(), file.c_str()))
    {
        const std::string error = SDL_GetError();
        SDL_RemovePath(temp.c_str());
        return SDL_SetError("Cannot save material: %s", error.c_str());
    }
    ++materialRevision;
    return true;
}
