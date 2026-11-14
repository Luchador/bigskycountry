#include "project_settings.hpp"
#include <SDL3/SDL.h>
#include <cJSON.h>
#include <cmath>
#include <filesystem>
#include <memory>

ProjectSettings g_ProjectSettings;

namespace
{
    constexpr auto filename = "project-settings.json";
    constexpr auto distanceKey = "railroadDetailDistanceMeters";
    using Json = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;

    Json ReadSettings(const std::string& path)
    {
        std::error_code error;
        const std::filesystem::path file(std::u8string(path.begin(), path.end()));
        if (!std::filesystem::exists(file, error) && !error)
            return Json(cJSON_CreateObject(), cJSON_Delete);

        std::size_t size = 0;
        std::unique_ptr<void, decltype(&SDL_free)> data(SDL_LoadFile(path.c_str(), &size), SDL_free);
        Json settings(nullptr, cJSON_Delete);
        if (!data) return settings;
        const char* end = nullptr;
        settings.reset(cJSON_ParseWithLengthOpts(static_cast<const char*>(data.get()), size + 1, &end, true));
        if (!cJSON_IsObject(settings.get()) || end != static_cast<const char*>(data.get()) + size)
        {
            SDL_SetError("Invalid project settings in %s", path.c_str());
            settings.reset();
        }
        return settings;
    }

    bool WriteSettings(const std::string& path, const char* data)
    {
        SDL_Time now;
        if (!SDL_GetCurrentTime(&now)) return false;
        const auto temp = path + "." + std::to_string(now) + ".tmp";
        SDL_IOStream* stream = SDL_IOFromFile(temp.c_str(), "wbx");
        if (!stream) return false;
        const bool written = SDL_SaveFile_IO(stream, data, SDL_strlen(data), false);
        if (SDL_CloseIO(stream) && written && SDL_RenamePath(temp.c_str(), path.c_str()))
            return true;
        const std::string error = SDL_GetError();
        SDL_RemovePath(temp.c_str());
        return SDL_SetError("Cannot save %s: %s", path.c_str(), error.c_str());
    }
}

bool ProjectSettings::Load(AssetRoot location)
{
    const auto path = AssetPaths::Root(location) + filename;
    auto settings = ReadSettings(path);
    if (!settings) return false;
    const auto* distance = cJSON_GetObjectItemCaseSensitive(settings.get(), distanceKey);
    if (distance && (!cJSON_IsNumber(distance) || !std::isfinite(distance->valuedouble) || distance->valuedouble < 0))
        return SDL_SetError("Invalid railroad detail distance in %s", path.c_str());
    railroadDetailDistance = distance ? distance->valuedouble : defaultRailroadDetailDistance;
    return true;
}

bool ProjectSettings::SaveRailroadDetailDistance(double meters)
{
    if (!std::isfinite(meters) || meters < 0)
        return SDL_SetError("Enter a distance of zero or more meters.");
    const auto source = AssetPaths::Root(AssetRoot::Project) + filename;
    auto settings = ReadSettings(source);
    if (!settings) return false;
    cJSON_DeleteItemFromObjectCaseSensitive(settings.get(), distanceKey);
    if (!cJSON_AddNumberToObject(settings.get(), distanceKey, meters)) return SDL_OutOfMemory();
    std::unique_ptr<char, decltype(&cJSON_free)> data(cJSON_Print(settings.get()), cJSON_free);
    if (!data) return SDL_OutOfMemory();
    if (!WriteSettings(source, data.get())) return false;
    railroadDetailDistance = meters;

    // Publish immediately for a game launched from this build. Other builds
    // and packaged games get the same source settings through asset staging.
    if (WriteSettings(AssetPaths::Root(AssetRoot::Cooked) + filename, data.get())) return true;
    const std::string error = SDL_GetError();
    return SDL_SetError("Saved project settings, but could not update the game build: %s", error.c_str());
}
