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
    constexpr auto shadowDistanceKey = "shadowDistanceMeters";
    constexpr auto viewDistanceKey = "viewDistanceMeters";
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
    const auto* view = cJSON_GetObjectItemCaseSensitive(settings.get(), viewDistanceKey);
    if (view && (!cJSON_IsNumber(view) || !std::isfinite(view->valuedouble) ||
                 view->valuedouble < minViewDistance || view->valuedouble > maxViewDistance))
        return SDL_SetError("Invalid view distance in %s", path.c_str());
    const auto* shadow = cJSON_GetObjectItemCaseSensitive(settings.get(), shadowDistanceKey);
    if (shadow && (!cJSON_IsNumber(shadow) || !std::isfinite(shadow->valuedouble) ||
                   shadow->valuedouble < 0 || shadow->valuedouble > maxShadowDistance))
        return SDL_SetError("Invalid shadow distance in %s", path.c_str());
    StreamingSettings loaded;
    const auto readBudget = [&](const char* key, std::size_t& bytes) {
        const auto* value = cJSON_GetObjectItemCaseSensitive(settings.get(), key);
        if (!value) return true;
        if (!cJSON_IsNumber(value) || !std::isfinite(value->valuedouble) || value->valuedouble < 64 || value->valuedouble > 16384)
            return SDL_SetError("Invalid %s (expected 64 to 16384 MiB)", key);
        bytes = static_cast<std::size_t>(value->valuedouble * StreamingSettings::MiB);
        return true;
    };
    if (!readBudget("streamGeometryMiB", loaded.geometryBytes) || !readBudget("streamAssetsMiB", loaded.assetBytes) ||
        !readBudget("streamCollisionMiB", loaded.collisionBytes)) return false;
    shadowDistance = shadow ? shadow->valuedouble : defaultShadowDistance;
    streaming = loaded;
    railroadDetailDistance = distance ? distance->valuedouble : defaultRailroadDetailDistance;
    viewDistance = view ? view->valuedouble : defaultViewDistance;
    return true;
}

bool ProjectSettings::SaveDistances(double meters, double viewMeters)
{
    return SaveOptions(meters, viewMeters, streaming);
}

bool ProjectSettings::SaveOptions(double meters, double viewMeters, const StreamingSettings& budgets, double shadowMeters)
{
    if (!std::isfinite(shadowMeters) || shadowMeters < 0 || shadowMeters > maxShadowDistance)
        return SDL_SetError("Enter a shadow distance between 0 and 10 kilometers (0 disables shadows).");
    for (auto bytes : {budgets.geometryBytes, budgets.assetBytes, budgets.collisionBytes})
        if (bytes < 64 * StreamingSettings::MiB || bytes > 16384 * StreamingSettings::MiB)
            return SDL_SetError("Streaming budgets must be between 64 and 16384 MiB.");
    if (!std::isfinite(meters) || meters < 0)
        return SDL_SetError("Enter a distance of zero or more meters.");
    if (!std::isfinite(viewMeters) || viewMeters < minViewDistance || viewMeters > maxViewDistance)
        return SDL_SetError("Enter a view distance between 0.1 and 128 kilometers.");
    const auto source = AssetPaths::Root(AssetRoot::Project) + filename;
    auto settings = ReadSettings(source);
    if (!settings) return false;
    cJSON_DeleteItemFromObjectCaseSensitive(settings.get(), distanceKey);
    if (!cJSON_AddNumberToObject(settings.get(), distanceKey, meters)) return SDL_OutOfMemory();
    cJSON_DeleteItemFromObjectCaseSensitive(settings.get(), viewDistanceKey);
    if (!cJSON_AddNumberToObject(settings.get(), viewDistanceKey, viewMeters)) return SDL_OutOfMemory();
    cJSON_DeleteItemFromObjectCaseSensitive(settings.get(), shadowDistanceKey);
    if (!cJSON_AddNumberToObject(settings.get(), shadowDistanceKey, shadowMeters)) return SDL_OutOfMemory();
    const auto addBudget = [&](const char* key, std::size_t bytes) {
        cJSON_DeleteItemFromObjectCaseSensitive(settings.get(), key);
        return cJSON_AddNumberToObject(settings.get(), key, double(bytes) / StreamingSettings::MiB) != nullptr;
    };
    if (!addBudget("streamGeometryMiB", budgets.geometryBytes) || !addBudget("streamAssetsMiB", budgets.assetBytes) ||
        !addBudget("streamCollisionMiB", budgets.collisionBytes)) return SDL_OutOfMemory();
    std::unique_ptr<char, decltype(&cJSON_free)> data(cJSON_Print(settings.get()), cJSON_free);
    if (!data) return SDL_OutOfMemory();
    if (!WriteSettings(source, data.get())) return false;
    shadowDistance = shadowMeters;
    streaming = budgets;
    railroadDetailDistance = meters;
    viewDistance = viewMeters;

    // Publish immediately for a game launched from this build. Other builds
    // and packaged games get the same source settings through asset staging.
    if (WriteSettings(AssetPaths::Root(AssetRoot::Cooked) + filename, data.get())) return true;
    const std::string error = SDL_GetError();
    return SDL_SetError("Saved project settings, but could not update the game build: %s", error.c_str());
}
