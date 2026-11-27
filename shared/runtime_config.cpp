#include "runtime_config.hpp"
#include <SDL3/SDL.h>
#include <charconv>
#include <cmath>
#include <memory>
#include <string_view>

namespace
{
    std::string_view Trim(std::string_view text)
    {
        const auto first = text.find_first_not_of(" \t\r\n");
        if (first == text.npos) return text.substr(text.size());
        return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
    }
    bool Equal(std::string_view a, std::string_view b)
    {
        return a.size() == b.size() && SDL_strncasecmp(a.data(), b.data(), a.size()) == 0;
    }
}

RuntimeConfig RuntimeConfig::Load(const std::string& path)
{
    RuntimeConfig result;
    SDL_PathInfo info{};
    if (!SDL_GetPathInfo(path.c_str(), &info)) return result; // Missing file uses defaults.
    std::size_t size = 0;
    std::unique_ptr<char, decltype(&SDL_free)> bytes(static_cast<char*>(SDL_LoadFile(path.c_str(), &size)), SDL_free);
    if (!bytes)
    {
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Cannot read %s: %s; using default runtime settings", path.c_str(), SDL_GetError());
        return result;
    }
    std::string_view contents(bytes.get(), size), section;
    if (contents.starts_with("\xef\xbb\xbf")) contents.remove_prefix(3);
    unsigned lineNumber = 0;
    while (!contents.empty())
    {
        const auto newline = contents.find('\n');
        auto line = contents.substr(0, newline);
        contents = newline == contents.npos ? std::string_view{} : contents.substr(newline + 1);
        ++lineNumber;
        line = Trim(line.substr(0, line.find_first_of(";#")));
        if (line.empty()) continue;
        if (line.front() == '[')
        {
            section = line.back() == ']' ? Trim(line.substr(1, line.size() - 2)) : std::string_view{};
            continue;
        }
        const auto equals = line.find('=');
        if (equals == line.npos) continue;
        const auto key = Trim(line.substr(0, equals));
        const auto value = Trim(line.substr(equals + 1));
        if (Equal(section, "Graphics"))
        {
            float* setting = nullptr;
            float low = 0, high = 0, fallback = 0;
            if (Equal(key, "BloomIntensity")) { setting = &result.bloomIntensity; high = 100; fallback = .08f; }
            else if (Equal(key, "BloomRadius")) { setting = &result.bloomRadius; low = .5f; high = 32; fallback = 1; }
            else if (Equal(key, "BloomThreshold")) { setting = &result.bloomThreshold; high = 16; fallback = 1; }
            if (setting)
            {
                float number = 0;
                const auto parsed = std::from_chars(value.data(), value.data() + value.size(), number);
                const bool valid = parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size() &&
                    std::isfinite(number) && number >= low && number <= high;
                *setting = valid ? number : fallback;
                if (!valid) SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "%s:%u: %.*s must be %.2f..%.2f; using %.2f",
                    path.c_str(), lineNumber, int(key.size()), key.data(), low, high, fallback);
                continue;
            }
        }
        int* budget = nullptr;
        if (Equal(section, "Streaming"))
        {
            if (Equal(key, "GeometryMiB")) budget = &result.streamGeometryMiB;
            else if (Equal(key, "AssetsMiB")) budget = &result.streamAssetsMiB;
            else if (Equal(key, "CollisionMiB")) budget = &result.streamCollisionMiB;
            else continue;
        }
        else if (!Equal(section, "Graphics") ||
                 (!Equal(key, "MSAA") && !Equal(key, "SSAO") && !Equal(key, "SSAOSamples") && !Equal(key, "Bloom"))) continue;
        int number = 0;
        const auto parsed = std::from_chars(value.data(), value.data() + value.size(), number);
        const bool integer = parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size();
        if (budget)
        {
            const bool valid = integer && (number == 0 || (number >= 64 && number <= 16384));
            *budget = valid ? number : 0;
            if (!valid) SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                "%s:%u: %.*s must be 0 or 64..16384 MiB; using the project budget",
                path.c_str(), lineNumber, int(key.size()), key.data());
        }
        else if (Equal(key, "MSAA"))
        {
            const bool valid = integer && (number == 0 || number == 2 || number == 4);
            result.msaa = valid ? number : 4;
            if (!valid) SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "%s:%u: MSAA must be 0, 2 or 4; using 4", path.c_str(), lineNumber);
        }
        else if (Equal(key, "SSAO") || Equal(key, "Bloom"))
        {
            const bool valid = integer && (number == 0 || number == 1);
            (Equal(key, "Bloom") ? result.bloom : result.ssao) = valid ? number != 0 : true;
            if (!valid) SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "%s:%u: %.*s must be 0 or 1; using 1",
                path.c_str(), lineNumber, int(key.size()), key.data());
        }
        else
        {
            const bool valid = integer && number >= 1 && number <= 64;
            result.ssaoSamples = valid ? number : 16;
            if (!valid) SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "%s:%u: SSAOSamples must be 1..64; using 16", path.c_str(), lineNumber);
        }
    }
    return result;
}

StreamingSettings RuntimeConfig::ApplyStreamingOverrides(StreamingSettings settings) const
{
    if (streamGeometryMiB) settings.geometryBytes = std::size_t(streamGeometryMiB) * StreamingSettings::MiB;
    if (streamAssetsMiB) settings.assetBytes = std::size_t(streamAssetsMiB) * StreamingSettings::MiB;
    if (streamCollisionMiB) settings.collisionBytes = std::size_t(streamCollisionMiB) * StreamingSettings::MiB;
    return settings;
}

const RuntimeConfig& RuntimeConfig::Get()
{
    static const RuntimeConfig config = [] {
        const char* base = SDL_GetBasePath();
        return base ? Load(std::string(base) + "config.ini") : RuntimeConfig{};
    }();
    return config;
}
