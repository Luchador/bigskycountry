#include "runtime_config.hpp"
#include <SDL3/SDL.h>
#include <charconv>
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
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Cannot read %s: %s; using default graphics settings", path.c_str(), SDL_GetError());
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
        if (!Equal(section, "Graphics") || equals == line.npos || !Equal(Trim(line.substr(0, equals)), "MSAA")) continue;
        const auto value = Trim(line.substr(equals + 1));
        int samples = 4;
        const auto parsed = std::from_chars(value.data(), value.data() + value.size(), samples);
        if (parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size() && (samples == 0 || samples == 2 || samples == 4))
            result.msaa = samples;
        else
        {
            result.msaa = 4;
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "%s:%u: MSAA must be 0, 2 or 4; using 4", path.c_str(), lineNumber);
        }
    }
    return result;
}

const RuntimeConfig& RuntimeConfig::Get()
{
    static const RuntimeConfig config = [] {
        const char* base = SDL_GetBasePath();
        return base ? Load(std::string(base) + "config.ini") : RuntimeConfig{};
    }();
    return config;
}
