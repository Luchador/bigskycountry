#include "world_document.hpp"
#include "road_io.hpp"

#include <SDL3/SDL.h>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <utility>

namespace
{
    using Tag = std::array<char, 4>;
    constexpr Tag worldMagic = {'B', 'S', 'C', 'W'};
    constexpr Uint32 worldVersion = 1;
    constexpr Tag roadTag = {'R', 'O', 'A', 'D'};

    bool ReadRoads(std::string payload, Uint32 version, std::vector<Road>& roads)
    {
        std::istringstream input(std::move(payload));
        input.imbue(std::locale::classic());
        input >> std::ws;
        while (!input.eof())
        {
            Road road = RoadIO::ReadRoad(input, version);
            if (!input || road.points.size() < 2)
            {
                return false;
            }
            roads.push_back(std::move(road));
            input >> std::ws;
        }
        return true;
    }

    bool WriteSection(SDL_IOStream* output, const Tag& tag, Uint32 version, const std::string& payload)
    {
        return SDL_WriteIO(output, tag.data(), tag.size()) == tag.size() &&
               SDL_WriteU32LE(output, version) && SDL_WriteU64LE(output, payload.size()) &&
               SDL_WriteIO(output, payload.data(), payload.size()) == payload.size();
    }
}

bool WorldDocument::Open(const char* documentPath)
{
    WorldDocument loaded;
    if (documentPath)
    {
        loaded.path = documentPath;
    }
    else
    {
        char* directory = SDL_GetPrefPath("BigSkyCountry", "BigSkyCountry");
        if (!directory)
        {
            return false;
        }
        loaded.path = std::string(directory) + "world.wld";
        SDL_free(directory);
    }

    const std::filesystem::path filePath(std::u8string(loaded.path.begin(), loaded.path.end()));
    std::error_code error;
    if (!std::filesystem::exists(filePath, error) && !error)
    {
        *this = std::move(loaded);
        return true;
    }

    using InputFile = std::unique_ptr<SDL_IOStream, decltype(&SDL_CloseIO)>;
    InputFile input(SDL_IOFromFile(loaded.path.c_str(), "rb"), SDL_CloseIO);
    if (!input)
    {
        return false;
    }

    const Sint64 fileSize = SDL_GetIOSize(input.get());
    Tag magic;
    Uint32 version, sectionCount;
    if (fileSize < 0 || SDL_ReadIO(input.get(), magic.data(), magic.size()) != magic.size() ||
        magic != worldMagic || !SDL_ReadU32LE(input.get(), &version) || !SDL_ReadU32LE(input.get(), &sectionCount))
    {
        return SDL_SetError("Invalid world header in '%s'", loaded.path.c_str());
    }
    if (version != worldVersion)
    {
        return SDL_SetError("Unsupported world version %u in '%s'", version, loaded.path.c_str());
    }

    bool sawRoads = false;
    for (Uint32 i = 0; i < sectionCount; ++i)
    {
        Section section;
        Uint64 size;
        if (SDL_ReadIO(input.get(), section.tag.data(), section.tag.size()) != section.tag.size() ||
            !SDL_ReadU32LE(input.get(), &section.version) || !SDL_ReadU64LE(input.get(), &size))
        {
            return SDL_SetError("Incomplete world section header in '%s'", loaded.path.c_str());
        }
        if (section.tag == roadTag && sawRoads)
        {
            return SDL_SetError("Duplicate ROAD section in '%s'", loaded.path.c_str());
        }
        if (section.tag == roadTag && (section.version < 1 || section.version > RoadIO::currentVersion))
        {
            return SDL_SetError("Unsupported ROAD version %u in '%s'", section.version, loaded.path.c_str());
        }
        const Sint64 offset = SDL_TellIO(input.get());
        if (offset < 0 || offset > fileSize || size > static_cast<Uint64>(fileSize - offset) ||
            size > section.payload.max_size())
        {
            return SDL_SetError("Invalid world section length in '%s'", loaded.path.c_str());
        }
        section.payload.resize(static_cast<std::size_t>(size));
        if (SDL_ReadIO(input.get(), section.payload.data(), section.payload.size()) != section.payload.size())
        {
            return SDL_SetError("Incomplete world section in '%s'", loaded.path.c_str());
        }

        if (section.tag == roadTag)
        {
            sawRoads = true;
            if (!ReadRoads(std::move(section.payload), section.version, loaded.roads))
            {
                return SDL_SetError("Invalid road record in '%s'", loaded.path.c_str());
            }
        }
        else
        {
            loaded.otherSections.push_back(std::move(section));
        }
    }
    if (SDL_TellIO(input.get()) != fileSize)
    {
        return SDL_SetError("Unexpected data after the last section in '%s'", loaded.path.c_str());
    }
    for (Road& road : loaded.roads)
    {
        road.Rebuild();
    }
    *this = std::move(loaded);
    return true;
}

bool WorldDocument::Save() const
{
    if (path.empty())
    {
        return SDL_SetError("Open a world before saving it");
    }
    if (otherSections.size() >= std::numeric_limits<Uint32>::max())
    {
        return SDL_SetError("Too many sections to save in '%s'", path.c_str());
    }

    std::ostringstream roadData;
    roadData.imbue(std::locale::classic());
    roadData << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const Road& road : roads)
    {
        RoadIO::WriteRoad(roadData, road);
        if (!roadData || road.points.size() < 2)
        {
            return SDL_SetError("Cannot save invalid road in '%s'", path.c_str());
        }
    }

    SDL_Time now;
    if (!SDL_GetCurrentTime(&now))
    {
        return false;
    }
    const std::string temporary = path + "." + std::to_string(now) + ".tmp";
    SDL_IOStream* output = SDL_IOFromFile(temporary.c_str(), "wbx");
    if (!output)
    {
        return false;
    }
    const Uint32 count = static_cast<Uint32>(otherSections.size()) + 1;
    bool wroteAll = SDL_WriteIO(output, worldMagic.data(), worldMagic.size()) == worldMagic.size() &&
                    SDL_WriteU32LE(output, worldVersion) && SDL_WriteU32LE(output, count) &&
                    WriteSection(output, roadTag, RoadIO::currentVersion, std::move(roadData).str());
    for (const Section& section : otherSections)
    {
        if (!wroteAll)
        {
            break;
        }
        wroteAll = WriteSection(output, section.tag, section.version, section.payload);
    }
    if (!SDL_CloseIO(output) || !wroteAll || !SDL_RenamePath(temporary.c_str(), path.c_str()))
    {
        const std::string reason = SDL_GetError();
        SDL_RemovePath(temporary.c_str());
        return SDL_SetError("Cannot save '%s': %s", path.c_str(), reason.c_str());
    }
    return true;
}
