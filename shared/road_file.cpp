#include "road.hpp"

#include <SDL3/SDL.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>

bool RoadMap::Open(const char* documentPath)
{
    std::string filename;
    if (documentPath)
    {
        filename = documentPath;
    }
    else
    {
        char* directory = SDL_GetPrefPath("BigSkyCountry", "BigSkyCountry");
        if (!directory)
        {
            return false;
        }
        filename = std::string(directory) + "world.roads";
        SDL_free(directory);
    }

    const auto filePath = std::filesystem::u8path(filename);
    std::error_code error;
    if (!std::filesystem::exists(filePath, error) && !error)
    {
        path = std::move(filename);
        roads.clear();
        return true;
    }

    std::ifstream file(filePath);
    file.imbue(std::locale::classic());
    std::string token;
    int version;
    if (!(file >> token >> version) || token != "BSC_ROADS" || (version != 1 && version != 2))
    {
        return SDL_SetError("Cannot read road document: %s", filename.c_str());
    }

    std::vector<Road> loaded;
    while (file >> token)
    {
        Road road;
        auto& style = road.style;
        int laneCount;
        if (token != "road" ||
            !(file >> style.curved >> laneCount >> style.leftShoulder >> style.rightShoulder) || laneCount < 1)
        {
            return SDL_SetError("Invalid road settings: %s", filename.c_str());
        }
        style.SetLaneCount(laneCount);
        if (laneCount > RoadStyle::maxLanes)
        {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Road in '%s' uses %d lanes. Capped at %d.",
                        filename.c_str(), laneCount, RoadStyle::maxLanes);
        }
        if (version == 2)
        {
            int layout;
            if (!(file >> layout) || layout < static_cast<int>(ThreeLaneLayout::TwoNorthbound) ||
                layout > static_cast<int>(ThreeLaneLayout::CenterTurn))
            {
                return SDL_SetError("Invalid three-lane layout: %s", filename.c_str());
            }
            style.threeLaneLayout = static_cast<ThreeLaneLayout>(layout);
        }
        while (file >> token && token == "point")
        {
            RoadPoint point{};
            bool manual;
            double angle;
            if (!(file >> point.position.x >> point.position.z >> manual >> angle) ||
                !std::isfinite(point.position.x) || !std::isfinite(point.position.z) || !std::isfinite(angle))
            {
                return SDL_SetError("Invalid road control point: %s", filename.c_str());
            }
            if (manual)
            {
                point.heading = angle;
            }
            road.points.push_back(point);
        }
        if (!file || token != "end" || road.points.size() < 2)
        {
            return SDL_SetError("Incomplete road: %s", filename.c_str());
        }
        road.Rebuild();
        loaded.push_back(std::move(road));
    }
    if (file.bad())
    {
        return SDL_SetError("Error reading road document: %s", filename.c_str());
    }
    path = std::move(filename);
    roads = std::move(loaded);
    return true;
}

bool RoadMap::Save() const
{
    std::ostringstream file;
    file.imbue(std::locale::classic());
    file << "BSC_ROADS 2\n" << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const Road& road : roads)
    {
        const auto& style = road.style;
        file << "road " << style.curved << ' ' << style.GetLaneCount() << ' ' << style.leftShoulder << ' '
             << style.rightShoulder << ' ' << static_cast<int>(style.threeLaneLayout) << '\n';
        for (const RoadPoint& point : road.points)
        {
            file << "point " << point.position.x << ' ' << point.position.z << ' ' << point.heading.has_value()
                 << ' ' << point.heading.value_or(0.0) << '\n';
        }
        file << "end\n";
    }
    const std::string data = file.str();
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
    const bool wroteAll = SDL_WriteIO(output, data.data(), data.size()) == data.size();
    if (!SDL_CloseIO(output) || !wroteAll || !SDL_RenamePath(temporary.c_str(), path.c_str()))
    {
        const std::string reason = SDL_GetError();
        SDL_RemovePath(temporary.c_str());
        return SDL_SetError("Cannot save '%s': %s", path.c_str(), reason.c_str());
    }
    return true;
}
