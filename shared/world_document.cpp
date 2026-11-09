#include "world_document.hpp"
#include "road_io.hpp"
#include "object_io.hpp"
#include "building.hpp"
#include "character.hpp"
#include "prop.hpp"
#include "vehicle.hpp"

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
    constexpr Tag meshTag = {'M', 'E', 'S', 'H'};
    constexpr Uint32 meshVersion = 1;
    constexpr Tag terrainTag = {'T', 'E', 'R', 'R'};
    constexpr Uint32 terrainVersion = 1;
    constexpr Tag objectTag = {'O', 'B', 'J', 'S'};

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

    bool ReadMeshes(std::string payload, WorldDocument& world)
    {
        std::istringstream in(std::move(payload));
        in.imbue(std::locale::classic());
        in >> std::ws;
        while (!in.eof())
        {
            std::string path;
            MeshInstance instance;
            auto& t = instance.transform;
            if (!(in >> std::quoted(path) >> t.origin.x >> t.origin.y >> t.origin.z
                     >> t.x.x >> t.x.y >> t.x.z >> t.y.x >> t.y.y >> t.y.z >> t.z.x >> t.z.y >> t.z.z) || !t.Valid())
            {
                return SDL_SetError("Invalid mesh instance transform or model path in MESH");
            }
            instance.mesh = world.models.Load(path);
            if (!instance.mesh)
            {
                return false;
            }
            world.meshes.push_back(std::move(instance));
            in >> std::ws;
        }
        return true;
    }

    bool WriteMeshes(std::ostream& out, const std::vector<MeshInstance>& meshes)
    {
        out.imbue(std::locale::classic());
        out << std::setprecision(std::numeric_limits<double>::max_digits10);
        for (const auto& instance : meshes)
        {
            const auto& t = instance.transform;
            if (!instance.mesh || instance.mesh->assetPath.empty() || !t.Valid())
            {
                return SDL_SetError("Cannot save an invalid mesh instance");
            }
            out << std::quoted(instance.mesh->assetPath) << ' ' << t.origin.x << ' ' << t.origin.y << ' ' << t.origin.z
                << ' ' << t.x.x << ' ' << t.x.y << ' ' << t.x.z << ' ' << t.y.x << ' ' << t.y.y << ' ' << t.y.z
                << ' ' << t.z.x << ' ' << t.z.y << ' ' << t.z.z << '\n';
        }
        return out.good() || SDL_SetError("Could not serialize mesh instances");
    }

    bool ReadTerrain(std::string payload, WorldDocument& world)
    {
        std::istringstream in(std::move(payload));
        in.imbue(std::locale::classic());
        in >> std::ws;
        while (!in.eof())
        {
            std::string path;
            ObjectTransform t;
            if (!(in >> std::quoted(path) >> t.position.x >> t.position.y >> t.position.z
                     >> t.rotation.x >> t.rotation.y >> t.rotation.z))
            {
                return SDL_SetError("Invalid terrain chunk in TERR");
            }
            if (!world.PlaceTerrain(path, t))
            {
                return false;
            }
            in >> std::ws;
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

BSCObject* WorldDocument::PlaceObject(ObjectType type, const std::string& modelPath, const ObjectTransform& transform)
{
    std::unique_ptr<BSCObject> object;
    switch (type)
    {
    case ObjectType::Object:
        object = std::make_unique<BSCObject>();
        break;
    case ObjectType::Building:
        object = std::make_unique<Building>();
        break;
    case ObjectType::Vehicle:
        object = std::make_unique<Vehicle>();
        break;
    case ObjectType::Prop:
        object = std::make_unique<Prop>();
        break;
    case ObjectType::Character:
        object = std::make_unique<Character>();
        break;
    default:
        SDL_SetError("Unknown world object type %u", static_cast<unsigned>(type));
        return nullptr;
    }

    if (!object->SetTransform(transform))
    {
        return nullptr;
    }
    object->SetMesh(models.Load(modelPath));
    if (!object->IsLoaded())
    {
        return nullptr;
    }
    return objects.emplace_back(std::move(object)).get();
}

TerrainChunk* WorldDocument::PlaceTerrain(const std::string& modelPath, const ObjectTransform& transform)
{
    auto chunk = std::make_unique<TerrainChunk>();
    if (!chunk->SetTransform(transform))
    {
        return nullptr;
    }
    auto model = models.Load(modelPath);
    if (!model)
    {
        return nullptr;
    }
    chunk->SetMesh(std::move(model));
    return terrain.emplace_back(std::move(chunk)).get();
}

void WorldDocument::TickObjects(float deltaSeconds)
{
    // Objects placed during a tick begin updating on the following frame.
    const std::size_t count = objects.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        objects[i]->Tick(deltaSeconds);
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
    bool sawMeshes = false;
    bool sawObjects = false;
    bool sawTerrain = false;
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
        if (section.tag == meshTag && (sawMeshes || section.version != meshVersion))
        {
            return SDL_SetError("Duplicate or unsupported MESH section in '%s'", loaded.path.c_str());
        }
        if (section.tag == objectTag && (sawObjects || section.version != ObjectIO::currentVersion))
        {
            return SDL_SetError("Duplicate or unsupported OBJS section in '%s'", loaded.path.c_str());
        }
        if (section.tag == terrainTag && (sawTerrain || section.version != terrainVersion))
        {
            return SDL_SetError("Duplicate or unsupported TERR section in '%s'", loaded.path.c_str());
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
        else if (section.tag == meshTag)
        {
            sawMeshes = true;
            if (!ReadMeshes(std::move(section.payload), loaded))
            {
                return false;
            }
        }
        else if (section.tag == terrainTag)
        {
            sawTerrain = true;
            if (!ReadTerrain(std::move(section.payload), loaded))
            {
                return false;
            }
        }
        else if (section.tag == objectTag)
        {
            sawObjects = true;
            if (!ObjectIO::Read(std::move(section.payload), loaded))
            {
                return false;
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
    const Uint32 knownSections = 1 + !meshes.empty() + !objects.empty() + !terrain.empty();
    if (otherSections.size() > std::numeric_limits<Uint32>::max() - knownSections)
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

    std::ostringstream meshData;
    if (!WriteMeshes(meshData, meshes))
    {
        return false;
    }
    std::ostringstream objectData;
    if (!ObjectIO::Write(objectData, *this))
    {
        return false;
    }
    std::ostringstream terrainData;
    terrainData.imbue(std::locale::classic());
    terrainData << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& chunk : terrain)
    {
        const auto& mesh = chunk->Instance().mesh;
        const auto& t = chunk->GetTransform();
        if (!mesh || mesh->assetPath.empty() ||
            !(terrainData << std::quoted(mesh->assetPath) << ' ' << t.position.x << ' ' << t.position.y << ' ' << t.position.z
                          << ' ' << t.rotation.x << ' ' << t.rotation.y << ' ' << t.rotation.z << '\n'))
        {
            return SDL_SetError("Cannot save terrain chunk");
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
    const Uint32 count = static_cast<Uint32>(otherSections.size()) + knownSections;
    bool wroteAll = SDL_WriteIO(output, worldMagic.data(), worldMagic.size()) == worldMagic.size() &&
                    SDL_WriteU32LE(output, worldVersion) && SDL_WriteU32LE(output, count) &&
                    WriteSection(output, roadTag, RoadIO::currentVersion, std::move(roadData).str()) &&
                    (meshes.empty() || WriteSection(output, meshTag, meshVersion, std::move(meshData).str())) &&
                    (objects.empty() || WriteSection(output, objectTag, ObjectIO::currentVersion, std::move(objectData).str())) &&
                    (terrain.empty() || WriteSection(output, terrainTag, terrainVersion, std::move(terrainData).str()));
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
