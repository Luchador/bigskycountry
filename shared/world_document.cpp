#include "world_document.hpp"
#include "material.hpp"
#include "road_io.hpp"
#include "object_io.hpp"
#include "building.hpp"
#include "character.hpp"
#include "prop.hpp"
#include "vehicle.hpp"

#include <SDL3/SDL.h>
#include <filesystem>
#include <cmath>
#include <iomanip>
#include <set>
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
    constexpr Tag intersectionTag = {'I', 'N', 'T', 'S'};
    constexpr Tag roadTag = {'R', 'O', 'A', 'D'};
    constexpr Tag meshTag = {'M', 'E', 'S', 'H'};
    constexpr Uint32 meshVersion = 1;
    constexpr Tag terrainTag = {'T', 'E', 'R', 'R'};
    constexpr Uint32 terrainVersion = 2;
    constexpr Tag outlinerTag = {'O', 'U', 'T', 'L'};
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

    bool ReadTerrain(std::string payload, Uint32 version, WorldDocument& world)
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
            std::string material;
            if (version >= 2 && (!(in >> std::quoted(material)) || (!material.empty() && !Material::ValidAssetPath(material))))
            {
                return SDL_SetError("Invalid terrain material path in TERR");
            }
            TerrainChunk* chunk = world.PlaceTerrain(path, t);
            if (!chunk) return false;
            chunk->material = std::move(material);
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

std::vector<WorldDocument::ConnectedEnd> WorldDocument::ConnectedEnds(int intersectionId) const
{
    std::vector<ConnectedEnd> ends;
    for (std::size_t i = 0; i < roads.size(); ++i)
    {
        const auto& points = roads[i].points;
        if (points.size() < 2) continue;
        for (bool first : {true, false})
        {
            const auto& anchor = (first ? points.front() : points.back()).anchor;
            if (anchor && anchor->intersectionId == intersectionId) ends.push_back({i, first, *anchor});
        }
    }
    return ends;
}

void WorldDocument::UpdateIntersections()
{
    if (intersections.empty() && junctions.empty()) return;
    constexpr int sides = IntersectionAnchor::sideCount;
    struct End { std::size_t road; int end; };
    std::map<int, std::array<std::optional<End>, sides>> incident;
    for (std::size_t i = 0; i < roads.size(); ++i)
    {
        const auto& points = roads[i].points;
        if (points.size() < 2) continue;
        for (int end = 0; end < 2; ++end)
        {
            const auto& point = end == 0 ? points.front() : points.back();
            if (point.anchor && point.anchor->Valid() && intersections.contains(point.anchor->intersectionId))
                incident[point.anchor->intersectionId][point.anchor->side] = End{i, end};
        }
    }
    std::vector<std::array<std::optional<RoadMouth>, 2>> mouths(roads.size());
    std::set<int> active;
    for (const auto& [id, ends] : incident)
    {
        std::set<std::size_t> distinct;
        const auto& item = intersections.at(id);
        IntersectionLayout layout{item.position, item.cornerRadius};
        for (int i = 0; i < sides; ++i)
        {
            if (!ends[i]) continue;
            const auto [road, end] = *ends[i];
            distinct.insert(road);
            const auto& s = roads[road].style;
            layout.arms[i] = IntersectionApproach{end == 0 ? s.LeftWidth() : s.RightWidth(),
                                                   end == 0 ? s.RightWidth() : s.LeftWidth(), s.edge};
        }
        if (distinct.size() < 3) continue;
        active.insert(id);
        auto& surface = junctions[id];
        if (layout != surface.layout) surface.Build(layout);
        for (int i = 0; i < sides; ++i)
        {
            if (!ends[i]) continue;
            const auto [road, end] = *ends[i];
            const Vector3 dir = IntersectionAnchor::directions[i];
            mouths[road][end] = RoadMouth{item.AnchorPosition(i), item.position + dir * surface.setbacks[i], dir};
        }
    }
    std::erase_if(junctions, [&](const auto& entry) { return !active.contains(entry.first); });
    for (std::size_t i = 0; i < roads.size(); ++i) roads[i].SetMouths(mouths[i]);
}

bool WorldDocument::ValidConnections() const
{
    std::set<IntersectionAnchor> occupied;
    for (const Road& road : roads)
    {
        if (road.points.size() < 2) return SDL_SetError("Road needs two endpoints");
        for (const RoadPoint* point : {&road.points.front(), &road.points.back()})
        {
            if (!point->anchor) continue;
            const auto ref = *point->anchor;
            const auto it = intersections.find(ref.intersectionId);
            if (!ref.Valid() || it == intersections.end() || !occupied.insert(ref).second ||
                point->position != it->second.AnchorPosition(ref.side))
                return SDL_SetError("Invalid road connection to intersection %d, side %d", ref.intersectionId, ref.side);
        }
    }
    return true;
}

int WorldDocument::CreateIntersection(Vector3 position)
{
    if (!std::isfinite(Math::Length(position)) ||
        (!intersections.empty() && intersections.rbegin()->first == std::numeric_limits<int>::max()))
    {
        SDL_SetError("Cannot place intersection");
        return 0;
    }
    const int id = intersections.empty() ? 1 : intersections.rbegin()->first + 1;
    intersections.emplace(id, Intersection{position});
    return id;
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

std::unique_ptr<BSCObject> WorldDocument::TakeObject(int index)
{
    if (index < 0 || static_cast<std::size_t>(index) >= objects.size())
    {
        SDL_SetError("Invalid object index");
        return {};
    }
    hierarchy.ReplaceRange(SceneKind::Object, index, 1, 0);
    auto object = std::move(objects[index]);
    objects.erase(objects.begin() + index);
    return object;
}

bool WorldDocument::InsertObject(int index, std::unique_ptr<BSCObject>&& object, int folder)
{
    if (!object || index < 0 || static_cast<std::size_t>(index) > objects.size())
    {
        return SDL_SetError("Invalid object insertion");
    }
    objects.reserve(objects.size() + 1);
    hierarchy.ReplaceRange(SceneKind::Object, index, 0, 1, folder);
    objects.insert(objects.begin() + index, std::move(object));
    return true;
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

bool WorldDocument::RemoveTerrain(std::size_t index)
{
    if (index >= terrain.size())
    {
        return SDL_SetError("Invalid terrain chunk index");
    }
    hierarchy.ReplaceRange(SceneKind::Terrain, static_cast<int>(index), 1, 0);
    terrain.erase(terrain.begin() + index);
    return true;
}

bool WorldDocument::Contains(SceneKey key) const
{
    const auto index = static_cast<std::size_t>(key.index);
    switch (key.kind)
    {
    case SceneKind::World: return key.index == 0;
    case SceneKind::Folder: return hierarchy.Folders().contains(key.index);
    case SceneKind::Intersection: return intersections.contains(key.index);
    case SceneKind::Road: return index < roads.size();
    case SceneKind::Terrain: return index < terrain.size();
    case SceneKind::Object: return index < objects.size();
    case SceneKind::Mesh: return index < meshes.size();
    }
    return false;
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
    WorldDocument loaded(models.Location());
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

    bool sawIntersections = false;
    bool sawRoads = false;
    bool sawMeshes = false;
    bool sawObjects = false;
    bool sawTerrain = false;
    bool sawOutliner = false;
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
        if (section.tag == terrainTag && (sawTerrain || section.version < 1 || section.version > terrainVersion))
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

        if (section.tag == outlinerTag)
        {
            std::istringstream data(std::move(section.payload));
            data.imbue(std::locale::classic());
            if (sawOutliner || section.version != 1 || !loaded.hierarchy.Read(data))
            {
                return SDL_SetError("Invalid or unsupported OUTL section in '%s'", loaded.path.c_str());
            }
            sawOutliner = true;
        }
        else if (section.tag == intersectionTag)
        {
            if (sawIntersections || section.version != 1) return SDL_SetError("Duplicate or unsupported INTS section");
            sawIntersections = true;
            std::istringstream data(std::move(section.payload));
            data.imbue(std::locale::classic());
            while (!(data >> std::ws).eof())
            {
                int id;
                Intersection item;
                auto& p = item.position;
                if (!(data >> id >> p.x >> p.y >> p.z >> item.cornerRadius >> std::quoted(item.material)) ||
                    id <= 0 || !std::isfinite(Math::Length(p)) || !std::isfinite(item.cornerRadius) || item.cornerRadius < 0 ||
                    !Material::ValidAssetPath(item.material) || !loaded.intersections.emplace(id, std::move(item)).second)
                    return SDL_SetError("Invalid intersection in INTS");
            }
        }
        else if (section.tag == roadTag)
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
            if (!ReadTerrain(std::move(section.payload), section.version, loaded))
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
    for (const auto& [key, parent] : loaded.hierarchy.Members())
    {
        if (!loaded.Contains(key)) return SDL_SetError("Missing OUTL entity in '%s'", loaded.path.c_str());
    }
    if (!loaded.ValidConnections()) return false;
    for (Road& road : loaded.roads)
    {
        road.Rebuild();
    }
    loaded.UpdateIntersections();
    *this = std::move(loaded);
    return true;
}

bool WorldDocument::Save() const
{
    if (path.empty())
    {
        return SDL_SetError("Open a world before saving it");
    }
    if (!ValidConnections()) return false;
    for (const auto& [key, parent] : hierarchy.Members())
    {
        if (!Contains(key)) return SDL_SetError("Cannot save an outliner entry for a missing entity");
    }
    const bool hasOutliner = !hierarchy.Folders().empty();
    std::ostringstream outlinerData;
    outlinerData.imbue(std::locale::classic());
    if (hasOutliner && !hierarchy.Write(outlinerData)) return SDL_SetError("Could not serialize the outliner");
    const Uint32 knownSections = 1 + !meshes.empty() + !objects.empty() + !terrain.empty() + !intersections.empty() + hasOutliner;
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

    std::ostringstream junctionData;
    junctionData.imbue(std::locale::classic());
    junctionData << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& [id, item] : intersections)
    {
        const auto p = item.position;
        if (id <= 0 || !std::isfinite(Math::Length(p)) || !std::isfinite(item.cornerRadius) || item.cornerRadius < 0 ||
            !Material::ValidAssetPath(item.material) ||
            !(junctionData << id << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' ' << item.cornerRadius
                          << ' ' << std::quoted(item.material) << '\n'))
            return SDL_SetError("Cannot save invalid intersection");
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
        if (!mesh || mesh->assetPath.empty() || (!chunk->material.empty() && !Material::ValidAssetPath(chunk->material)) ||
            !(terrainData << std::quoted(mesh->assetPath) << ' ' << t.position.x << ' ' << t.position.y << ' ' << t.position.z
                          << ' ' << t.rotation.x << ' ' << t.rotation.y << ' ' << t.rotation.z << ' ' << std::quoted(chunk->material) << '\n'))
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
                    (intersections.empty() || WriteSection(output, intersectionTag, 1, std::move(junctionData).str())) &&
                    (meshes.empty() || WriteSection(output, meshTag, meshVersion, std::move(meshData).str())) &&
                    (objects.empty() || WriteSection(output, objectTag, ObjectIO::currentVersion, std::move(objectData).str())) &&
                    (terrain.empty() || WriteSection(output, terrainTag, terrainVersion, std::move(terrainData).str())) &&
                    (!hasOutliner || WriteSection(output, outlinerTag, 1, std::move(outlinerData).str()));
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
