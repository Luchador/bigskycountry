#include "world_document.hpp"
#include "spawn_point.hpp"
#include "curved_road.hpp"
#include "material.hpp"
#include "road_io.hpp"
#include "object_io.hpp"
#include "building.hpp"
#include "character.hpp"
#include "prop.hpp"
#include "marker.hpp"
#include "vehicle.hpp"

#include <SDL3/SDL.h>
#include <algorithm>
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
    constexpr Uint32 intersectionVersion = 9;
    constexpr Tag intersectionTag = {'I', 'N', 'T', 'S'};
    constexpr Tag roadTag = {'R', 'O', 'A', 'D'};
    constexpr Tag meshTag = {'M', 'E', 'S', 'H'};
    constexpr Uint32 meshVersion = 1;
    constexpr Tag terrainTag = {'T', 'E', 'R', 'R'};
    constexpr Uint32 terrainVersion = 10;
    constexpr Tag outlinerTag = {'O', 'U', 'T', 'L'};
    constexpr Tag identityTag = {'E', 'I', 'D', 'S'};
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

    template <class Id> bool ReadIds(std::istream& in, std::set<Id>& ids)
    {
        int count;
        if (!(in >> count) || count < 0) return false;
        for (int i = 0; i < count; ++i)
        {
            Id id;
            if (!(in >> std::ws) || in.peek() == '-') return false;
            if (!(in >> id) || id <= 0 || !ids.insert(id).second) return false;
        }
        return true;
    }

    bool ReadTerrain(std::string payload, Uint32 version, WorldDocument& world)
    {
        std::istringstream in(std::move(payload));
        in.imbue(std::locale::classic());
        in >> std::ws;
        while (!in.eof())
        {
            std::string path;
            std::string kind = "mesh";
            if (version >= 3 && !(in >> kind)) return SDL_SetError("Missing terrain kind in TERR");
            if (kind == "fill")
            {
                TerrainBoundary boundary;
                boundary.filterIncidentalPockets = false;
                std::string material;
                if (!(in >> std::quoted(material)) || (!material.empty() && !Material::ValidAssetPath(material)) ||
                    !ReadIds(in, boundary.roads) || !ReadIds(in, boundary.intersections) ||
                    (version >= 7 && !ReadIds(in, boundary.waterBodies)) ||
                    (version >= 9 && !(in >> boundary.filterIncidentalPockets)) || boundary.Empty())
                    return SDL_SetError("Invalid terrain boundary in TERR");
                auto* chunk = world.PlaceTerrainFill(std::move(boundary), true);
                if (!chunk) return false;
                chunk->material = std::move(material);
                in >> std::ws;
                continue;
            }
            if ((kind == "lake" && version >= 5) || (kind == "river" && version >= 6))
            {
                ObjectTransform t;
                WaterDefinition water = kind == "river" ? WaterDefinition{RiverDefinition{}} : WaterDefinition{LakeDefinition{}};
                std::string material;
                if (!(in >> t.position.x >> t.position.y >> t.position.z >> t.rotation.y >> std::quoted(material)) ||
                    (!material.empty() && !Material::ValidAssetPath(material)) || !std::visit([&](auto& source) {
                        if constexpr (std::is_same_v<std::decay_t<decltype(source)>, RiverDefinition>) return source.Read(in, version >= 8, version >= 10);
                        else return source.Read(in);
                    }, water))
                    return SDL_SetError("Invalid water body in TERR");
                auto* chunk = world.PlaceWater(water, t);
                if (!chunk) return false;
                chunk->material = std::move(material);
                in >> std::ws;
                continue;
            }
            if (kind != "mesh") return SDL_SetError("Unknown terrain kind in TERR");
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

void WorldDocument::UpdateIntersections(const std::set<int>& changedRoads)
{
    if (intersections.empty() && junctions.empty())
    {
        for (std::size_t i = 0; i < roads.size(); ++i)
        {
            const bool skewed = roads[i].skewEnds[0] || roads[i].skewEnds[1];
            roads[i].skewEnds = {};
            if (skewed || changedRoads.contains(static_cast<int>(i))) roads[i].Rebuild();
        }
        RefreshRampConnections(true);
        RefreshRailroadCrossings(true);
        terrainRevision = 0; // Also covers removal of the last road.
        RefreshTerrainFills();
        return;
    }
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
    for (const auto& [id, item] : intersections)
        if (item.shape == IntersectionShape::TownSquare) incident.try_emplace(id);
    std::vector<std::array<std::optional<RoadMouth>, 2>> mouths(roads.size());
    std::vector<std::array<bool, 2>> skewEnds(roads.size());
    std::set<int> active;
    for (const auto& [id, ends] : incident)
    {
        int connectedArms = 0;
        const auto& item = intersections.at(id);
        IntersectionLayout layout{item.position, item.cornerRadius};
        layout.paint = item.paint;
        layout.yaw = item.yaw;
        layout.skew = item.skew;
        layout.alignedApproaches = item.alignedApproaches;
        layout.shape = item.shape;
        layout.size = item.size;
        layout.island = item.island;
        layout.townIsland = item.townIsland;
        layout.islandSize = item.islandSize;
        for (int i = 0; i < sides; ++i)
        {
            if (!ends[i] || !item.HasAnchor(i)) continue;
            const auto [road, end] = *ends[i];
            skewEnds[road][end] = item.shape == IntersectionShape::TownSquare || item.skew[i] != 0 || item.AlignsOpposite(i);
            ++connectedArms; // A loop occupies a separate anchor at each end.
            layout.arms[i] = IntersectionApproach::FromRoad(roads[road], end == 0, item.stopLines.test(i));
        }
        const bool through = item.pavedThrough && ((ends[0] && ends[2]) || (ends[1] && ends[3]));
        if (connectedArms < 3 && !through && item.shape != IntersectionShape::TownSquare) continue;
        active.insert(id);
        auto& surface = junctions[id];
        if (layout != surface.layout) surface.Build(layout);
        for (int i = 0; i < sides; ++i)
        {
            if (!ends[i] || !item.HasAnchor(i)) continue;
            const auto [road, end] = *ends[i];
            mouths[road][end] = surface.Mouth(i);
        }
    }
    std::erase_if(junctions, [&](const auto& entry) { return !active.contains(entry.first); });
    for (std::size_t i = 0; i < roads.size(); ++i)
    {
        const bool skewChanged = roads[i].skewEnds != skewEnds[i];
        roads[i].skewEnds = skewEnds[i];
        roads[i].SetMouths(mouths[i], skewChanged || changedRoads.contains(static_cast<int>(i)));
    }
    RefreshRampConnections(true);
    RefreshRailroadCrossings(true);
    terrainRevision = 0; // Attachment changes can open a junction without rebuilding a road.
    RefreshTerrainFills();
}

bool WorldDocument::ValidConnections() const
{
    std::set<int> sockets, targets;
    std::set<int> stations;
    std::set<std::pair<int, int>> rampEnds;
    for (const auto& road : roads)
        for (const auto& p : road.points)
            if (p.rampStation && !stations.insert(p.rampStation).second) return SDL_SetError("Duplicate highway ramp station");
    std::map<int, const Road*> stationOwners, socketOwners;
    for (const auto& road : roads) for (const auto& p : road.points)
    {
        if (p.rampStation) stationOwners[p.rampStation] = &road;
        if (p.connectorSocket) socketOwners[p.connectorSocket] = &road;
    }
    std::set<int> switchStations;
    std::map<int, std::set<int>> switchParents;
    std::set<IntersectionAnchor> occupied;
    for (const Road& road : roads)
    {
        if (road.points.size() < 2) return SDL_SetError("Road needs two endpoints");
        if (road.style.railroad && std::ranges::any_of(road.points, [](const RoadPoint& p) { return p.anchor || p.ramp; }))
            return SDL_SetError("Railroad control points cannot have highway or junction links");
        for (const auto& p : road.points)
        {
            if (p.curveJoin)
            {
                const auto source = stationOwners.find(p.curveJoin->station);
                if (!p.curveJoin->Valid() || road.style.railroad || p.anchor || p.ramp || p.railSwitch || p.connectorSocket || p.connectorTarget ||
                    (&p != &road.points.front() && &p != &road.points.back()) || source == stationOwners.end() ||
                    source->second == &road || !CanHostCurvedConnector(*source->second))
                    return SDL_SetError("Invalid curved road connection");
            }
            if (!p.railSwitch) continue;
            const auto source = stationOwners.find(p.railSwitch->station);
            if (!road.style.railroad || !p.railSwitch->Valid() || p.anchor || p.ramp || p.connectorSocket || p.connectorTarget ||
                (&p != &road.points.front() && &p != &road.points.back()) || source == stationOwners.end() ||
                !source->second->style.railroad || source->second == &road || !switchStations.insert(p.railSwitch->station).second)
                return SDL_SetError("Invalid railroad switch connection");
            switchParents[road.id].insert(source->second->id);
        }
        for (const RoadPoint* point : {&road.points.front(), &road.points.back()})
        {
            if (point->connectorSocket && !sockets.insert(point->connectorSocket).second)
                return SDL_SetError("Duplicate road connector socket");
            if (point->connectorTarget && !targets.insert(point->connectorTarget).second)
                return SDL_SetError("Duplicate road connector attachment");
            if (point->connectorTarget && socketOwners.contains(point->connectorTarget) &&
                road.style.railroad.has_value() != socketOwners.at(point->connectorTarget)->style.railroad.has_value())
                return SDL_SetError("Cannot connect a road endpoint directly to a railroad");
            if (point->ramp)
            {
                const auto& link = *point->ramp;
                if (!link.Valid() || !road.style.oneWay || point->anchor || !stations.contains(link.station) ||
                    !rampEnds.emplace(link.station, link.side).second) return SDL_SetError("Invalid highway ramp connection");
            }
            if (!point->anchor) continue;
            const auto ref = *point->anchor;
            const auto it = intersections.find(ref.intersectionId);
            if (!ref.Valid() || it == intersections.end() || !it->second.HasAnchor(ref.side) || !occupied.insert(ref).second ||
                !(Math::Length(point->position - it->second.AnchorPosition(ref.side)) <= 1e-8))
                return SDL_SetError("Invalid road connection to intersection %d, side %d", ref.intersectionId, ref.side);
        }
    }
    for (const auto& [id, parents] : switchParents)
    {
        auto pending = parents;
        std::set<int> visited;
        while (!pending.empty())
        {
            const int next = *pending.begin(); pending.erase(pending.begin());
            if (next == id) return SDL_SetError("Cyclic railroad switch connection");
            if (visited.insert(next).second && switchParents.contains(next))
                pending.insert(switchParents.at(next).begin(), switchParents.at(next).end());
        }
    }
    if (sockets != targets) return SDL_SetError("Missing road connector endpoint");
    return true;
}

int WorldDocument::CreateIntersection(Vector3 position)
{
    int last = std::max(lastIntersectionId, intersections.empty() ? 0 : intersections.rbegin()->first);
    for (const auto& chunk : terrain)
        if (chunk->boundary && !chunk->boundary->intersections.empty())
            last = std::max(last, *chunk->boundary->intersections.rbegin());
    if (!std::isfinite(Math::Length(position)) || last == std::numeric_limits<int>::max())
    {
        SDL_SetError("Cannot place intersection");
        return 0;
    }
    const int id = lastIntersectionId = last + 1;
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
    case ObjectType::Marker:
        if (modelPath != Marker::modelPath)
        {
            SDL_SetError("Invalid marker model path");
            return nullptr;
        }
        object = std::make_unique<Marker>();
        break;
    case ObjectType::SpawnPoint:
        if (modelPath != SpawnPoint::modelPath || SpawnPointIndex() >= 0)
        {
            SDL_SetError("A world can have only one spawn point using editor/start.glb");
            return nullptr;
        }
        object = std::make_unique<SpawnPoint>();
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
    if (!(object->persistentId = AllocatePlacementId())) return nullptr;
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
    if (object->Type() == ObjectType::SpawnPoint && SpawnPointIndex() >= 0)
        return SDL_SetError("A world can have only one spawn point");
    if (!object->persistentId && !(object->persistentId = AllocatePlacementId())) return false;
    for (const auto& existing : objects)
        if (existing->persistentId == object->persistentId) return SDL_SetError("Duplicate object identity");
    lastPlacementId = std::max(lastPlacementId, object->persistentId);
    objects.reserve(objects.size() + 1);
    hierarchy.ReplaceRange(SceneKind::Object, index, 0, 1, folder);
    objects.insert(objects.begin() + index, std::move(object));
    return true;
}

TerrainChunk* WorldDocument::PlaceLake(const LakeDefinition& lake, const ObjectTransform& transform)
{
    return PlaceWater(lake, transform);
}

TerrainChunk* WorldDocument::PlaceWater(const WaterDefinition& water, const ObjectTransform& transform)
{
    auto chunk = std::make_unique<TerrainChunk>(geometryCache);
    if (!chunk->SetWater(water) || !chunk->SetTransform(transform)) return nullptr;
    if (!(chunk->persistentId = AllocatePlacementId())) return nullptr;
    return terrain.emplace_back(std::move(chunk)).get();
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
    if (!(chunk->persistentId = AllocatePlacementId())) return nullptr;
    return terrain.emplace_back(std::move(chunk)).get();
}

bool WorldDocument::RemoveTerrain(std::size_t index)
{
    if (index >= terrain.size())
    {
        return SDL_SetError("Invalid terrain chunk index");
    }
    hierarchy.ReplaceRange(SceneKind::Terrain, static_cast<int>(index), 1, 0);
    if (terrain[index]->HasWater()) { terrainWaterRevision = 0; riverRevision = 0; }
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

bool WorldDocument::OpenPath(const char* documentPath)
{
    // Only a fresh document can become a path-only runtime catalog.
    if (!path.empty() || !roads.empty() || !objects.empty() || !terrain.empty() || !meshes.empty())
        return SDL_SetError("Runtime streaming requires an empty world document");
    if (documentPath) path = documentPath;
    else
    {
        char* directory = SDL_GetPrefPath("BigSkyCountry", "BigSkyCountry");
        if (!directory) return false;
        path = std::string(directory) + "world.wld";
        SDL_free(directory);
    }
    runtimePathOnly = true;
    return true;
}

bool WorldDocument::Open(const char* documentPath)
{
    WorldDocument loaded(models.Location(), geometryCache);
    if (!loaded.OpenPath(documentPath)) return false;
    loaded.runtimePathOnly = false;

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

    std::optional<std::string> identities;
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
        if (section.tag == objectTag && (sawObjects || section.version < 1 || section.version > ObjectIO::currentVersion))
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

        if (section.tag == identityTag)
        {
            if (identities || section.version != 1) return SDL_SetError("Duplicate or unsupported EIDS section");
            identities = std::move(section.payload);
        }
        else if (section.tag == outlinerTag)
        {
            std::istringstream data(std::move(section.payload));
            data.imbue(std::locale::classic());
            if (sawOutliner || !loaded.hierarchy.Read(data, section.version))
            {
                return SDL_SetError("Invalid or unsupported OUTL section in '%s'", loaded.path.c_str());
            }
            sawOutliner = true;
        }
        else if (section.tag == intersectionTag)
        {
            if (sawIntersections || section.version < 1 || section.version > intersectionVersion)
                return SDL_SetError("Duplicate or unsupported INTS section");
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
                    !Material::ValidAssetPath(item.material))
                    return SDL_SetError("Invalid intersection in INTS");
                if (section.version >= 2)
                {
                    unsigned int mask;
                    if (!(data >> mask) || mask >= (1u << IntersectionAnchor::sideCount))
                        return SDL_SetError("Invalid intersection stop lines in INTS");
                    item.stopLines = decltype(item.stopLines){mask};
                }
                if (section.version >= 3 && (!(data >> item.yaw) || !std::isfinite(item.yaw)))
                    return SDL_SetError("Invalid intersection heading in INTS");
                if (section.version >= 4 && !(data >> item.pavedThrough))
                    return SDL_SetError("Invalid through pavement flag in INTS");
                if (section.version >= 5)
                    for (int side = 0; side < (section.version >= 9 ? IntersectionAnchor::sideCount : IntersectionAnchor::cardinalCount); ++side)
                        if (!(data >> item.skew[side]) || !std::isfinite(item.skew[side]) || std::abs(item.skew[side]) > Intersection::maxSkew)
                            return SDL_SetError("Invalid intersection anchor skew in INTS");
                if (section.version >= 6)
                {
                    int paint;
                    if (!(data >> paint) || paint < 0 || paint > 2)
                        return SDL_SetError("Invalid intersection paint mode in INTS");
                    item.paint = static_cast<IntersectionPaint>(paint);
                }
                if (section.version >= 7)
                {
                    int shape;
                    if (!(data >> shape >> item.size >> item.island) || shape < 0 || shape > (section.version >= 9 ? 2 : 1) ||
                        !std::isfinite(item.size) || item.size < Intersection::minSize || item.size > Intersection::maxSize)
                        return SDL_SetError("Invalid intersection shape/size in INTS");
                    item.shape = static_cast<IntersectionShape>(shape);
                }
                if (section.version >= 8)
                {
                    unsigned mask;
                    if (!(data >> mask) || mask > 3) return SDL_SetError("Invalid intersection alignment mask in INTS");
                    item.alignedApproaches = decltype(item.alignedApproaches){mask};
                    if (!item.ValidAlignment()) return SDL_SetError("Invalid aligned intersection approaches in INTS");
                }
                if (section.version >= 9)
                {
                    int island;
                    if (!(data >> island >> item.islandSize) || island < 0 || island > 2)
                        return SDL_SetError("Invalid town square island in INTS");
                    item.townIsland = static_cast<TownIsland>(island);
                    if (!item.ValidTownSquare()) return SDL_SetError("Invalid town square dimensions in INTS");
                }
                if (!loaded.intersections.emplace(id, std::move(item)).second)
                    return SDL_SetError("Duplicate intersection ID in INTS");
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
            if (!ObjectIO::Read(std::move(section.payload), section.version, loaded))
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
    std::set<int> used;
    for (const Road& road : loaded.roads)
    {
        if (road.id && !used.insert(road.id).second) return SDL_SetError("Duplicate road ID in ROAD");
        loaded.lastRoadId = std::max(loaded.lastRoadId, road.id);
        for (const auto& crossing : road.crossings)
            loaded.lastRoadId = std::max(loaded.lastRoadId, crossing.railroadId);
    }
    for (Road& road : loaded.roads)
    {
        if (!road.id && !(road.id = loaded.AllocateRoadId())) return false;
        // Sine/cosine can differ by an ULP across platforms. Use the local anchor exactly.
        for (RoadPoint* end : {&road.points.front(), &road.points.back()})
            if (end->anchor) end->position = loaded.intersections.at(end->anchor->intersectionId).AnchorPosition(end->anchor->side);
        road.Rebuild();
    }
    if (identities && !loaded.ReadStableIds(*identities)) return false;
    if (!loaded.EnsureStableIds()) return false;
    if (!loaded.RefreshRiverConnections()) return false;
    loaded.UpdateIntersections();
    *this = std::move(loaded);
    return true;
}

bool WorldDocument::Save()
{
    if (runtimePathOnly) return SDL_SetError("Cannot save a streamed runtime world over authored data");
    if (path.empty())
    {
        return SDL_SetError("Open a world before saving it");
    }
    if (!ValidConnections() || !EnsureStableIds() || !RefreshRiverConnections()) return false;
    for (const auto& [key, parent] : hierarchy.Members())
    {
        if (!Contains(key)) return SDL_SetError("Cannot save an outliner entry for a missing entity");
    }
    const bool hasOutliner = !hierarchy.Folders().empty();
    std::ostringstream outlinerData;
    outlinerData.imbue(std::locale::classic());
    if (hasOutliner && !hierarchy.Write(outlinerData)) return SDL_SetError("Could not serialize the outliner");
    const Uint32 knownSections = 2 + !meshes.empty() + !objects.empty() + !terrain.empty() + !intersections.empty() + hasOutliner;
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
        if (id <= 0 || !std::isfinite(item.yaw) || !std::isfinite(Math::Length(p)) || !std::isfinite(item.cornerRadius) || item.cornerRadius < 0 ||
            !Material::ValidAssetPath(item.material) || static_cast<unsigned>(item.paint) > 2 || !item.ValidAlignment() ||
            static_cast<unsigned>(item.shape) > 2 || !item.ValidTownSquare() || !std::isfinite(item.size) || item.size < Intersection::minSize || item.size > Intersection::maxSize ||
            !(junctionData << id << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' ' << item.cornerRadius
                          << ' ' << std::quoted(item.material) << ' ' << item.stopLines.to_ulong() << ' ' << item.yaw << ' ' << item.pavedThrough))
            return SDL_SetError("Cannot save invalid intersection");
        for (double angle : item.skew)
            if (!std::isfinite(angle) || std::abs(angle) > Intersection::maxSkew || !(junctionData << ' ' << angle))
                return SDL_SetError("Cannot save invalid intersection anchor skew");
        junctionData << ' ' << static_cast<int>(item.paint) << ' ' << static_cast<int>(item.shape) << ' ' << item.size << ' ' << item.island << ' ' << item.alignedApproaches.to_ulong() << ' ' << static_cast<int>(item.townIsland) << ' ' << item.islandSize << '\n';
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
        if (chunk->EditingWater()) return SDL_SetError("Finish or cancel shoreline editing before saving");
        if (!chunk->material.empty() && !Material::ValidAssetPath(chunk->material))
            return SDL_SetError("Cannot save invalid terrain material");
        if (chunk->HasWater())
        {
            const auto& t = chunk->GetTransform();
            terrainData << (chunk->River() ? "river " : "lake ") << t.position.x << ' ' << t.position.y << ' ' << t.position.z << ' ' << t.rotation.y
                        << ' ' << std::quoted(chunk->material) << ' ';
            if (!std::visit([&](const auto& source) { return source.Write(terrainData); }, *chunk->WaterSource())) return SDL_SetError("Cannot save water body");
            terrainData << '\n';
            continue;
        }
        if (chunk->boundary)
        {
            terrainData << "fill " << std::quoted(chunk->material);
            for (const auto* ids : {&chunk->boundary->roads, &chunk->boundary->intersections})
            {
                terrainData << ' ' << ids->size();
                for (int id : *ids) terrainData << ' ' << id;
            }
            terrainData << ' ' << chunk->boundary->waterBodies.size();
            for (auto id : chunk->boundary->waterBodies) terrainData << ' ' << id;
            terrainData << ' ' << chunk->boundary->filterIncidentalPockets << '\n';
            continue;
        }
        const auto& mesh = chunk->Instance().mesh;
        const auto& t = chunk->GetTransform();
        if (!mesh || mesh->assetPath.empty() ||
            !(terrainData << "mesh " << std::quoted(mesh->assetPath) << ' ' << t.position.x << ' ' << t.position.y << ' ' << t.position.z
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
                    WriteSection(output, identityTag, 1, WriteStableIds()) &&
                    WriteSection(output, roadTag, RoadIO::currentVersion, std::move(roadData).str()) &&
                    (intersections.empty() || WriteSection(output, intersectionTag, intersectionVersion, std::move(junctionData).str())) &&
                    (meshes.empty() || WriteSection(output, meshTag, meshVersion, std::move(meshData).str())) &&
                    (objects.empty() || WriteSection(output, objectTag, ObjectIO::currentVersion, std::move(objectData).str())) &&
                    (terrain.empty() || WriteSection(output, terrainTag, terrainVersion, std::move(terrainData).str())) &&
                    (!hasOutliner || WriteSection(output, outlinerTag, SceneHierarchy::currentVersion, std::move(outlinerData).str()));
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

int WorldDocument::SpawnPointIndex() const
{
    for (int i = 0; i < static_cast<int>(objects.size()); ++i)
        if (objects[i]->Type() == ObjectType::SpawnPoint) return i;
    return -1;
}
