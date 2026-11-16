#include "world_document.hpp"
#include <algorithm>
#include <limits>
#include <locale>
#include <sstream>
#include <set>

std::uint64_t WorldDocument::AllocatePlacementId()
{
    if (lastPlacementId == std::numeric_limits<std::uint64_t>::max())
    {
        SDL_SetError("World placement IDs exhausted");
        return 0;
    }
    return ++lastPlacementId;
}

EntityId WorldDocument::StableId(SceneKey key) const
{
    if (!Contains(key)) return {};
    switch (key.kind)
    {
    case SceneKind::Road: return {key.kind, static_cast<std::uint64_t>(roads.at(key.index).id)};
    case SceneKind::Intersection: return {key.kind, static_cast<std::uint64_t>(key.index)};
    case SceneKind::Object: return {key.kind, objects.at(key.index)->persistentId};
    case SceneKind::Terrain: return {key.kind, terrain.at(key.index)->persistentId};
    case SceneKind::Mesh: return {key.kind, meshes.at(key.index).persistentId};
    default: return {};
    }
}

bool WorldDocument::EnsureStableIds()
{
    std::set<int> roadIds;
    for (auto& road : roads)
    {
        if (road.id < 0 || (road.id && !roadIds.insert(road.id).second)) return SDL_SetError("Invalid road identity");
        lastRoadId = std::max(lastRoadId, road.id);
    }
    for (auto& road : roads)
        if (!road.id && !(road.id = AllocateRoadId())) return false;
    if (!intersections.empty()) lastIntersectionId = std::max(lastIntersectionId, intersections.rbegin()->first);
    std::vector<std::uint64_t*> placements;
    for (auto& mesh : meshes) placements.push_back(&mesh.persistentId);
    for (auto& chunk : terrain) placements.push_back(&chunk->persistentId);
    for (auto& object : objects) placements.push_back(&object->persistentId);
    std::set<std::uint64_t> used;
    for (auto* id : placements)
    {
        if (*id && !used.insert(*id).second) return SDL_SetError("Duplicate world placement identity");
        lastPlacementId = std::max(lastPlacementId, *id);
    }
    for (auto* id : placements)
        if (!*id && !(*id = AllocatePlacementId())) return false;
    return true;
}

std::string WorldDocument::WriteStableIds() const
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << lastRoadId << ' ' << lastIntersectionId << ' ' << lastPlacementId << '\n';
    out << meshes.size();
    for (const auto& mesh : meshes) out << ' ' << mesh.persistentId;
    out << '\n' << terrain.size();
    for (const auto& chunk : terrain) out << ' ' << chunk->persistentId;
    out << '\n' << objects.size();
    for (const auto& object : objects) out << ' ' << object->persistentId;
    out << '\n';
    return out.str();
}

bool WorldDocument::ReadStableIds(const std::string& payload)
{
    std::istringstream in(payload);
    in.imbue(std::locale::classic());
    int roadHigh, intersectionHigh;
    std::uint64_t placementHigh;
    if (!(in >> roadHigh >> intersectionHigh >> placementHigh) || roadHigh < lastRoadId || intersectionHigh < 0 ||
        (!intersections.empty() && intersectionHigh < intersections.rbegin()->first))
        return SDL_SetError("Invalid EIDS allocation counters");
    std::set<std::uint64_t> used;
    auto read = [&](auto& items, auto id) {
        std::uint64_t count;
        if (!(in >> count) || count != items.size()) return false;
        for (auto& item : items)
        {
            auto& value = id(item);
            if (!(in >> value) || !value || value > placementHigh || !used.insert(value).second) return false;
        }
        return true;
    };
    if (!read(meshes, [](auto& m) -> auto& { return m.persistentId; }) ||
        !read(terrain, [](auto& t) -> auto& { return t->persistentId; }) ||
        !read(objects, [](auto& o) -> auto& { return o->persistentId; }) || !(in >> std::ws).eof())
        return SDL_SetError("Invalid EIDS placement table");
    lastRoadId = roadHigh;
    lastIntersectionId = intersectionHigh;
    lastPlacementId = placementHigh;
    return true;
}
