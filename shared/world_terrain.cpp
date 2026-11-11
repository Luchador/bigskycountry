#include "world_document.hpp"
#include "terrain_fill.hpp"

#include <algorithm>
#include <limits>

int WorldDocument::AllocateRoadId()
{
    if (lastRoadId == std::numeric_limits<int>::max())
    {
        SDL_SetError("No road IDs available");
        return 0;
    }
    return ++lastRoadId;
}

void WorldDocument::RebuildTerrain(TerrainChunk& chunk, const std::map<int, const Road*>& byId)
{
    std::vector<const RoadGeometry*> footprints;
    std::vector<std::uint64_t> revisions;
    std::vector<std::array<Vector3, 2>> links;
    const auto& boundary = *chunk.boundary;
    for (int id : boundary.roads)
    {
        const auto it = byId.find(id);
        if (it == byId.end())
        {
            revisions.push_back(0);
            continue;
        }
        const Road& road = *it->second;
        footprints.push_back(&road);
        revisions.push_back(road.revision);
        if (road.points.size() < 2) continue;
        for (const RoadPoint* end : {&road.points.front(), &road.points.back()})
        {
            if (!end->anchor || !boundary.intersections.contains(end->anchor->intersectionId)) continue;
            const auto it = intersections.find(end->anchor->intersectionId);
            if (it != intersections.end() && !junctions.contains(it->first) &&
                Math::Length(end->position - it->second.AnchorPosition(end->anchor->side)) < 1e-7)
                links.push_back({end->position, it->second.position});
        }
    }
    for (int id : boundary.intersections)
    {
        const auto it = junctions.find(id);
        revisions.push_back(it == junctions.end() ? 0 : it->second.geometry.revision);
        if (it != junctions.end()) footprints.push_back(&it->second.geometry);
    }
    if (revisions == chunk.sourceRevisions && links == chunk.anchorLinks) return;
    chunk.SetMesh(BuildTerrainFill(footprints, links));
    chunk.sourceRevisions = std::move(revisions);
    chunk.anchorLinks = std::move(links);
}

void WorldDocument::RefreshTerrainFills()
{
    if (terrainRevision == RoadGeometry::latestRevision) return;
    terrainRevision = RoadGeometry::latestRevision;
    std::map<int, const Road*> byId;
    bool indexed = false;
    for (auto& chunk : terrain)
    {
        if (!chunk->boundary) continue;
        if (!indexed)
        {
            for (const auto& road : roads) byId.emplace(road.id, &road);
            indexed = true;
        }
        RebuildTerrain(*chunk, byId);
    }
}

TerrainChunk* WorldDocument::PlaceTerrainFill(TerrainBoundary boundary, bool allowOpen)
{
    if (boundary.roads.empty() || *boundary.roads.begin() <= 0 ||
        (!boundary.intersections.empty() && *boundary.intersections.begin() <= 0))
    {
        SDL_SetError("Select the roads and intersections around the area to fill");
        return nullptr;
    }
    auto chunk = std::make_unique<TerrainChunk>();
    chunk->boundary = std::move(boundary);
    if (!allowOpen)
    {
        std::map<int, const Road*> byId;
        for (const auto& road : roads) byId.emplace(road.id, &road);
        RebuildTerrain(*chunk, byId);
        if (!chunk->Instance().mesh)
        {
            SDL_SetError("Select roads and intersections that enclose an area at matching elevations");
            return nullptr;
        }
    }
    lastRoadId = std::max(lastRoadId, *chunk->boundary->roads.rbegin());
    terrainRevision = 0;
    return terrain.emplace_back(std::move(chunk)).get();
}

void WorldDocument::MoveLastTerrainTo(std::size_t index)
{
    if (index < terrain.size() - 1)
    {
        hierarchy.ReplaceRange(SceneKind::Terrain, static_cast<int>(index), 0, 1);
        std::rotate(terrain.begin() + index, terrain.end() - 1, terrain.end());
    }
}
