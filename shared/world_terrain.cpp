#include "world_document.hpp"
#include "terrain_fill.hpp"
#include "railroad_switch.hpp"
#include "spatial_index.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    // Old splits retained the crossing link on only one half. An adjoining,
    // selected railroad with the same endpoint is also part of that crossing.
    // Keep this local to the linked site; nearby unconnected tracks must not
    // become terrain seals just because their footprints overlap in plan view.
    std::vector<const Road*> CrossingPieces(const Road& source, Vector3 site,
                                            std::span<const Road* const> railroads)
    {
        std::vector<const Road*> result{&source};
        if (source.points.size() < 2) return result;
        for (const auto* end : {&source.points.front(), &source.points.back()})
        {
            if (std::hypot(end->position.x-site.x, end->position.z-site.z) > 1e-6) continue;
            for (const auto* road : railroads)
            {
                if (road->points.size() < 2 || std::ranges::find(result, road) != result.end()) continue;
                if (Math::Length(road->points.front().position-end->position) < 1e-6 ||
                    Math::Length(road->points.back().position-end->position) < 1e-6) result.push_back(road);
            }
        }
        return result;
    }
}

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
    std::vector<const RoadGeometry*> supportingFootprints;
    std::vector<std::uint64_t> revisions;
    std::vector<std::array<Vector3, 2>> links;
    const auto& boundary = *chunk.boundary;
    auto groundRoads = boundary.roads;
    for (int id : boundary.roads)
        if (auto it = byId.find(id); it != byId.end())
            for (const auto& p : it->second->points) if (p.bridge)
                groundRoads.insert(p.bridge->lowerRoads.begin(), p.bridge->lowerRoads.end());
    for (int id : groundRoads)
    {
        const auto it = byId.find(id);
        if (it == byId.end())
        {
            revisions.push_back(0);
            continue;
        }
        const Road& road = *it->second;
        footprints.push_back(&road);
        // An overpass needs its lower elements for ground seams and exclusions.
        // Their full lengths may enclose other land far from the selected block;
        // only explicitly selected elements should contribute to pocket choice.
        if (!boundary.roads.contains(id)) supportingFootprints.push_back(&road);
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
    Bounds3 bounds;
    for (const auto* footprint : footprints)
        if (!footprint->bounds.Empty()) { bounds.Include(footprint->bounds.min); bounds.Include(footprint->bounds.max); }
    for (const auto& link : links) for (auto point : link) bounds.Include(point);
    auto missingWater = boundary.waterBodies;
    for (const auto& water : terrain)
    {
        if (!boundary.waterBodies.contains(water->persistentId) || !water->HasWater() || !water->Water().mesh) continue;
        missingWater.erase(water->persistentId);
        const auto b = WorldBounds(water->Water().mesh->bounds, water->Water().transform);
        if (!b.Empty()) { bounds.Include(b.min); bounds.Include(b.max); }
    }
    std::vector<const TerrainChunk*> lakes;
    for (const auto& lake : terrain)
    {
        if (!lake->HasWater() || !lake->Water().mesh || bounds.Empty()) continue;
        const auto b = WorldBounds(lake->Water().mesh->bounds, lake->Water().transform);
        // Ignore elevation here: surrounding terrain slopes down/up to the shore.
        if (!b.Empty() && b.max.x >= bounds.min.x && b.min.x <= bounds.max.x &&
            b.max.z >= bounds.min.z && b.min.z <= bounds.max.z) lakes.push_back(lake.get());
    }
    std::ranges::sort(lakes, [](const auto* a, const auto* b) { return a->persistentId < b->persistentId; });
    std::vector<std::pair<std::uint64_t, std::uint64_t>> waterRevisions;
    for (const auto* lake : lakes) waterRevisions.emplace_back(lake->persistentId, lake->WaterRevision());
    for (auto id : missingWater) waterRevisions.emplace_back(id, 0);
    std::ranges::sort(waterRevisions);
    if (revisions == chunk.sourceRevisions && links == chunk.anchorLinks && waterRevisions == chunk.sourceWaterBodies) return;
    if (!missingWater.empty())
    {
        // A selected bank is part of the authored perimeter, not just an optional
        // exclusion. Suspend the fill until that source returns (e.g. on undo).
        chunk.SetMesh({});
        chunk.sourceRevisions = std::move(revisions);
        chunk.anchorLinks = std::move(links);
        chunk.sourceWaterBodies = std::move(waterRevisions);
        return;
    }
    std::vector<std::vector<Vector3>> shorelines;
    for (const auto* lake : lakes)
    {
        auto& rim = shorelines.emplace_back();
        rim.reserve(lake->BuiltShoreline().size());
        for (const auto& sample : lake->BuiltShoreline()) rim.push_back(lake->Water().transform.Point(sample.position));
    }
    std::vector<std::array<Vector3, 2>> junctionSeams;
    std::vector<const Road*> railroads;
    for (int id : groundRoads)
        if (const auto road = byId.find(id); road != byId.end() && road->second->style.railroad)
            railroads.push_back(road->second);
    std::map<std::pair<int,int>, std::vector<CrossingSite>> crossingSites;
    const auto sitesFor = [&](const Road& a, const Road& b) -> const std::vector<CrossingSite>&
    {
        const auto [found, inserted] = crossingSites.try_emplace(std::pair{a.id,b.id});
        if (inserted) found->second = FindRailroadCrossings(a,b);
        return found->second;
    };
    for (const auto* source : railroads)
    {
        const Road& road = *source;
        for (const auto& link : road.crossings)
        {
            const auto other = byId.find(link.railroadId);
            if (other == byId.end() || !other->second->style.railroad || !groundRoads.contains(link.railroadId)) continue;
            const auto& sites = sitesFor(road,*other->second);
            if (sites.empty()) continue;
            const auto score = [&](const CrossingSite& site)
            {
                return std::abs(site.link.roadFraction - link.roadFraction) +
                       std::abs(site.link.railFraction - link.railFraction);
            };
            const auto best = std::min_element(sites.begin(), sites.end(),
                                               [&](const auto& a, const auto& b) { return score(a) < score(b); });
            const auto firstPieces = CrossingPieces(road,best->position,railroads);
            const auto secondPieces = CrossingPieces(*other->second,best->position,railroads);
            for (const auto* a : firstPieces)
                for (const auto* b : secondPieces)
                    for (const auto& site : sitesFor(*a,*b))
                        if (std::hypot(site.position.x-best->position.x, site.position.z-best->position.z) < 1e-6)
                            if (auto seams = PlanRailroadJunction(*a,*b,site))
                                junctionSeams.insert(junctionSeams.end(), seams->begin(), seams->end());
        }
    }
    std::map<int,const Road*> switchHosts;
    for (int id : groundRoads)
        if (const auto road=byId.find(id);road!=byId.end() && road->second->style.railroad)
            for (const auto& p : road->second->points) if (p.rampStation) switchHosts[p.rampStation]=road->second;
    for (int id : groundRoads)
        if (const auto road=byId.find(id);road!=byId.end() && road->second->style.railroad)
            for (const auto& p : road->second->points)
                if (p.railSwitch && switchHosts.contains(p.railSwitch->station))
                {
                    auto seams=RailroadSwitchTerrainSeams(*switchHosts.at(p.railSwitch->station),*road->second);
                    junctionSeams.insert(junctionSeams.end(),seams.begin(),seams.end());
                }
    const auto scope = boundary.filterIncidentalPockets ? TerrainFillScope::SelectedBoundary : TerrainFillScope::AllPockets;
    chunk.SetMesh(BuildTerrainFill(footprints, links, shorelines, junctionSeams, scope, supportingFootprints));
    chunk.sourceRevisions = std::move(revisions);
    chunk.anchorLinks = std::move(links);
    chunk.sourceWaterBodies = std::move(waterRevisions);
}

void WorldDocument::RefreshTerrainFills()
{
    RefreshOverpasses();
    if (terrainRevision == RoadGeometry::latestRevision && terrainWaterRevision == TerrainChunk::LatestWaterRevision()) return;
    terrainRevision = RoadGeometry::latestRevision;
    terrainWaterRevision = TerrainChunk::LatestWaterRevision();
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
    if (boundary.Empty() || (!boundary.roads.empty() && *boundary.roads.begin() <= 0) ||
        (!boundary.intersections.empty() && *boundary.intersections.begin() <= 0) ||
        (!boundary.waterBodies.empty() && *boundary.waterBodies.begin() == 0))
    {
        SDL_SetError("Select roads, railroads, intersections, rivers or lakes around the area to fill");
        return nullptr;
    }
    RefreshOverpasses();
    auto chunk = std::make_unique<TerrainChunk>();
    chunk->boundary = std::move(boundary);
    int folder = 0;
    if (!allowOpen)
    {
        for (const auto& water : terrain)
            if (chunk->boundary->waterBodies.contains(water->persistentId) && water->EditingWater())
            {
                SDL_SetError("Finish editing the selected river or lake before filling terrain");
                return nullptr;
            }
        std::map<int, const Road*> byId;
        for (std::size_t i = 0; i < roads.size(); ++i)
        {
            const Road& road = roads[i];
            byId.emplace(road.id, &road);
            if (!folder && chunk->boundary->roads.contains(road.id))
                folder = hierarchy.Parent({SceneKind::Road, static_cast<int>(i)});
        }
        RebuildTerrain(*chunk, byId);
        if (!chunk->Instance().mesh)
        {
            SDL_SetError("Select boundaries that enclose dry land, with road edges meeting river/lake banks at matching elevations");
            return nullptr;
        }
    }
    if (!folder)
        for (int id : chunk->boundary->intersections)
            if ((folder = hierarchy.Parent({SceneKind::Intersection, id}))) break;
    if (!folder)
        for (std::size_t i = 0; i < terrain.size(); ++i)
            if (chunk->boundary->waterBodies.contains(terrain[i]->persistentId) &&
                (folder = hierarchy.Parent({SceneKind::Terrain, static_cast<int>(i)}))) break;
    if (!chunk->boundary->roads.empty()) lastRoadId = std::max(lastRoadId, *chunk->boundary->roads.rbegin());
    terrainRevision = 0;
    const SceneKey key{SceneKind::Terrain, static_cast<int>(terrain.size())};
    if (!(chunk->persistentId = AllocatePlacementId())) return nullptr;
    auto& placed = terrain.emplace_back(std::move(chunk));
    if (folder) hierarchy.Move({key}, folder);
    return placed.get();
}

void WorldDocument::MoveLastTerrainTo(std::size_t index)
{
    if (index < terrain.size() - 1)
    {
        hierarchy.ReplaceRange(SceneKind::Terrain, static_cast<int>(index), 0, 1);
        std::rotate(terrain.begin() + index, terrain.end() - 1, terrain.end());
    }
}
