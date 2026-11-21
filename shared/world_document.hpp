#pragma once

#include "intersection.hpp"
#include <map>
#include "scene_hierarchy.hpp"
#include "bscobject.hpp"
#include "terrain_chunk.hpp"
#include "editor_geometry_cache.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Typed persistent identity. Road and intersection IDs retain their existing namespace.
struct EntityId
{
    SceneKind kind = SceneKind::World;
    std::uint64_t value = 0;
    auto operator<=>(const EntityId&) const = default;
};

class WorldDocument
{
public:
    explicit WorldDocument(AssetRoot location = AssetRoot::Cooked,
                           std::shared_ptr<EditorGeometryCache> cache = {})
        : models(location), geometryCache(std::move(cache)) {}
    SceneHierarchy hierarchy;
    bool Contains(SceneKey key) const;
    const std::string& Path() const { return path; }
    MeshAssets models;
    std::vector<Road> roads;
    std::map<int, Intersection> intersections; // Stable positive IDs, also used by the outliner.
    void UpdateIntersections(const std::set<int>& changedRoads = {});
    const auto& IntersectionSurfaces() const { return junctions; }
    int CreateIntersection(Vector3 position); // Returns 0 on failure.
    struct ConnectedEnd
    {
        std::size_t road;
        bool first;
        IntersectionAnchor anchor;
        bool operator==(const ConnectedEnd&) const = default;
    };
    std::vector<ConnectedEnd> ConnectedEnds(int intersectionId) const;
    std::vector<MeshInstance> meshes; // Static MESH placements, including arbitrary affine transforms.

    // Returns a world-owned object, or nullptr with an SDL error and no object added.
    BSCObject* PlaceObject(ObjectType type, const std::string& modelPath, const ObjectTransform& transform = {});
    // Transfer ownership to/from editor history while keeping outliner indices in sync.
    std::unique_ptr<BSCObject> TakeObject(int index);
    bool InsertObject(int index, std::unique_ptr<BSCObject>&& object, int folder);
    const std::vector<std::unique_ptr<BSCObject>>& Objects() const { return objects; }
    TerrainChunk* PlaceTerrain(const std::string& modelPath, const ObjectTransform& transform = {});
    TerrainChunk* PlaceWater(const WaterDefinition& water, const ObjectTransform& transform = {});
    TerrainChunk* PlaceRiver(const RiverDefinition& river, const ObjectTransform& transform = {}) { return PlaceWater(river, transform); }
    TerrainChunk* PlaceLake(const LakeDefinition& lake, const ObjectTransform& transform = {});
    int AllocateRoadId();
    int NextRampStationId() const;
    void RefreshRampConnections(bool force = false);
    // Loading and history can restore a perimeter before all of its sources exist.
    TerrainChunk* PlaceTerrainFill(TerrainBoundary boundary, bool allowOpen = false);
    void RefreshTerrainFills();
    bool RefreshRiverConnections();
    bool ConnectRiver(int tributary, int end, int receiver, Vector3 position);
    void RefreshOverpasses();
    void RefreshRailroadCrossings(bool force = false);
    // Restore history at a specific outliner index after successfully loading the chunk.
    void MoveLastTerrainTo(std::size_t index);
    bool RemoveTerrain(std::size_t index);
    const std::vector<std::unique_ptr<TerrainChunk>>& Terrain() const { return terrain; }
    void TickObjects(float deltaSeconds);
    int SpawnPointIndex() const;

    template<typename Visit>
    void ForEachMesh(Visit visit) const
    {
        for (const MeshInstance& instance : meshes)
        {
            visit(instance);
        }
        for (const auto& chunk : terrain)
        {
            visit(chunk->Instance());
        }
        for (const auto& object : objects)
        {
            if (object->IsLoaded() && !object->EditorOnly())
            {
                visit(object->Instance());
            }
        }
    }

    // Resolve a runtime source path without loading authored meshes; saving is disabled.
    bool OpenPath(const char* documentPath);
    bool Open(const char* documentPath);
    bool Save();
    bool EnsureStableIds();
    EntityId StableId(SceneKey key) const;
    std::uint64_t AllocatePlacementId();

private:
    std::shared_ptr<EditorGeometryCache> geometryCache; // Editor opt-in; never saved or required by the game/cooker.
    std::map<int, IntersectionSurface> junctions; // Derived from the current road ends; never serialized.
    bool ValidConnections() const;
    int lastRoadId = 0, lastIntersectionId = 0;
    std::uint64_t lastPlacementId = 0;
    bool ReadStableIds(const std::string& payload);
    std::string WriteStableIds() const;
    std::uint64_t rampRevision = 0;
    // Only source revisions, never copies of generated turnout meshes.
    std::map<int, std::uint64_t> switchSources;
    std::set<std::pair<int,int>> switchLinks;
    void AlignRailroadSwitches();
    void RefreshRailroadSwitches();
    void RefreshCurvedConnectors();
    std::map<int, std::uint64_t> curvedConnectorSources;
    std::set<std::pair<int, int>> curvedConnectorLinks;
    std::uint64_t terrainRevision = 0;
    std::uint64_t terrainWaterRevision = 0;
    std::uint64_t riverRevision = 0;
    std::map<std::uint64_t, std::vector<std::pair<std::uint64_t,std::uint64_t>>> riverNetworks;
    std::uint64_t bridgeRevision = 0;
    std::uint64_t bridgeWaterRevision = 0;
    std::size_t bridgeRoadCount = 0;
    std::size_t bridgeTerrainCount = 0;
    std::uint64_t crossingRevision = 0;
    struct CrossingSources
    {
        std::uint64_t roadRevision = 0;
        std::vector<std::pair<RailroadCrossingLink, std::uint64_t>> railroads;
        bool operator==(const CrossingSources&) const = default;
    };
    // One transient entry per road with crossings; no edit history or mesh copies.
    std::map<int, CrossingSources> crossingSources;
    void RebuildTerrain(TerrainChunk& chunk, const std::map<int, const Road*>& byId);
    struct Section
    {
        std::array<char, 4> tag;
        std::uint32_t version;
        std::string payload;
    };

    std::vector<std::unique_ptr<TerrainChunk>> terrain;
    std::vector<std::unique_ptr<BSCObject>> objects;
    std::string path;
    bool runtimePathOnly = false;
    std::vector<Section> otherSections; // Copied verbatim when this build edits a world with newer data types.
};
