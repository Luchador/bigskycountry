#pragma once

#include "road.hpp"
#include "scene_hierarchy.hpp"
#include "bscobject.hpp"
#include "terrain_chunk.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class WorldDocument
{
public:
    SceneHierarchy hierarchy;
    bool Contains(SceneKey key) const;
    const std::string& Path() const { return path; }
    MeshAssets models;
    std::vector<Road> roads;
    std::vector<MeshInstance> meshes; // Static MESH placements, including arbitrary affine transforms.

    // Returns a world-owned object, or nullptr with an SDL error and no object added.
    BSCObject* PlaceObject(ObjectType type, const std::string& modelPath, const ObjectTransform& transform = {});
    // Transfer ownership to/from editor history while keeping outliner indices in sync.
    std::unique_ptr<BSCObject> TakeObject(int index);
    bool InsertObject(int index, std::unique_ptr<BSCObject>&& object, int folder);
    const std::vector<std::unique_ptr<BSCObject>>& Objects() const { return objects; }
    TerrainChunk* PlaceTerrain(const std::string& modelPath, const ObjectTransform& transform = {});
    bool RemoveTerrain(std::size_t index);
    const std::vector<std::unique_ptr<TerrainChunk>>& Terrain() const { return terrain; }
    void TickObjects(float deltaSeconds);

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
            if (object->IsLoaded())
            {
                visit(object->Instance());
            }
        }
    }

    bool Open(const char* documentPath);
    bool Save() const;

private:
    struct Section
    {
        std::array<char, 4> tag;
        std::uint32_t version;
        std::string payload;
    };

    std::vector<std::unique_ptr<TerrainChunk>> terrain;
    std::vector<std::unique_ptr<BSCObject>> objects;
    std::string path;
    std::vector<Section> otherSections; // Copied verbatim when this build edits a world with newer data types.
};
