#pragma once

#include "world_document.hpp"
#include "material.hpp"
#include <filesystem>
#include <span>

// Disk format is explicit little-endian; never serialize these C++ structs directly.
namespace WorldPackage
{
    inline constexpr std::uint32_t version = 1;
    inline constexpr double cellMeters = 512.0;
    inline constexpr double distantMeters = 2000.0;
    struct CellKey
    {
        std::int32_t x = 0, z = 0;
        auto operator<=>(const CellKey&) const = default;
        Vector3 Origin() const { return {x * cellMeters, 0, z * cellMeters}; }
    };
    enum class Layer : std::uint32_t { Surface, Paint, Rail, Water, Transition };
    struct Batch
    {
        EntityId owner;
        Layer layer = Layer::Surface;
        std::string material;
        MeshInstance nearMesh, distantMesh;
    };
    struct Ties { EntityId owner; RailroadTieRun run; };
    struct Cell
    {
        CellKey key;
        std::vector<Batch> batches;
        std::vector<Ties> ties;
    };
    struct Entity
    {
        EntityId id;
        int folder = 0;
        std::string name, model, material;
        MeshTransform transform;
        Bounds3 bounds;
        std::vector<EntityId> dependencies;
    };
    enum class AssetKind : std::uint32_t { Model, Material };
    struct Asset
    {
        AssetKind kind;
        std::string path;
        std::uint64_t hash = 0; // Zero records a missing optional asset/fallback.
        std::array<std::string, materialSlotCount> textures; // Logical paths, indexed by MaterialSlot.
    };
    struct CellRecord
    {
        CellKey key;
        Bounds3 bounds;
        std::uint64_t offset = 0, bytes = 0, hash = 0, nearTriangles = 0, distantTriangles = 0;
        std::vector<EntityId> owners;
    };
    struct Manifest
    {
        std::uint64_t sourceHash = 0;
        std::vector<Entity> entities;
        std::vector<Asset> assets;
        std::vector<CellRecord> cells;
    };
    // Opens only the manifest. Cell payloads can subsequently be loaded independently.
    class Reader
    {
    public:
        bool Open(const std::string& file);
        bool Matches(const std::string& sourceWorld, AssetRoot root) const;
        bool ReadCell(std::size_t index, Cell& cell) const;
        const Manifest& Index() const { return manifest; }
    private:
        std::string path;
        Manifest manifest;
        std::uint64_t payloadStart = 0;
    };
    std::string DefaultPath(const WorldDocument& world);
    std::uint64_t FileHash(const std::string& file);
    std::uint64_t AssetHash(AssetKind kind, const std::string& path, AssetRoot root);
    bool Write(const std::string& file, Manifest manifest, std::span<const Cell> cells);
    bool Cook(WorldDocument& world, const std::string& file);
    // Exposed for regression tests and future incremental cooking.
    std::shared_ptr<Mesh> Simplify(const Mesh& mesh);
    void ClipTriangle(std::map<CellKey, Cell>& cells, EntityId owner, Layer layer,
                      const std::string& material, const std::array<MeshVertex, 3>& triangle);
}
