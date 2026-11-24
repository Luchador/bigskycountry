#pragma once

#include "asset_paths.hpp"
#include "math.hpp"

#include <SDL3/SDL.h>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

struct MeshTransform
{
    Vector3 origin{};
    Vector3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};

    bool operator==(const MeshTransform&) const = default;
    static MeshTransform AxisRotation(Vector3 unitAxis, double radians);
    Vector3 Point(Vector3 p) const { return origin + x * p.x + y * p.y + z * p.z; }
    Vector3 Direction(Vector3 p) const { return x * p.x + y * p.y + z * p.z; }
    // Inverse rotation for an orthonormal basis.
    Vector3 Unrotate(Vector3 p) const { return {Math::Dot(p, x), Math::Dot(p, y), Math::Dot(p, z)}; }
    double Determinant() const { return Math::Dot(x, Math::Cross(y, z)); }
    // Inverse-transpose up to a positive scalar. Normalize after applying Direction().
    MeshTransform NormalTransform() const;
    bool Valid() const;
};

struct Bounds3
{
    static constexpr double inf = std::numeric_limits<double>::infinity();
    Vector3 min{inf, inf, inf}, max{-inf, -inf, -inf};
    bool operator==(const Bounds3&) const = default;
    bool Empty() const { return min.x > max.x; }
    void Include(Vector3 point);
};

struct MeshVertex
{
    Vector3 position;
    Vector3 normal;
    SDL_FPoint uv{};
    SDL_FColor color{1, 1, 1, 1};
};

struct MeshMaterialSlot
{
    std::string name;
    std::string material; // Relative to assets/textures; empty uses imported vertex colors.
};

// Shared between all imports/instances of the same model in this asset root.
struct MeshMaterials
{
    std::vector<MeshMaterialSlot> slots;
    std::string file;
    AssetRoot root = AssetRoot::Project;
    std::uint64_t revision = 1;
};

struct MeshPart
{
    std::size_t firstIndex, indexCount, slot;
};

struct Mesh
{
    static constexpr std::size_t maxLODCount = 16;
    // LOD0 is this mesh. Additional levels share its material assignments.
    struct LOD { std::shared_ptr<Mesh> mesh; double threshold; };
    std::vector<LOD> lods;
    bool lodDistances = false; // Thresholds are meters; otherwise fractions of screen height.
    std::size_t SelectLOD(double screenHeight, double localDistance, int previous = -1) const;
    const Mesh& Level(std::size_t level) const { return level ? *lods.at(level - 1).mesh : *this; }
    Bounds3 bounds; // Local space. Filled when an imported or generated world mesh is finished.
    void UpdateBounds();
    std::string assetPath; // Relative to assets/models.
    std::vector<MeshVertex> vertices;
    // Imported COLOR_0, before material tint. Optional; used by mesh inspection.
    std::vector<SDL_FColor> vertexColors;
    std::vector<std::uint32_t> indices; // Triangle list; front winding matches the world renderer.
    std::vector<MeshPart> parts; // Imported primitive ranges, including their generated back faces.
    std::shared_ptr<MeshMaterials> materials;
    const std::string& MaterialAt(std::size_t index) const;
    bool HasMaterials() const;
};

struct MeshInstance
{
    std::shared_ptr<const Mesh> mesh;
    MeshTransform transform;
    std::uint64_t persistentId = 0; // Authored placement identity; zero for transient render instances.
};

struct ObjectTransform
{
    Vector3 position{};
    Vector3 rotation{};       // Radians about X, Y, Z, applied in that order.
    Vector3 scale{1, 1, 1};   // Along the object's local axes.

    MeshTransform ToMeshTransform() const;
    // Compose a world-space rotation about this transform's own origin.
    void Rotate(Vector3 unitAxis, double radians);
    bool operator==(const ObjectTransform&) const = default;
};

class MeshPlacement
{
public:
    std::uint64_t persistentId = 0; // Preserved by history; duplicates receive a new ID.
    const ObjectTransform& GetTransform() const { return transform; }
    bool SetTransform(const ObjectTransform& value);
    void SetMesh(std::shared_ptr<const Mesh> mesh);
    bool IsLoaded() const { return instance.mesh != nullptr; }
    const MeshInstance& Instance() const { return instance; }

private:
    ObjectTransform transform;
    MeshInstance instance; // Updated on transform edits. Rendering and picking share this matrix.
};

class MeshAssets
{
public:
    explicit MeshAssets(AssetRoot location = AssetRoot::Cooked) : location(location) {}
    AssetRoot Location() const { return location; }
    void PruneUnused();
    std::shared_ptr<const Mesh> Load(const std::string& relativePath);

private:
    AssetRoot location;
    std::unordered_map<std::string, std::shared_ptr<const Mesh>> loaded;
};
