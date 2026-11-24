#pragma once
#include "mesh.hpp"
#include <atomic>
#include <optional>

// Authored independently of render topology and LODs. Dimensions are meters.
enum class MeshCollisionMode { Detailed, Simple, None };
enum class CollisionShapeType { Capsule, Sphere };
struct CollisionShape
{
    CollisionShapeType type = CollisionShapeType::Capsule;
    // X/Z are the diameter; Y is the capsule's straight section length.
    // Spheres use uniform scale (diameter) on all three axes.
    ObjectTransform transform;
    bool operator==(const CollisionShape&) const = default;
};
struct MeshCollisionData
{
    MeshCollisionMode mode = MeshCollisionMode::Detailed;
    std::vector<CollisionShape> shapes;
    bool operator==(const MeshCollisionData&) const = default;
};
struct MeshCollisionState
{
    std::string file;
    AssetRoot root = AssetRoot::Project;
    std::atomic<std::shared_ptr<const MeshCollisionData>> data;
};

// Runtime primitives are analytic, not tessellated meshes.
struct CollisionCapsule
{
    Vector3 a, b;
    double radius;
    Bounds3 Bounds() const;
    Vector3 ClosestAxisPoint(Vector3 point) const;
    struct Hit { double distance; Vector3 normal; };
    std::optional<Hit> Raycast(Vector3 origin, Vector3 unitDirection, double distance, bool walkableOnly = false) const;
};
namespace MeshCollision
{
    inline constexpr std::size_t maxShapes = 128;
    using Snapshot = std::shared_ptr<const MeshCollisionData>;
    Snapshot Current(const Mesh& mesh);
    bool Bind(Mesh& mesh, AssetRoot root);
    bool Save(const Mesh& mesh, const MeshCollisionData& data);
    bool Valid(const MeshCollisionData& data);
    CollisionCapsule Capsule(const CollisionShape& shape, const MeshTransform& placement = {});
    // Includes shapes outside the visible mesh, for collision streaming/culling.
    Bounds3 Bounds(const MeshInstance& instance);
}
