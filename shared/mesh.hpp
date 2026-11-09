#pragma once

#include "math.hpp"

#include <SDL3/SDL.h>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

struct MeshTransform
{
    Vector3 origin{};
    Vector3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};

    Vector3 Point(Vector3 p) const { return origin + x * p.x + y * p.y + z * p.z; }
    Vector3 Direction(Vector3 p) const { return x * p.x + y * p.y + z * p.z; }
    // Inverse rotation for an orthonormal basis.
    Vector3 Unrotate(Vector3 p) const { return {Math::Dot(p, x), Math::Dot(p, y), Math::Dot(p, z)}; }
    double Determinant() const { return Math::Dot(x, Math::Cross(y, z)); }
    bool Valid() const;
};

struct MeshVertex
{
    Vector3 position;
    Vector3 normal;
    SDL_FPoint uv{};
    SDL_FColor color{1, 1, 1, 1};
};

struct Mesh
{
    std::string assetPath; // Relative to assets/models.
    std::vector<MeshVertex> vertices;
    std::vector<std::uint32_t> indices; // Triangle list; front winding matches the world renderer.
};

struct MeshInstance
{
    std::shared_ptr<const Mesh> mesh;
    MeshTransform transform;
};

struct ObjectTransform
{
    Vector3 position{};
    Vector3 rotation{};       // Radians about X, Y, Z, applied in that order.
    Vector3 scale{1, 1, 1};   // Along the object's local axes.

    MeshTransform ToMeshTransform() const;
    bool operator==(const ObjectTransform&) const = default;
};

class MeshPlacement
{
public:
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
    std::shared_ptr<const Mesh> Load(const std::string& relativePath);

private:
    std::unordered_map<std::string, std::shared_ptr<const Mesh>> loaded;
};
