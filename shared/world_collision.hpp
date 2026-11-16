#pragma once

#include "mesh.hpp"
#include "rigid_body.hpp"
#include <optional>
#include <map>

class WorldDocument;

// Immutable per-chunk BVHs can be attached/detached on the main thread.
// Dynamic pawns remain separate; streaming never rebuilds a whole-world tree.
class WorldCollision
{
public:
    struct Hit
    {
        Vector3 position, normal;
        double distance;
    };
    void Build(const WorldDocument& world);
    void AddMesh(const MeshInstance& instance);
    void Finish();
    void Attach(std::uint64_t key, std::shared_ptr<const WorldCollision> chunk) { chunks[key] = std::move(chunk); }
    void Detach(std::uint64_t key) { chunks.erase(key); }
    void ClearChunks() { chunks.clear(); }
    std::size_t ChunkCount() const { return chunks.size(); }
    std::size_t MemoryBytes() const;
    const Bounds3& Bounds() const;
    std::optional<Hit> Raycast(Vector3 origin, Vector3 direction, double distance, bool walkableOnly = false) const;
    std::optional<Hit> Ground(Vector3 feet, double above, double below) const;
    bool SphereBlocked(Vector3 center, double radius) const;
    // Physical contacts include floor and ceiling faces as well as obstacles.
    std::optional<BodyContact> SphereContact(Vector3 center, double radius) const;

private:
    std::map<std::uint64_t, std::shared_ptr<const WorldCollision>> chunks;
    struct Triangle { Vector3 a, b, c, normal; Bounds3 bounds; };
    struct Node { Bounds3 bounds; size_t first = 0, count = 0, left = 0, right = 0; };
    std::vector<Triangle> triangles;
    std::vector<Node> nodes;
    void AddTriangle(Vector3 a, Vector3 b, Vector3 c);
    size_t BuildNode(size_t first, size_t count);
    void Trace(size_t node, Vector3 origin, Vector3 direction, bool walkableOnly, std::optional<Hit>& hit, double& distance) const;
    bool Overlap(size_t node, Vector3 center, double radius) const;
    void Contact(size_t node, Vector3 center, double radius, std::optional<BodyContact>& result) const;
};
