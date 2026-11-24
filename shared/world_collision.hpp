#pragma once

#include "mesh.hpp"
#include "mesh_collision.hpp"
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
    // Terrain surfaces always retain triangle collision, even when their source
    // asset has simple shapes for use as an object.
    void AddMesh(const MeshInstance& instance, bool useMeshCollision = true);
    void Finish();
    void Attach(std::uint64_t key, std::shared_ptr<const WorldCollision> chunk);
    void Detach(std::uint64_t key);
    void ClearChunks();
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
    std::vector<CollisionCapsule> capsules;
    std::vector<Node> capsuleNodes;
    size_t BuildCapsuleNode(size_t first, size_t count);
    void TraceCapsules(size_t node, Vector3 origin, Vector3 direction, bool walkableOnly, std::optional<Hit>& hit, double& distance) const;
    void ContactCapsules(size_t node, Vector3 center, double radius, bool blocking, std::optional<BodyContact>& result) const;
    // A second BVH rejects distant streamed chunks before consulting their
    // triangle trees. Rebuild once after a batch of attachment changes, never
    // for each car or physics substep. Chunks are immutable while attached.
    struct Chunk { const WorldCollision* collision; Bounds3 bounds; };
    mutable std::vector<Chunk> orderedChunks;
    mutable std::vector<Node> chunkNodes;
    mutable Bounds3 bounds;
    mutable bool chunksDirty = true;
    void IndexChunks() const;
    size_t BuildChunkNode(size_t first, size_t count) const;
    void TraceChunks(size_t node, Vector3 origin, Vector3 direction, bool walkableOnly,
                     std::optional<Hit>& hit, double& distance) const;
    bool OverlapChunks(size_t node, Vector3 center, double radius) const;
    void ContactChunks(size_t node, Vector3 center, double radius, std::optional<BodyContact>& result) const;
    void AddTriangle(Vector3 a, Vector3 b, Vector3 c);
    size_t BuildNode(size_t first, size_t count);
    void Trace(size_t node, Vector3 origin, Vector3 direction, bool walkableOnly, std::optional<Hit>& hit, double& distance) const;
    bool Overlap(size_t node, Vector3 center, double radius) const;
    void Contact(size_t node, Vector3 center, double radius, std::optional<BodyContact>& result) const;
};
