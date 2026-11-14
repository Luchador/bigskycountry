#pragma once

#include "mesh.hpp"
#include <optional>

class WorldDocument;

// Built once from the authored world when gameplay starts. Dynamic pawns are
// checked separately by GameWorld, so moving a car never rebuilds this tree.
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
    std::optional<Hit> Raycast(Vector3 origin, Vector3 direction, double distance, bool walkableOnly = false) const;
    std::optional<Hit> Ground(Vector3 feet, double above, double below) const;
    bool SphereBlocked(Vector3 center, double radius) const;

private:
    struct Triangle { Vector3 a, b, c, normal; Bounds3 bounds; };
    struct Node { Bounds3 bounds; size_t first = 0, count = 0, left = 0, right = 0; };
    std::vector<Triangle> triangles;
    std::vector<Node> nodes;
    void AddTriangle(Vector3 a, Vector3 b, Vector3 c);
    size_t BuildNode(size_t first, size_t count);
    void Trace(size_t node, Vector3 origin, Vector3 direction, bool walkableOnly, std::optional<Hit>& hit, double& distance) const;
    bool Overlap(size_t node, Vector3 center, double radius) const;
};
