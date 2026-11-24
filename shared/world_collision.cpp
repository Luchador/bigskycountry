#include "world_collision.hpp"
#include "world_document.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    double Axis(Vector3 p, int axis)
    {
        if (axis == 0) return p.x;
        if (axis == 1) return p.y;
        return p.z;
    }

    bool RayBounds(const Bounds3& bounds, Vector3 origin, Vector3 direction, double length)
    {
        double near = 0, far = length;
        for (int axis = 0; axis < 3; ++axis)
        {
            const double p = Axis(origin, axis), d = Axis(direction, axis);
            const double lo = Axis(bounds.min, axis), hi = Axis(bounds.max, axis);
            if (std::abs(d) < 1e-12)
            {
                if (p < lo || p > hi) return false;
                continue;
            }
            double a = (lo - p) / d, b = (hi - p) / d;
            if (a > b) std::swap(a, b);
            near = std::max(near, a);
            far = std::min(far, b);
            if (near > far) return false;
        }
        return true;
    }

    bool SphereBounds(const Bounds3& bounds, Vector3 center, double radius)
    {
        const Vector3 nearest{std::clamp(center.x, bounds.min.x, bounds.max.x),
                              std::clamp(center.y, bounds.min.y, bounds.max.y),
                              std::clamp(center.z, bounds.min.z, bounds.max.z)};
        const auto offset = center - nearest;
        return radius > 0 && Math::Dot(offset, offset) < radius * radius;
    }

    Vector3 ClosestPoint(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        const Vector3 ab = b - a, ac = c - a, ap = p - a;
        const double d1 = Math::Dot(ab, ap), d2 = Math::Dot(ac, ap);
        if (d1 <= 0 && d2 <= 0) return a;
        const Vector3 bp = p - b;
        const double d3 = Math::Dot(ab, bp), d4 = Math::Dot(ac, bp);
        if (d3 >= 0 && d4 <= d3) return b;
        const double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
        const Vector3 cp = p - c;
        const double d5 = Math::Dot(ab, cp), d6 = Math::Dot(ac, cp);
        if (d6 >= 0 && d5 <= d6) return c;
        const double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
        const double va = d3 * d6 - d5 * d4;
        if (va <= 0 && d4 >= d3 && d5 >= d6)
            return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
        const double inverse = 1.0 / (va + vb + vc);
        return a + ab * (vb * inverse) + ac * (vc * inverse);
    }
}

void WorldCollision::AddTriangle(Vector3 a, Vector3 b, Vector3 c)
{
    const Vector3 normal = Math::Cross(b - a, c - a);
    if (Math::Length(normal) < 1e-10) return;
    Bounds3 bounds;
    bounds.Include(a);
    bounds.Include(b);
    bounds.Include(c);
    triangles.push_back({a, b, c, Math::Normalize(normal), bounds});
}

void WorldCollision::AddMesh(const MeshInstance& instance, bool useMeshCollision)
{
    if (!instance.mesh) return;
    const auto& mesh = *instance.mesh;
    const auto collision = MeshCollision::Current(mesh);
    if (useMeshCollision && collision->mode == MeshCollisionMode::None) return;
    if (useMeshCollision && collision->mode == MeshCollisionMode::Simple)
    {
        for (const auto& shape : collision->shapes) capsules.push_back(MeshCollision::Capsule(shape, instance.transform));
        return;
    }
    const bool mirrored = instance.transform.Determinant() < 0;
    for (size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        const auto point = [&](size_t index) {
            return instance.transform.Point(mesh.vertices[mesh.indices[index]].position);
        };
        AddTriangle(point(i), point(i + (mirrored ? 2 : 1)), point(i + (mirrored ? 1 : 2)));
    }
}

void WorldCollision::Build(const WorldDocument& world)
{
    ClearChunks();
    triangles.clear();
    nodes.clear();
    capsules.clear();
    capsuleNodes.clear();
    const auto road = [&](const RoadGeometry& geometry) {
        const auto surface = [&](const auto& faces) {
            for (const auto& t : faces) AddTriangle(t[0].position, t[1].position, t[2].position);
        };
        surface(geometry.pavementTriangles);
        surface(geometry.edgeTriangles);
        surface(geometry.bridgeTriangles);
        for (const auto& [path, faces] : geometry.transitionStrips) surface(faces);
        for (const auto& segment : geometry.railroadSegments)
        {
            surface(segment.rails);
            for (const auto& run : segment.ties)
                for (int i = 0; i < run.count; ++i) AddMesh({RailroadTieMesh(), run.At(i)});
        }
    };
    for (const auto& item : world.roads) road(item);
    for (const auto& [id, item] : world.IntersectionSurfaces()) road(item.geometry);
    for (const auto& mesh : world.meshes) AddMesh(mesh);
    for (const auto& chunk : world.Terrain()) AddMesh(chunk->Instance(), false);
    for (const auto& object : world.Objects())
        if (object->hasCollision && !object->EditorOnly()) AddMesh(object->Instance());
    Finish();
}

void WorldCollision::Finish()
{
    nodes.clear();
    if (!triangles.empty()) BuildNode(0, triangles.size());
    capsuleNodes.clear();
    if (!capsules.empty()) BuildCapsuleNode(0, capsules.size());
    chunksDirty = true; // Include the updated local tree in the aggregate bounds.
}

void WorldCollision::Attach(std::uint64_t key, std::shared_ptr<const WorldCollision> chunk)
{
    if (!chunk) { Detach(key); return; }
    auto [it, inserted] = chunks.try_emplace(key, chunk);
    if (inserted || it->second != chunk)
    {
        it->second = std::move(chunk);
        chunksDirty = true;
    }
}

void WorldCollision::Detach(std::uint64_t key)
{
    if (chunks.erase(key)) chunksDirty = true;
}

void WorldCollision::ClearChunks()
{
    chunks.clear();
    orderedChunks.clear();
    chunkNodes.clear();
    chunksDirty = true;
}

void WorldCollision::IndexChunks() const
{
    if (!chunksDirty) return;
    orderedChunks.clear();
    chunkNodes.clear();
    bounds = nodes.empty() ? Bounds3{} : nodes.front().bounds;
    if (!capsuleNodes.empty())
    { bounds.Include(capsuleNodes.front().bounds.min); bounds.Include(capsuleNodes.front().bounds.max); }
    for (const auto& [key, chunk] : chunks)
    {
        const auto& childBounds = chunk->Bounds();
        if (childBounds.Empty()) continue;
        orderedChunks.push_back({chunk.get(), childBounds});
        bounds.Include(childBounds.min); bounds.Include(childBounds.max);
    }
    if (!orderedChunks.empty()) BuildChunkNode(0, orderedChunks.size());
    chunksDirty = false;
}

size_t WorldCollision::BuildChunkNode(size_t first, size_t count) const
{
    Bounds3 box;
    for (size_t i = first; i < first + count; ++i)
    {
        box.Include(orderedChunks[i].bounds.min);
        box.Include(orderedChunks[i].bounds.max);
    }
    const size_t index = chunkNodes.size();
    chunkNodes.push_back({box, first, count});
    if (count <= 4) return index;
    const Vector3 extent = box.max - box.min;
    int axis = extent.x > extent.y ? 0 : 1;
    if (extent.z > Axis(extent, axis)) axis = 2;
    const size_t middle = first + count / 2;
    std::nth_element(orderedChunks.begin() + first, orderedChunks.begin() + middle, orderedChunks.begin() + first + count,
        [axis](const Chunk& a, const Chunk& b) {
            return Axis(a.bounds.min + a.bounds.max, axis) < Axis(b.bounds.min + b.bounds.max, axis);
        });
    const size_t left = BuildChunkNode(first, middle - first);
    const size_t right = BuildChunkNode(middle, first + count - middle);
    chunkNodes[index].left = left; chunkNodes[index].right = right;
    chunkNodes[index].count = 0;
    return index;
}

void WorldCollision::TraceChunks(size_t index, Vector3 origin, Vector3 direction, bool walkableOnly,
                                 std::optional<Hit>& hit, double& distance) const
{
    const auto& node = chunkNodes[index];
    if (!RayBounds(node.bounds, origin, direction, distance)) return;
    if (!node.count)
    {
        TraceChunks(node.left, origin, direction, walkableOnly, hit, distance);
        TraceChunks(node.right, origin, direction, walkableOnly, hit, distance);
        return;
    }
    for (size_t i = node.first; i < node.first + node.count; ++i)
        if (RayBounds(orderedChunks[i].bounds, origin, direction, distance))
            if (const auto next = orderedChunks[i].collision->Raycast(origin, direction, distance, walkableOnly))
            { hit = next; distance = next->distance; }
}

bool WorldCollision::OverlapChunks(size_t index, Vector3 center, double radius) const
{
    const auto& node = chunkNodes[index];
    if (!SphereBounds(node.bounds, center, radius)) return false;
    if (!node.count) return OverlapChunks(node.left, center, radius) || OverlapChunks(node.right, center, radius);
    for (size_t i = node.first; i < node.first + node.count; ++i)
        if (SphereBounds(orderedChunks[i].bounds, center, radius) &&
            orderedChunks[i].collision->SphereBlocked(center, radius)) return true;
    return false;
}

void WorldCollision::ContactChunks(size_t index, Vector3 center, double radius, std::optional<BodyContact>& result) const
{
    const auto& node = chunkNodes[index];
    if (!SphereBounds(node.bounds, center, radius)) return;
    if (!node.count)
    {
        ContactChunks(node.left, center, radius, result);
        ContactChunks(node.right, center, radius, result);
        return;
    }
    for (size_t i = node.first; i < node.first + node.count; ++i)
        if (SphereBounds(orderedChunks[i].bounds, center, radius))
            if (const auto next = orderedChunks[i].collision->SphereContact(center, radius);
                next && (!result || next->penetration > result->penetration)) result = next;
}

size_t WorldCollision::BuildNode(size_t first, size_t count)
{
    Bounds3 bounds;
    for (size_t i = first; i < first + count; ++i)
    {
        bounds.Include(triangles[i].bounds.min);
        bounds.Include(triangles[i].bounds.max);
    }
    const size_t index = nodes.size();
    nodes.push_back({bounds, first, count});
    if (count <= 12) return index;
    const Vector3 extent = bounds.max - bounds.min;
    int axis = extent.x > extent.y ? 0 : 1;
    if (extent.z > Axis(extent, axis)) axis = 2;
    const size_t middle = first + count / 2;
    std::nth_element(triangles.begin() + first, triangles.begin() + middle, triangles.begin() + first + count,
        [axis](const Triangle& a, const Triangle& b) {
            return Axis(a.bounds.min + a.bounds.max, axis) < Axis(b.bounds.min + b.bounds.max, axis);
        });
    const size_t left = BuildNode(first, middle - first);
    const size_t right = BuildNode(middle, first + count - middle);
    nodes[index].left = left;
    nodes[index].right = right;
    nodes[index].count = 0;
    return index;
}

void WorldCollision::Trace(size_t index, Vector3 origin, Vector3 direction, bool walkableOnly,
                           std::optional<Hit>& hit, double& distance) const
{
    const auto& node = nodes[index];
    if (!RayBounds(node.bounds, origin, direction, distance)) return;
    if (!node.count)
    {
        Trace(node.left, origin, direction, walkableOnly, hit, distance);
        Trace(node.right, origin, direction, walkableOnly, hit, distance);
        return;
    }
    for (size_t i = node.first; i < node.first + node.count; ++i)
    {
        const auto& t = triangles[i];
        if (walkableOnly && t.normal.y < 0.65) continue;
        const Vector3 ab = t.b - t.a, ac = t.c - t.a;
        const Vector3 p = Math::Cross(direction, ac);
        const double determinant = Math::Dot(ab, p);
        if (std::abs(determinant) < 1e-12) continue;
        const Vector3 fromA = origin - t.a;
        const double u = Math::Dot(fromA, p) / determinant;
        const Vector3 q = Math::Cross(fromA, ab);
        const double v = Math::Dot(direction, q) / determinant;
        if (u < -1e-8 || v < -1e-8 || u + v > 1.00000001) continue;
        const double depth = Math::Dot(ac, q) / determinant;
        if (depth >= 0 && depth <= distance)
        {
            distance = depth;
            hit = Hit{origin + direction * depth, t.normal, depth};
        }
    }
}

std::optional<WorldCollision::Hit> WorldCollision::Raycast(Vector3 origin, Vector3 direction, double distance, bool walkableOnly) const
{
    std::optional<Hit> hit;
    if (distance < 0 || Math::Length(direction) <= 0) return hit;
    direction = Math::Normalize(direction);
    if (!nodes.empty()) Trace(0, origin, direction, walkableOnly, hit, distance);
    if (!capsuleNodes.empty()) TraceCapsules(0, origin, direction, walkableOnly, hit, distance);
    IndexChunks();
    if (!chunkNodes.empty()) TraceChunks(0, origin, direction, walkableOnly, hit, distance);
    return hit;
}

std::optional<WorldCollision::Hit> WorldCollision::Ground(Vector3 feet, double above, double below) const
{
    return Raycast(feet + Vector3{0, above, 0}, {0, -1, 0}, above + below, true);
}

bool WorldCollision::Overlap(size_t index, Vector3 center, double radius) const
{
    const auto& node = nodes[index];
    const Vector3 nearest{std::clamp(center.x, node.bounds.min.x, node.bounds.max.x),
                          std::clamp(center.y, node.bounds.min.y, node.bounds.max.y),
                          std::clamp(center.z, node.bounds.min.z, node.bounds.max.z)};
    if (Math::Length(center - nearest) >= radius) return false;
    if (!node.count) return Overlap(node.left, center, radius) || Overlap(node.right, center, radius);
    for (size_t i = node.first; i < node.first + node.count; ++i)
    {
        const auto& t = triangles[i];
        const Vector3 closest = ClosestPoint(center, t.a, t.b, t.c);
        // Ground contact is resolved by the downward support query, including slopes.
        if (t.normal.y >= 0.65)
        {
            const double groundY = t.a.y - (t.normal.x * (center.x - t.a.x) + t.normal.z * (center.z - t.a.z)) / t.normal.y;
            if (groundY <= center.y - radius + 0.06) continue;
        }
        if (Math::Length(center - closest) < radius - 0.005) return true;
    }
    return false;
}

bool WorldCollision::SphereBlocked(Vector3 center, double radius) const
{
    if (!nodes.empty() && Overlap(0, center, radius)) return true;
    std::optional<BodyContact> contact;
    if (!capsuleNodes.empty()) ContactCapsules(0, center, radius, true, contact);
    if (contact) return true;
    IndexChunks();
    return !chunkNodes.empty() && OverlapChunks(0, center, radius);
}

void WorldCollision::Contact(size_t index, Vector3 center, double radius, std::optional<BodyContact>& result) const
{
    const auto& node = nodes[index];
    const Vector3 nearest{std::clamp(center.x, node.bounds.min.x, node.bounds.max.x),
                          std::clamp(center.y, node.bounds.min.y, node.bounds.max.y),
                          std::clamp(center.z, node.bounds.min.z, node.bounds.max.z)};
    if (Math::Length(center - nearest) >= radius) return;
    if (!node.count)
    {
        Contact(node.left, center, radius, result);
        Contact(node.right, center, radius, result);
        return;
    }
    for (size_t i = node.first; i < node.first + node.count; ++i)
    {
        const auto& triangle = triangles[i];
        const Vector3 point = ClosestPoint(center, triangle.a, triangle.b, triangle.c);
        const Vector3 offset = center - point;
        const double distance = Math::Length(offset);
        const double penetration = radius - distance;
        if (penetration > 0 && (!result || penetration > result->penetration))
            result = BodyContact{point, distance > 1e-10 ? offset * (1.0 / distance) : triangle.normal, penetration};
    }
}

std::optional<BodyContact> WorldCollision::SphereContact(Vector3 center, double radius) const
{
    std::optional<BodyContact> contact;
    if (!nodes.empty()) Contact(0, center, radius, contact);
    if (!capsuleNodes.empty()) ContactCapsules(0, center, radius, false, contact);
    IndexChunks();
    if (!chunkNodes.empty()) ContactChunks(0, center, radius, contact);
    return contact;
}

std::size_t WorldCollision::MemoryBytes() const
{
    return sizeof(*this) + triangles.capacity() * sizeof(Triangle) + nodes.capacity() * sizeof(Node) +
        capsules.capacity() * sizeof(CollisionCapsule) + capsuleNodes.capacity() * sizeof(Node) +
        orderedChunks.capacity() * sizeof(Chunk) + chunkNodes.capacity() * sizeof(Node);
}
const Bounds3& WorldCollision::Bounds() const
{
    IndexChunks();
    return bounds;
}

size_t WorldCollision::BuildCapsuleNode(size_t first, size_t count)
{
    Bounds3 box;
    for (size_t i = first; i < first + count; ++i)
    { const auto b = capsules[i].Bounds(); box.Include(b.min); box.Include(b.max); }
    const size_t index = capsuleNodes.size();
    capsuleNodes.push_back({box, first, count});
    if (count <= 8) return index;
    const auto extent = box.max - box.min;
    int axis = extent.x > extent.y ? 0 : 1;
    if (extent.z > Axis(extent, axis)) axis = 2;
    const size_t middle = first + count / 2;
    std::nth_element(capsules.begin() + first, capsules.begin() + middle, capsules.begin() + first + count,
        [axis](const auto& a, const auto& b) { return Axis(a.a + a.b, axis) < Axis(b.a + b.b, axis); });
    const auto left = BuildCapsuleNode(first, middle - first), right = BuildCapsuleNode(middle, first + count - middle);
    capsuleNodes[index].left = left; capsuleNodes[index].right = right; capsuleNodes[index].count = 0;
    return index;
}

void WorldCollision::TraceCapsules(size_t index, Vector3 origin, Vector3 direction, bool walkableOnly,
                                   std::optional<Hit>& hit, double& distance) const
{
    const auto& node = capsuleNodes[index];
    if (!RayBounds(node.bounds, origin, direction, distance)) return;
    if (!node.count)
    {
        TraceCapsules(node.left, origin, direction, walkableOnly, hit, distance);
        TraceCapsules(node.right, origin, direction, walkableOnly, hit, distance);
        return;
    }
    for (size_t i = node.first; i < node.first + node.count; ++i)
        if (const auto next = capsules[i].Raycast(origin, direction, distance, walkableOnly))
        { distance = next->distance; hit = Hit{origin + direction * distance, next->normal, distance}; }
}

void WorldCollision::ContactCapsules(size_t index, Vector3 center, double radius, bool blocking,
                                     std::optional<BodyContact>& result) const
{
    const auto& node = capsuleNodes[index];
    if (!SphereBounds(node.bounds, center, radius)) return;
    if (!node.count)
    {
        ContactCapsules(node.left, center, radius, blocking, result);
        ContactCapsules(node.right, center, radius, blocking, result);
        return;
    }
    for (size_t i = node.first; i < node.first + node.count; ++i)
    {
        const auto& capsule = capsules[i];
        const auto nearest = capsule.ClosestAxisPoint(center), offset = center - nearest;
        const double distance = Math::Length(offset), penetration = radius + capsule.radius - distance;
        if (penetration <= (blocking ? .005 : 0)) continue;
        auto normal = distance > 1e-10 ? offset * (1 / distance) : Vector3{1, 0, 0};
        if (distance <= 1e-10 && Math::Length(capsule.b - capsule.a) > 1e-10)
        {
            const auto axis = Math::Normalize(capsule.b - capsule.a);
            normal = Math::Normalize(Math::Cross(axis, std::abs(axis.x) < .9 ? Vector3{1, 0, 0} : Vector3{0, 1, 0}));
        }
        const auto point = nearest + normal * capsule.radius;
        if (blocking && normal.y >= .65 && point.y <= center.y - radius + .06) continue;
        if (!result || penetration > result->penetration) result = BodyContact{point, normal, penetration};
    }
}
