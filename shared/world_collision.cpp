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

void WorldCollision::AddMesh(const MeshInstance& instance)
{
    if (!instance.mesh) return;
    const auto& mesh = *instance.mesh;
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
    triangles.clear();
    nodes.clear();
    const auto road = [&](const RoadGeometry& geometry) {
        const auto surface = [&](const auto& faces) {
            for (const auto& t : faces) AddTriangle(t[0].position, t[1].position, t[2].position);
        };
        surface(geometry.pavementTriangles);
        surface(geometry.edgeTriangles);
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
    world.ForEachMesh([&](const MeshInstance& mesh) { AddMesh(mesh); });
    Finish();
}

void WorldCollision::Finish()
{
    nodes.clear();
    if (!triangles.empty()) BuildNode(0, triangles.size());
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
    if (!nodes.empty() && distance >= 0 && Math::Length(direction) > 0)
        Trace(0, origin, Math::Normalize(direction), walkableOnly, hit, distance);
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
    return !nodes.empty() && Overlap(0, center, radius);
}
