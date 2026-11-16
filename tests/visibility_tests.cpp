#include "spatial_index.hpp"
#include "world_visibility.hpp"
#include "world_document.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>
#include <set>

namespace
{
    void Check(bool condition, const char* message)
    {
        if (!condition) { std::cerr << message << '\n'; std::exit(1); }
    }

    double ReferenceDistance(const Bounds3& b, Vector3 p)
    {
        if (b.Empty()) return 0;
        const auto nearest = Vector3{std::clamp(p.x, b.min.x, b.max.x),
                                     std::clamp(p.y, b.min.y, b.max.y),
                                     std::clamp(p.z, b.min.z, b.max.z)};
        const auto d = nearest - p;
        return Math::Dot(d, d);
    }

    void Compare(SpatialIndex& index, const std::vector<Bounds3>& bounds, Vector3 eye, double radius)
    {
        std::set<SceneKey> expected;
        for (int i = 0; i < static_cast<int>(bounds.size()); ++i)
            if (ReferenceDistance(bounds[i], eye) <= radius * radius) expected.insert({SceneKind::Road, i});
        const auto result = index.Query(eye, radius);
        Check(std::set<SceneKey>(result.begin(), result.end()) == expected, "Grid query differs from brute-force bounds test");
        Check(result.size() == expected.size(), "Multi-cell geometry was returned twice");
    }

    std::shared_ptr<Mesh> Box(Bounds3 bounds)
    {
        auto mesh = std::make_shared<Mesh>();
        mesh->bounds = bounds;
        return mesh;
    }
}

int main()
{
    SpatialIndex index;
    std::vector<Bounds3> bounds{
        {{-512, -1, -512}, {0, 1, 0}}, // Negative coordinates and exact cell edges.
        {{512, 0, 0}, {512, 0, 0}},
        {{-40000, 10, -10}, {40000, 12, 10}}, // Nearby middle, distant center/endpoints.
        {{-300000, 0, -300000}, {300000, 1, 300000}}, // Oversized fallback.
        {{0, 10000, 0}, {1, 10001, 1}}, // Altitude matters.
        {} // Unknown bounds remain conservative.
    };
    std::mt19937_64 random(1234);
    std::uniform_real_distribution<double> coordinate(-1e6, 1e6), extent(0, 2000);
    for (int i = 0; i < 4000; ++i)
    {
        const Vector3 p{coordinate(random), coordinate(random) * 0.01, coordinate(random)};
        const Vector3 size{extent(random), extent(random), extent(random)};
        bounds.push_back({p, p + size});
    }
    const auto sync = [&] {
        index.BeginSync();
        for (int i = 0; i < static_cast<int>(bounds.size()); ++i) index.Upsert({SceneKind::Road, i}, bounds[i]);
        index.EndSync();
    };
    sync();
    Check(index.Stats().oversized >= 1, "Oversized geometry must not expand the grid without bound");
    sync();
    Check(index.Stats().updated == 0, "Idle frame reindexed unchanged geometry");
    for (double radius : {0.1, 100.0, 512.0, 8000.0, 128000.0})
    {
        Compare(index, bounds, {}, radius);
        Compare(index, bounds, {-512, 0, 0}, radius);
        Compare(index, bounds, {40000, 10, 0}, radius);
        for (int j = 0; j < 100; ++j)
            Compare(index, bounds, {coordinate(random), coordinate(random) * 0.01, coordinate(random)}, radius);
    }
    bounds[0] = {{100000, 0, 100000}, {100001, 1, 100001}};
    bounds.pop_back();
    sync();
    Check(index.Stats().updated == 2, "Move/deletion must update only changed entries");
    Compare(index, bounds, {}, 8000);
    Compare(index, bounds, {100000, 0, 100000}, 100);
    index.BeginSync(); index.EndSync();
    Check(index.Query({}, 8000).empty() && index.Stats().cells == 0, "Removed world left stale cells");

    // Arbitrary affine placements, including shear and a negative scale.
    const Bounds3 local{{-1, -2, -3}, {4, 5, 6}};
    const MeshTransform transform{{-700000, 100, 400000}, {-2, 0, 0}, {1, 3, 0}, {0, 1, 2}};
    const auto transformed = WorldBounds(local, transform);
    for (double x : {-1., 0., 4.}) for (double y : {-2., 0., 5.}) for (double z : {-3., 0., 6.})
        Check(BoundsDistanceSquared(transformed, transform.Point({x, y, z})) == 0, "Affine bounds excluded a mesh point");

    // Direct edits, index compaction and same-size replacement must not depend
    // on explicit invalidation calls from every editor mutation/undo path.
    WorldDocument world;
    world.meshes.push_back({Box({{-1, -1, -1}, {1, 1, 1}}), {}});
    world.meshes.push_back({Box({{-1, -1, -1}, {1, 1, 1}}), {{20000, 0, 0}}});
    WorldVisibility visibility;
    visibility.Synchronize(world);
    visibility.Synchronize(world);
    Check(visibility.Index().Stats().updated == 0, "Idle mesh bounds recalculated");
    world.meshes.erase(world.meshes.begin());
    visibility.Synchronize(world);
    Check(visibility.Index().Stats().entries == 1, "Compacted mesh list retained deleted entry");
    Check(visibility.Index().Bounds({SceneKind::Mesh, 0}).min.x == 19999, "Compacted index used old placement");
    world.meshes[0].transform.origin = {};
    visibility.Synchronize(world);
    Check(visibility.Index().Bounds({SceneKind::Mesh, 0}).min.x == -1, "Edited transform did not reindex");
    world.meshes[0].mesh = Box({{-2000, -1, -1}, {2000, 1, 1}});
    visibility.Synchronize(world);
    Check(visibility.Index().Bounds({SceneKind::Mesh, 0}).min.x == -2000, "Rebuilt geometry did not reindex");
    std::cout << "Visibility regression checks passed\n";
}
