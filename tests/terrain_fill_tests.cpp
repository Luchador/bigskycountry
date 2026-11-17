#include "terrain_fill.hpp"
#include "world_package.hpp"
#include "world_collision.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
void Check(bool ok, const char* message)
{ if (!ok) throw std::runtime_error(std::string(message) + ": " + SDL_GetError()); }

double Area(const Mesh& mesh)
{
    double area = 0;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        const auto a = mesh.vertices[mesh.indices[i]].position, b = mesh.vertices[mesh.indices[i+1]].position,
                   c = mesh.vertices[mesh.indices[i+2]].position;
        area += std::abs(Math::Cross(b - a, c - a).y) * .5;
    }
    return area;
}

std::vector<Vector3> Rim(const TerrainChunk& lake)
{
    std::vector<Vector3> result;
    for (const auto& sample : lake.BuiltShoreline()) result.push_back(lake.Water().transform.Point(sample.position));
    return result;
}
double Area(const TerrainChunk& lake)
{
    const auto rim = Rim(lake);
    double area = 0;
    for (std::size_t i = 1; i < rim.size(); ++i)
        area += Math::Cross(rim[i-1] - rim[0], rim[i] - rim[0]).y;
    return std::abs(area) * .5;
}

// Compare actual boundary edges, not just approximate positions or bounds.
// Every shared edge must have exactly one land face and one existing basin face.
void Sealed(const Mesh& fill, const TerrainChunk& lake)
{
    using Point = std::array<double, 3>;
    using Edge = std::pair<Point, Point>;
    const auto point = [](Vector3 p) { return Point{p.x, p.y, p.z}; };
    const auto edges = [&](const Mesh& mesh, const MeshTransform& transform) {
        std::map<Edge, unsigned> result;
        for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
            for (int j = 0; j < 3; ++j)
            {
                const auto a = point(transform.Point(mesh.vertices[mesh.indices[i+j]].position));
                const auto b = point(transform.Point(mesh.vertices[mesh.indices[i+(j+1)%3]].position));
                ++result[std::minmax(a, b)];
            }
        return result;
    };
    const auto land = edges(fill, {}), bed = edges(*lake.Instance().mesh, lake.Instance().transform);
    const auto rim = Rim(lake);
    for (std::size_t i = 1; i < rim.size(); ++i)
    {
        const auto a = point(rim[i-1]), b = point(rim[i]);
        const Edge edge = std::minmax(a, b);
        Check(land.contains(edge) && land.at(edge) == 1, "Terrain did not preserve an exact shoreline edge");
        Check(bed.contains(edge) && bed.at(edge) == 1, "Lake basin did not share the same edge");
    }
}

TerrainBoundary Perimeter(WorldDocument& world, Vector3 origin = {})
{
    const std::array<Vector3, 4> corners{{{-100,10,-100}, {100,10,-100}, {100,10,100}, {-100,10,100}}};
    TerrainBoundary boundary;
    for (int i = 0; i < 4; ++i)
    {
        Road road; road.id = world.AllocateRoadId(); road.style.curved = false;
        road.points = {{origin + corners[i]}, {origin + corners[(i+1)%4]}};
        road.Rebuild(); boundary.roads.insert(road.id); world.roads.push_back(std::move(road));
    }
    return boundary;
}

void GeometryAndUpdates()
{
    WorldDocument world(AssetRoot::Project);
    const Vector3 origin{500000, 0, -300000};
    auto* fill = world.PlaceTerrainFill(Perimeter(world, origin));
    Check(fill && fill->Instance().mesh, "Create road-bounded terrain");
    const auto original = fill->Instance().mesh;
    Check(original->vertices.size() > 20, "Fill has no interior subdivision");
    auto* lake = world.PlaceLake(LakeDefinition::Circle(60), ObjectTransform{origin + Vector3{0,3,0}});
    Check(lake, "Place enclosed lake");
    const auto bed = lake->Instance().mesh;
    world.RefreshTerrainFills();
    Check(fill->Instance().mesh && fill->Instance().mesh != original, "New lake did not update an existing fill");
    Sealed(*fill->Instance().mesh, *lake);
    Check(std::abs(Area(*fill->Instance().mesh) + Area(*lake) - Area(*original)) < 1e-5, "Fill covered the lake or lost surrounding land");
    Check(lake->Instance().mesh == bed, "Fill rebuilt the lake's own bottom");
    const auto unchanged = fill->Instance().mesh;
    for (int i = 0; i < 10; ++i) world.RefreshTerrainFills();
    Check(fill->Instance().mesh == unchanged, "Unchanged fill was rebuilt");

    // Keep expensive fill triangulation out of the shoreline editing loop.
    Check(lake->BeginLakeEdit(), "Begin lake edit");
    auto shape = *lake->Lake(); Check(shape.MovePoint(0, {45,0,0}), "Reshape lake");
    Check(lake->SetLake(shape), "Preview lake"); world.RefreshTerrainFills();
    Check(fill->Instance().mesh == unchanged, "Draft edit rebuilt terrain");
    lake->CancelLakeEdit(); world.RefreshTerrainFills();
    Check(fill->Instance().mesh == unchanged, "Canceled draft rebuilt terrain");
    Check(lake->BeginLakeEdit() && lake->SetLake(shape) && lake->FinishLakeEdit(), "Commit lake edit");
    world.RefreshTerrainFills(); Check(fill->Instance().mesh != unchanged, "Finished lake did not update fill");
    Sealed(*fill->Instance().mesh, *lake);
    const auto edited = fill->Instance().mesh;
    Check(lake->BeginLakeEdit(), "Begin invalid lake edit");
    auto invalid = *lake->Lake(); invalid.points[0] = invalid.points[2];
    Check(lake->SetLake(invalid) && !lake->FinishLakeEdit(), "Accept preview but reject self-crossing lake");
    world.RefreshTerrainFills(); Check(fill->Instance().mesh == edited, "Invalid lake changed terrain");
    lake->CancelLakeEdit();

    auto concave = LakeDefinition::Circle(40); Check(concave.MovePoint(0, {-5,0,0}), "Make concave shore");
    auto* second = world.PlaceLake(concave, ObjectTransform{origin + Vector3{-65,8,0}, {0,.25,0}});
    Check(second, "Place second, concave lake"); world.RefreshTerrainFills();
    Check(fill->Instance().mesh != edited, "Second lake omitted");
    Sealed(*fill->Instance().mesh, *lake); Sealed(*fill->Instance().mesh, *second);
    Check(std::abs(Area(*fill->Instance().mesh) + Area(*lake) + Area(*second) - Area(*original)) < 1e-5, "Multiple lake holes have incorrect area");
    // Outliner positions are not dependency identities.
    const auto twoLakes = fill->Instance().mesh;
    world.MoveLastTerrainTo(1); world.RefreshTerrainFills();
    Check(fill->Instance().mesh == twoLakes, "Reordering lake rows rebuilt terrain");

    auto* distant = world.PlaceLake(LakeDefinition::Circle(40), ObjectTransform{origin + Vector3{10000,0,0}});
    Check(distant, "Place distant lake"); world.RefreshTerrainFills();
    Check(fill->Instance().mesh == twoLakes, "Distant lake invalidated fill");
    Check(distant->SetTransform(ObjectTransform{origin + Vector3{10010,10,0}}), "Move distant lake");
    world.RefreshTerrainFills(); Check(fill->Instance().mesh == twoLakes, "Distant lake movement invalidated fill");
    Check(world.RemoveTerrain(world.Terrain().size()-1), "Delete distant lake");
    world.RefreshTerrainFills(); Check(fill->Instance().mesh == twoLakes, "Distant lake deletion invalidated fill");
    Check(lake->SetTransform(ObjectTransform{origin + Vector3{15,6,10}, {0,.4,0}}), "Translate and rotate lake");
    world.RefreshTerrainFills(); Sealed(*fill->Instance().mesh, *lake);
    Check(world.RemoveTerrain(2) && world.RemoveTerrain(1), "Delete both lakes");
    world.RefreshTerrainFills();
    Check(fill->Instance().mesh && std::abs(Area(*fill->Instance().mesh) - Area(*original)) < 1e-5 && fill->sourceLakes.empty(), "Deleted lakes left stale holes/dependencies");
    auto* restored = world.PlaceLake(shape, ObjectTransform{origin + Vector3{0,3,0}});
    Check(restored, "Restore deleted lake"); world.RefreshTerrainFills(); Sealed(*fill->Instance().mesh, *restored);
    const auto beforeRoadEdit = fill->Instance().mesh;
    for (auto& point : world.roads[0].points) point.position.z -= 10;
    // Keep the neighboring road ends joined as well.
    world.roads[1].points.front().position = world.roads[0].points.back().position;
    world.roads[3].points.back().position = world.roads[0].points.front().position;
    for (auto& road : world.roads) road.Rebuild();
    world.RefreshTerrainFills();
    Check(fill->Instance().mesh && fill->Instance().mesh != beforeRoadEdit, "Road edit no longer updated fill");
    Sealed(*fill->Instance().mesh, *restored);
}

void PersistenceAndCooking(const std::filesystem::path& root)
{
    const auto source = (root / "lake-fill.wld").string(), cooked = (root / "lake-fill.bscworld").string();
    WorldDocument world(AssetRoot::Project); Check(world.Open(source.c_str()), "Open world");
    auto* fill = world.PlaceTerrainFill(Perimeter(world, {512,0,512})); Check(fill, "Place fill before lake");
    auto* lake = world.PlaceLake(LakeDefinition::Circle(80), ObjectTransform{{512,2,512}}); Check(lake, "Place lake");
    world.RefreshTerrainFills(); Sealed(*fill->Instance().mesh, *lake);
    const auto fillId = fill->persistentId, lakeId = lake->persistentId;
    Check(world.Save(), "Save world");
    WorldDocument loaded(AssetRoot::Project); Check(loaded.Open(source.c_str()), "Reload fill before lake");
    Check(loaded.Terrain()[0]->persistentId == fillId && loaded.Terrain()[1]->persistentId == lakeId, "Stable IDs changed");
    Sealed(*loaded.Terrain()[0]->Instance().mesh, *loaded.Terrain()[1]);
    Check(WorldPackage::Cook(loaded, cooked), "Cook lake fill");
    WorldPackage::Reader reader; Check(reader.Open(cooked), "Open cooked world");
    for (const auto& entity : reader.Index().entities)
        if (entity.id == EntityId{SceneKind::Terrain, fillId} || entity.id == EntityId{SceneKind::Terrain, lakeId})
            Check(std::ranges::find(entity.dependencies, EntityId{SceneKind::Terrain, entity.id.value == fillId ? lakeId : fillId}) != entity.dependencies.end(), "Missing cooked lake/fill dependency");
    double nearArea = 0, farArea = 0;
    WorldCollision collision;
    for (std::size_t i = 0; i < reader.Index().cells.size(); ++i)
    {
        WorldPackage::Cell cell; Check(reader.ReadCell(i, cell), "Read cooked cell");
        MeshInstance collisionMesh; Check(reader.ReadCollision(i, collisionMesh), "Read collision cell");
        collision.AddMesh(collisionMesh);
        for (const auto& batch : cell.batches)
            if (batch.owner == EntityId{SceneKind::Terrain, fillId})
            {
                nearArea += Area(*batch.nearMesh.mesh); farArea += Area(*batch.distantMesh.mesh);
            }
    }
    const double expected = Area(*loaded.Terrain()[0]->Instance().mesh);
    Check(std::abs(nearArea - expected) < 1e-5 && std::abs(farArea - expected) < 1e-5, "Cooked near/distant meshes filled the lake hole");
    collision.Finish();
    const auto floor = collision.Raycast({512,100,512}, {0,-1,0}, 200);
    Check(floor && std::abs(floor->position.y + 3) < 1e-5, "Terrain collision covered the lake's own bottom");
}
}

int main()
{
    SDL_Time now; SDL_GetCurrentTime(&now);
    const auto root = std::filesystem::temp_directory_path() / ("bigsky-lake-fill-" + std::to_string(now));
    try
    {
        std::filesystem::create_directories(root);
        GeometryAndUpdates(); PersistenceAndCooking(root);
        std::filesystem::remove_all(root); std::cout << "Lake terrain fill tests passed\n"; return 0;
    }
    catch (const std::exception& e) { std::cerr << e.what() << "\nFixture: " << root << '\n'; return 1; }
}
