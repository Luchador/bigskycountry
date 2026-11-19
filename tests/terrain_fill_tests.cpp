#include "terrain_fill.hpp"
#include "world_package.hpp"
#include "world_collision.hpp"
#include "road_io.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
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

RoadGeometry Rectangle(double left, double bottom, double right, double top)
{
    RoadGeometry geometry;
    geometry.outline = {{left,0,bottom}, {right,0,bottom}, {right,0,top}, {left,0,top}};
    return geometry;
}

bool Covers(const Mesh& mesh, Vector3 point)
{
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        const auto a = mesh.vertices[mesh.indices[i]].position, b = mesh.vertices[mesh.indices[i+1]].position,
                   c = mesh.vertices[mesh.indices[i+2]].position;
        if (std::abs(Math::Cross(b-a,c-a).y) < 1e-10) continue;
        const std::array turns{Math::Cross(b-a,point-a).y, Math::Cross(c-b,point-b).y, Math::Cross(a-c,point-c).y};
        if (*std::min_element(turns.begin(), turns.end()) >= -1e-8 ||
            *std::max_element(turns.begin(), turns.end()) <= 1e-8) return true;
    }
    return false;
}

void IncidentalPockets()
{
    // Scandia's layout: two tracks cross; a road crosses both tracks before
    // curving around the block. Selecting the whole perimeter also encloses
    // land on the far side of the east/west track, using only three sources.
    auto west = Rectangle(-2,-100,0,40), north = Rectangle(-10,30,110,32),
         east = Rectangle(100,20,102,40), track = Rectangle(-10,0,110,2);
    RoadGeometry curve;
    curve.outline = {{-10,0,-82}, {40,0,-82}, {70,0,18}, {110,0,18},
                     {110,0,22}, {70,0,22}, {40,0,-78}, {-10,0,-78}};
    std::array<const RoadGeometry*,5> perimeter{&west,&north,&east,&track,&curve};
    const auto selected = BuildTerrainFill(perimeter, {}, {}, {}, TerrainFillScope::SelectedBoundary);
    Check(selected && Covers(*selected, {10,0,10}), "Selected perimeter did not fill the intended block");
    Check(!Covers(*selected, {10,0,-10}), "Selected perimeter also filled an incidental crossing pocket");
    const auto all = BuildTerrainFill(perimeter, {});
    Check(all && Covers(*all, {10,0,10}) && Covers(*all, {10,0,-10}), "Geometry helpers lost all-pocket filling");
    const std::array<const RoadGeometry*,3> smallBoundary{&west,&track,&curve};
    const auto separate = BuildTerrainFill(smallBoundary, {}, {}, {}, TerrainFillScope::SelectedBoundary);
    Check(separate && Covers(*separate, {10,0,-10}), "Incidental pocket cannot be filled separately");
    Check(Area(*separate) > Area(*selected), "Fixture must reject the larger incidental pocket, not pick by area");
    Check(std::abs(Area(*all)-Area(*separate)-Area(*selected)) < 1e-6, "Pocket filtering changed the retained land area");
    std::reverse(perimeter.begin(),perimeter.end());
    const auto reversed = BuildTerrainFill(perimeter, {}, {}, {}, TerrainFillScope::SelectedBoundary);
    Check(reversed && std::abs(Area(*reversed)-Area(*selected)) < 1e-6 && !Covers(*reversed,{10,0,-10}),
          "Selection order changed the chosen pocket");

    // An unsealed larger source set must not suppress a valid smaller loop.
    for (auto& p : north.outline) p.y = 5;
    const auto unsealed = BuildTerrainFill(perimeter, {}, {}, {}, TerrainFillScope::SelectedBoundary);
    Check(unsealed && Covers(*unsealed,{10,0,-10}) && !Covers(*unsealed,{10,0,10}),
          "Invalid elevation seams suppressed a valid pocket");
}

void MultipleBlocks()
{
    auto west = Rectangle(-2,-10,0,110), middle = Rectangle(50,-10,52,110),
         east = Rectangle(100,-10,102,110), north = Rectangle(-10,100,110,102),
         south = Rectangle(-10,-2,110,0);
    const std::array<const RoadGeometry*,5> perimeter{&west,&middle,&east,&north,&south};
    const auto fill = BuildTerrainFill(perimeter, {}, {}, {}, TerrainFillScope::SelectedBoundary);
    Check(fill && Covers(*fill,{25,0,50}) && Covers(*fill,{75,0,50}) && std::abs(Area(*fill)-9800) < 1e-6,
          "Independent adjacent blocks no longer fill together");

    // Multiple pieces from one road (e.g. bridge ground contacts) still have
    // one owner. Equal source sets do not arbitrarily discard either block.
    RoadGeometry sides;
    for (const auto* side : {&west,&middle,&east}) sides.groundFootprints.emplace_back().outline = side->outline;
    const std::array<const RoadGeometry*,3> equalPerimeter{&sides,&north,&south};
    const auto equal = BuildTerrainFill(equalPerimeter, {}, {}, {}, TerrainFillScope::SelectedBoundary);
    Check(equal && Covers(*equal,{25,0,50}) && Covers(*equal,{75,0,50}) && std::abs(Area(*equal)-9800) < 1e-6,
          "Pockets sharing the same sources lost one of their blocks");
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

void LegacyFill(const std::filesystem::path& root)
{
    WorldDocument source(AssetRoot::Project);
    const auto boundary = Perimeter(source);
    const auto reference = source.PlaceTerrainFill(boundary);
    Check(reference && reference->Instance().mesh, "Build legacy fill reference");
    std::ostringstream roads, terrain;
    for (const auto& road : source.roads) RoadIO::WriteRoad(roads,road);
    terrain << "fill \"\" " << boundary.roads.size();
    for (int id : boundary.roads) terrain << ' ' << id;
    terrain << " 0 0\n"; // TERR v8: no pocket-selection flag.
    const auto path = (root / "legacy-fill.wld").string();
    auto* file = SDL_IOFromFile(path.c_str(), "wb"); Check(file, "Create legacy world");
    const auto section = [&](const char* tag, Uint32 version, const std::string& payload)
    {
        return SDL_WriteIO(file,tag,4) == 4 && SDL_WriteU32LE(file,version) && SDL_WriteU64LE(file,payload.size()) &&
               SDL_WriteIO(file,payload.data(),payload.size()) == payload.size();
    };
    const bool written = SDL_WriteIO(file,"BSCW",4) == 4 && SDL_WriteU32LE(file,1) && SDL_WriteU32LE(file,2) &&
                         section("ROAD",RoadIO::currentVersion,roads.str()) && section("TERR",8,terrain.str());
    const bool closed = SDL_CloseIO(file); Check(written && closed, "Write legacy world");
    WorldDocument loaded(AssetRoot::Project); Check(loaded.Open(path.c_str()), "Open TERR v8");
    const auto& old = loaded.Terrain().front();
    Check(old->boundary && !old->boundary->filterIncidentalPockets && old->boundary->SameSources(boundary),
          "Legacy fill coverage or source matching changed");
    Check(old->Instance().mesh && std::abs(Area(*old->Instance().mesh)-Area(*reference->Instance().mesh)) < 1e-6,
          "Legacy fill geometry changed");
    auto* added = loaded.PlaceTerrainFill(Perimeter(loaded, {500,0,0}));
    Check(added && added->boundary->filterIncidentalPockets, "New fill did not enable pocket filtering");
    Check(loaded.Save(), "Save old and new fill modes");
    WorldDocument reopened(AssetRoot::Project); Check(reopened.Open(path.c_str()), "Reload TERR v9");
    Check(reopened.Terrain().size() == 2 && !reopened.Terrain()[0]->boundary->filterIncidentalPockets &&
          reopened.Terrain()[1]->boundary->filterIncidentalPockets, "Fill modes were not preserved by save/reload");
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
    Check(fill->Instance().mesh && std::abs(Area(*fill->Instance().mesh) - Area(*original)) < 1e-5 && fill->sourceWaterBodies.empty(), "Deleted lakes left stale holes/dependencies");
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
    Check(loaded.Terrain()[0]->boundary->filterIncidentalPockets, "New fill mode lost on reload");
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
        IncidentalPockets(); MultipleBlocks(); LegacyFill(root); GeometryAndUpdates(); PersistenceAndCooking(root);
        std::filesystem::remove_all(root); std::cout << "Terrain fill tests passed\n"; return 0;
    }
    catch (const std::exception& e) { std::cerr << e.what() << "\nFixture: " << root << '\n'; return 1; }
}
