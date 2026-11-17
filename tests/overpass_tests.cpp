#include "world_package.hpp"
#include "world_collision.hpp"
#include "terrain_fill.hpp"
#include "road_io.hpp"
#include "../editor/edit_history.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace
{
void Check(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(std::string(message) + ": " + SDL_GetError());
}

Road Straight(WorldDocument& world, Vector3 a, Vector3 b, bool railway = false)
{
    Road road;
    road.id = world.AllocateRoadId();
    road.style.curved = false;
    if (railway) road.style.railroad.emplace();
    road.points = {{a}, {b}};
    road.Rebuild();
    return road;
}

double GroundArea(const Road& road)
{
    double area = 0;
    for (const auto& t : road.bridgeTriangles)
    {
        const auto a = t[0].position, b = t[1].position, c = t[2].position;
        // Ground faces point up; the deck underside points down and walls are vertical.
        const double normalY = Math::Cross(b - a, c - a).y;
        if (normalY > 0) area += normalY * .5;
    }
    return area;
}

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

std::vector<Vector3> Shore(const TerrainChunk& water)
{
    std::vector<Vector3> rim;
    for (const auto& p : water.BuiltShoreline()) rim.push_back(water.Water().transform.Point(p.position));
    return rim;
}

// Exercise a long highway crossing all three source kinds, including a namespace
// collision between water ID 1 and highway ID 1 and large world coordinates.
void Populate(WorldDocument& world, Vector3 origin = {})
{
    auto upper = Straight(world, origin + Vector3{-120,12,0}, origin + Vector3{120,12,0});
    Check(upper.SubdivideSegments({0}), "Subdivide highway span");
    world.roads.push_back(std::move(upper));
    world.roads.push_back(Straight(world, origin + Vector3{10,0,-100}, origin + Vector3{10,0,100}, true));
    world.roads.push_back(Straight(world, origin + Vector3{70,0,-100}, origin + Vector3{70,0,100}));
    Check(world.PlaceRiver(RiverDefinition::Straight(), ObjectTransform{origin + Vector3{-60,0,-100}}), "Place river");
}

BridgeSettings Install(WorldDocument& world)
{
    auto settings = PlanOverpass(world.roads[0], {0,1}, world.roads, world.Terrain());
    Check(settings.has_value(), "Plan combined overpass");
    Check(settings->lowerRoads == std::set<int>{world.roads[1].id, world.roads[2].id}, "Missed a road or railroad");
    Check(settings->lowerWaterBodies == std::set<std::uint64_t>{world.Terrain()[0]->persistentId}, "Missed river identity");
    for (int i : {0,1}) world.roads[0].points[i].bridge = settings;
    world.roads[0].Rebuild();
    world.RefreshTerrainFills();
    return *settings;
}

void GeometryAndUpdates()
{
    WorldDocument world(AssetRoot::Project);
    const Vector3 origin{500000,0,-300000};
    Populate(world, origin);
    auto* river = world.Terrain()[0].get();
    const auto riverBed = river->Instance().mesh, riverWater = river->Water().mesh;
    const auto railRevision = world.roads[1].revision, roadRevision = world.roads[2].revision;
    Install(world);
    auto& bridge = world.roads[0];
    Check(world.roads[1].revision == railRevision && world.roads[2].revision == roadRevision &&
          river->Instance().mesh == riverBed && river->Water().mesh == riverWater, "Bridge altered lower elements");
    Check(bridge.groundFootprints.size() == 1, "Selected segments did not form one continuous span");
    for (const auto& wall : bridge.groundFootprints[0].walls)
        Check(std::abs(std::abs(wall[0].x - origin.x) - 120) < 1e-6, "Internal abutment blocks lower elements");
    const double railWidth = world.roads[1].style.railroad->ToeHalfWidth() * 2;
    const double expected = (240 - 25 - railWidth - 10) * 10;
    Check(std::abs(GroundArea(bridge) - expected) < 1e-4, "Underpass ground has holes or covers lower elements");

    // Fill around the bridge. Its own ground and the adjacent terrain must cover
    // the whole enclosing rectangle exactly once, except the three lower footprints.
    const std::array<Vector3,4> corners{{origin + Vector3{-150,0,-80}, origin + Vector3{150,0,-80},
                                       origin + Vector3{150,0,80}, origin + Vector3{-150,0,80}}};
    std::vector<std::array<Vector3,2>> links;
    for (int i = 0; i < 4; ++i) links.push_back({corners[i], corners[(i+1)%4]});
    std::vector<const RoadGeometry*> footprints;
    for (const auto& road : world.roads) footprints.push_back(&road);
    const std::array shores{Shore(*river)};
    auto fill = BuildTerrainFill(footprints, links, shores);
    Check(fill && std::abs(Area(*fill) + GroundArea(bridge) - (300 - 25 - railWidth - 10) * 160) < 1e-4,
          "Bridge, banks and adjoining terrain do not seal without overlaps");

    const auto revision = bridge.revision;
    for (int i = 0; i < 5; ++i) world.RefreshTerrainFills();
    Check(bridge.revision == revision, "Unchanged bridge rebuilt each frame");
    Check(world.PlaceLake(LakeDefinition::Circle(30), ObjectTransform{origin + Vector3{10000,0,0}}), "Place unrelated lake");
    world.RefreshTerrainFills();
    Check(bridge.revision == revision, "Unrelated water rebuilt bridge");
    Check(world.RemoveTerrain(1), "Remove unrelated lake");
    world.RefreshTerrainFills();
    Check(bridge.revision == revision, "Unrelated water deletion rebuilt bridge");

    Check(river->BeginWaterEdit() && river->SetRiver(RiverDefinition::Straight(35)), "Widen river draft");
    world.RefreshTerrainFills();
    Check(bridge.revision == revision, "Draft bank edit rebuilt bridge");
    Check(river->FinishWaterEdit(), "Finish river edit");
    world.RefreshTerrainFills();
    Check(bridge.revision != revision && std::abs(GroundArea(bridge) - (expected - 100)) < 1e-4,
          "Committed banks did not update the underpass ground");
    Check(river->SetTransform(ObjectTransform{origin + Vector3{-55,2,-100}}), "Move river and raise water");
    world.RefreshTerrainFills();
    for (const auto& p : bridge.groundFootprints[0].outline)
        if (std::abs(std::abs(p.x - origin.x + 55) - 17.5) < 1e-6)
            Check(std::abs(p.y - 2) < 1e-6, "Ground seam missed changed bank elevation");
    const auto movedRevision = bridge.revision;
    const auto waterId = river->persistentId;
    Check(world.RemoveTerrain(0), "Delete river");
    world.RefreshTerrainFills();
    Check(bridge.revision != movedRevision && bridge.bridgeWaterSources.empty() &&
          std::abs(GroundArea(bridge) - (expected + 250)) < 1e-4, "Deleted river left a stale channel hole");
    // Same identity as an undo restore, including regeneration after the terrain index changed.
    auto* restored = world.PlaceRiver(RiverDefinition::Straight(), ObjectTransform{origin + Vector3{-60,0,-100}});
    Check(restored, "Restore river"); restored->persistentId = waterId;
    world.RefreshTerrainFills();
    Check(std::abs(GroundArea(bridge) - expected) < 1e-4, "Restored river was not rebound by stable ID");
    auto split = bridge.Section(0,1);
    Check(split.points[0].bridge->lowerWaterBodies.contains(waterId) && !split.bridgeWaterSources.empty(),
          "Split span lost water dependency or geometry source");
}

void Planning()
{
    WorldDocument world(AssetRoot::Project);
    Populate(world);
    auto& upper = world.roads[0];
    // Moving the water beyond the span must not add it just because it exists in the world.
    auto* river = world.Terrain()[0].get();
    Check(river->SetTransform(ObjectTransform{{-60,0,300}}), "Move river away");
    auto plan = PlanOverpass(upper, {0,1}, world.roads, world.Terrain());
    Check(plan && plan->lowerWaterBodies.empty() && plan->lowerRoads.size() == 2, "Noncrossing river captured");
    Check(river->SetTransform(ObjectTransform{{-60,0,-100}}), "Move river back");
    for (auto& p : upper.points) p.position.y = 1.5;
    upper.Rebuild();
    // Isolate water validation from road/rail checks.
    Check(!PlanOverpass(upper, {0,1}, {}, world.Terrain()), "Insufficient water clearance accepted");
    for (auto& p : upper.points) p.position.y = 12;
    upper.points.front().position.x = -60;
    upper.Rebuild();
    Check(!PlanOverpass(upper, {0,1}, {}, world.Terrain()), "Abutment in river accepted");
    upper.points.front().position.x = -120;
    upper.Rebuild();
    Check(PlanOverpass(upper, {0,1}, {}, world.Terrain()).has_value(), "River-only bridge rejected");
    auto slope = Straight(world, {-120,0,0}, {120,12,0});
    Check(PlanOverpass(slope, {0}, {}, world.Terrain()).has_value(), "Dry low approach incorrectly failed water clearance");
    auto* second = world.PlaceRiver(RiverDefinition::Straight(20), ObjectTransform{{45,0,-100}});
    Check(second, "Place second river");
    const auto twoRivers = PlanOverpass(upper, {0,1}, {}, world.Terrain());
    Check(twoRivers && twoRivers->lowerWaterBodies.size() == 2, "Second crossed water body omitted");
    Check(world.RemoveTerrain(1), "Remove second river");
    Check(river->BeginWaterEdit(), "Begin water draft");
    Check(!PlanOverpass(upper, {0,1}, {}, world.Terrain()), "Bridge accepted unfinished river");
    river->CancelWaterEdit();
    Check(world.RemoveTerrain(0), "Delete planning river");
    Check(world.PlaceLake(LakeDefinition::Circle(30), ObjectTransform{{-60,0,0}}), "Place lake below span");
    Check(PlanOverpass(upper, {0,1}, {}, world.Terrain()).has_value(), "Lake-only bridge rejected");
    upper.style.railroad.emplace(); upper.Rebuild();
    Check(PlanOverpass(upper, {0,1}, {}, world.Terrain()).has_value(), "Railroad bridge over water rejected");
}

void SplitApproaches()
{
    for (bool railway : {false, true})
    {
        WorldDocument world(AssetRoot::Project);
        Populate(world);
        if (railway)
        {
            world.roads[0].style.railroad.emplace();
            world.roads[0].Rebuild();
        }
        // A bridge section split out of a curved alignment retains geometric
        // endpoint joins, without connector sockets. Non-collinear sample chords
        // at the joins used to misidentify both approaches as lower crossings.
        auto left = Straight(world, {-220,12,-10}, {-120,12,0}, railway);
        auto right = Straight(world, {120,12,0}, {220,12,10}, railway);
        for (auto* road : {&left, &right})
        {
            road->style.curved = true;
            for (auto& p : road->points) p.heading = 0;
            road->Rebuild();
        }
        world.roads.push_back(std::move(left));
        world.roads.push_back(std::move(right));
        for (int direction = 0; direction < 2; ++direction)
        {
            const auto plan = PlanOverpass(world.roads[0], {0,1}, world.roads, world.Terrain());
            Check(plan && plan->lowerRoads == std::set<int>{world.roads[1].id, world.roads[2].id} &&
                  plan->lowerWaterBodies.size() == 1, "Shared approach endpoints blocked the overpass");
            Check(!PlanOverpass(world.roads[0], {0,1}, std::span(world.roads).subspan(3)) &&
                  std::string(SDL_GetError()).find("must cross above") != std::string::npos,
                  "Approach joins alone counted as an overpass");
            for (std::size_t i : {3,4})
            {
                auto& road = world.roads[i];
                std::reverse(road.points.begin(), road.points.end());
                for (auto& p : road.points) p.heading = *p.heading + std::acos(-1.0);
                road.Rebuild();
            }
        }
        // Matching XZ is insufficient: a separate endpoint just below the deck
        // must still meet the minimum clearance, including railway rail heads.
        world.roads[3].points.back().position.y -= .5;
        world.roads[3].Rebuild();
        Check(!PlanOverpass(world.roads[0], {0,1}, world.roads, world.Terrain()) &&
              std::string(SDL_GetError()).find("Raise the upper") != std::string::npos,
              "A vertically separate approach escaped the clearance check");
    }

    WorldDocument world(AssetRoot::Project);
    Populate(world);
    auto approach = Straight(world, {-120,12,0}, {-200,12,-20});
    approach.points.push_back({{0,12,-20}});
    approach.points.push_back({{0,12,20}});
    approach.Rebuild();
    world.roads.push_back(std::move(approach));
    Check(!PlanOverpass(world.roads[0], {0,1}, world.roads, world.Terrain()) &&
          std::string(SDL_GetError()).find("Raise the upper") != std::string::npos,
          "Ignoring an endpoint also ignored the same road's interior crossing");
    // A road terminating at an internal upper control point is not an end-to-end
    // approach, even when that point is the boundary of the selected span.
    world.roads.back() = Straight(world, {0,12,-40}, {0,12,0});
    Check(!PlanOverpass(world.roads[0], {0}, world.roads, world.Terrain()) &&
          std::string(SDL_GetError()).find("Raise the upper") != std::string::npos,
          "Internal control-point contact escaped the clearance check");
    world.roads.back() = Straight(world, {-120,12,-40}, {-120,12,40});
    Check(!PlanOverpass(world.roads[0], {0,1}, world.roads, world.Terrain()) &&
          std::string(SDL_GetError()).find("Raise the upper") != std::string::npos,
          "Lower road interior crossing an upper endpoint escaped the clearance check");
}

void PersistenceAndCooking(const std::filesystem::path& root)
{
    WorldDocument world(AssetRoot::Project);
    const auto source = (root / "multi-overpass.wld").string(), cooked = (root / "multi-overpass.bscworld").string();
    Check(world.Open(source.c_str()), "Open fixture"); Populate(world);
    RoadEditorState state; state.SelectPoint(0,-1);
    EditSnapshot before(state, &world.roads[0]);
    const auto settings = Install(world);
    EditHistory history;
    Check(history.Record(0, before, EditSnapshot(state, &world.roads[0])), "Record overpass creation");
    Check(history.Undo(world, state) == EditHistory::Result::MapChanged && !world.roads[0].HasOverpasses(), "Undo overpass");
    Check(history.Redo(world, state) == EditHistory::Result::MapChanged && world.roads[0].points[0].bridge == settings,
          "Redo lost multi-element dependencies");
    world.UpdateIntersections(); // The editor refreshes dependencies after a history restore.
    Check(world.Save(), "Save overpass");
    WorldDocument loaded(AssetRoot::Project); Check(loaded.Open(source.c_str()), "Load overpass");
    Check(loaded.roads[0].points[0].bridge == settings && !loaded.roads[0].bridgeWaterSources.empty(), "Reload lost water linkage");
    Check(std::abs(GroundArea(loaded.roads[0]) - GroundArea(world.roads[0])) < 1e-4, "Reload altered bridge ground");
    Check(WorldPackage::Cook(loaded, cooked), "Cook overpass");
    WorldPackage::Reader reader; Check(reader.Open(cooked), "Read cooked overpass");
    const EntityId upper{SceneKind::Road, std::uint64_t(loaded.roads[0].id)};
    const EntityId water{SceneKind::Terrain, loaded.Terrain()[0]->persistentId};
    for (const auto& entity : reader.Index().entities)
    {
        if (entity.id == upper)
        {
            Check(std::ranges::find(entity.dependencies, water) != entity.dependencies.end(), "Missing cooked water dependency");
            for (int road : settings.lowerRoads)
                Check(std::ranges::find(entity.dependencies, EntityId{SceneKind::Road, std::uint64_t(road)}) != entity.dependencies.end(),
                      "Missing cooked road dependency");
        }
        if (entity.id == water)
            Check(std::ranges::find(entity.dependencies, upper) != entity.dependencies.end(), "Missing reciprocal bridge dependency");
    }
    WorldCollision collision;
    for (std::size_t i = 0; i < reader.Index().cells.size(); ++i)
    {
        MeshInstance mesh; Check(reader.ReadCollision(i, mesh), "Read collision chunk"); collision.AddMesh(mesh);
    }
    collision.Finish();
    const auto bed = collision.Raycast({-60,8,0}, {0,-1,0}, 30);
    Check(bed && bed->position.y < -2, "Bridge ground collision covers the riverbed");
    const auto road = collision.Raycast({70,8,0}, {0,-1,0}, 30);
    Check(road && std::abs(road->position.y) < 1e-6, "Lower road blocked by bridge ground");
    const auto deck = collision.Raycast({-60,20,0}, {0,-1,0}, 30);
    Check(deck && std::abs(deck->position.y - 12) < .1, "Highway deck collision missing");
}

void LegacyRoad()
{
    WorldDocument world;
    auto road = Straight(world, {-100,12,0}, {100,12,0});
    const BridgeSettings settings{.625, 123.25, {101,102}, {}};
    road.points[0].bridge = settings;
    std::ostringstream saved; saved << std::setprecision(17); RoadIO::WriteRoad(saved, road);
    std::string legacy = saved.str();
    const std::string record = " 0.625 123.25 2 101 102 0";
    const auto at = legacy.find(record);
    Check(at != std::string::npos, "Find bridge record");
    // ROAD v24 ends the bridge record after the lower-road IDs; no water count.
    legacy.erase(at + record.size() - 2, 2);
    std::istringstream input(legacy);
    const auto loaded = RoadIO::ReadRoad(input, 24);
    Check(bool(input) && loaded.points.size() == 2 && loaded.points[0].bridge == settings,
          "Older road/rail bridges no longer load");
}
}

int main()
{
    SDL_Time now; SDL_GetCurrentTime(&now);
    const auto root = std::filesystem::temp_directory_path() / ("bigsky-overpasses-" + std::to_string(now));
    try
    {
        std::filesystem::create_directories(root);
        GeometryAndUpdates(); Planning(); SplitApproaches(); PersistenceAndCooking(root); LegacyRoad();
        std::filesystem::remove_all(root);
        std::cout << "Multi-element overpass tests passed\n"; return 0;
    }
    catch (const std::exception& e) { std::cerr << e.what() << "\nFixture: " << root << '\n'; return 1; }
}
