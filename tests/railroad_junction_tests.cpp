#include "world_package.hpp"
#include "world_collision.hpp"
#include "road_io.hpp"
#include "../editor/edit_history.hpp"
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

TerrainBoundary Populate(WorldDocument& world, Vector3 origin = {})
{
    const std::array<std::array<Vector3,2>,4> lines{{
        {{{-80,0,-50},{80,0,-50}}}, {{{50,.75,-80},{50,.75,80}}},
        {{{-80,0,50},{80,0,50}}}, {{{-50,0,-80},{-50,0,80}}}}};
    TerrainBoundary boundary;
    for (std::size_t i = 0; i < lines.size(); ++i)
    {
        Road road; road.id = world.AllocateRoadId(); road.style.curved = false;
        road.style.railroad.emplace(); road.style.edge = RoadEdge::Transition;
        road.style.railroad->height = i == 1 ? .75 : 1.5;
        road.style.railroad->width = i == 1 ? 6 : 4;
        road.points = {{origin + lines[i][0]}, {origin + lines[i][1]}};
        road.Rebuild(); boundary.roads.insert(road.id); world.roads.push_back(std::move(road));
    }
    return boundary;
}

void Connect(WorldDocument& world)
{
    for (int owner : {0,2})
    {
        auto& road = world.roads[owner];
        const auto sites = FindRailroadCrossings(road, world.roads[1]);
        Check(sites.size() == 1 && PlanRailroadJunction(road, world.roads[1], sites[0]).has_value(), "Plan rail junction");
        road.crossings.push_back(sites[0].link); road.Rebuild();
    }
    world.UpdateIntersections();
}

void Filled(const TerrainChunk* fill)
{
    Check(fill && fill->Instance().mesh, "Railroad boundary did not fill");
    Check(std::abs(Area(*fill->Instance().mesh) - 89.7 * 89.2) < 1e-5, "Railroad fill lost land or covered embankments");
    // A corner touching two different toe elevations must reach both original
    // rims, rather than just lifting the terrain away from the lower railroad.
    bool seam = false;
    const auto& mesh = *fill->Instance().mesh;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        const auto a = mesh.vertices[mesh.indices[i]].position, b = mesh.vertices[mesh.indices[i+1]].position,
                   c = mesh.vertices[mesh.indices[i+2]].position;
        const auto n = Math::Cross(b - a, c - a);
        if (std::abs(n.y) < Math::Length(n) * 1e-6 && Math::Length(n) > 1e-5) seam = true;
    }
    Check(seam, "Railroad fill omitted the vertical sealing faces");
}

void Geometry()
{
    WorldDocument world(AssetRoot::Project);
    const auto boundary = Populate(world, {500000,20,-300000});
    Check(!world.PlaceTerrainFill(boundary), "Unconnected unequal-height rims unexpectedly filled");
    const auto rail = world.roads[1].trackSections;
    const auto bed = world.roads[1].edgeTriangles;
    const auto outline = world.roads[1].outline;
    const auto revision = world.roads[1].revision;
    Connect(world);
    Check(world.roads[1].revision == revision && world.roads[1].outline == outline &&
          world.roads[1].trackSections == rail && world.roads[1].edgeTriangles.size() == bed.size(),
          "Junction modified the target railroad geometry");
    for (const auto& road : world.roads)
        Check(road.crossingProfiles.empty() && road.crossingSlabs.empty(), "Rail junction generated road slab geometry");
    auto* fill = world.PlaceTerrainFill(boundary); Filled(fill);
    const auto unchanged = fill->Instance().mesh;
    for (int i = 0; i < 5; ++i) { world.UpdateIntersections(); world.RefreshTerrainFills(); }
    Check(fill->Instance().mesh == unchanged, "Unchanged railroad junction rebuilt terrain");
    const auto links = world.roads[0].crossings;
    world.roads[0].crossings.clear(); world.roads[0].Rebuild(); world.RefreshTerrainFills();
    Check(!fill->Instance().mesh, "Removed junction left a stale seal");
    world.roads[0].crossings = links; world.roads[0].Rebuild(); world.RefreshTerrainFills(); Filled(fill);
    for (auto& p : world.roads[1].points) p.position.y += 2;
    world.roads[1].Rebuild(); world.RefreshTerrainFills();
    Check(!fill->Instance().mesh, "Elevated crossing retained an at-grade terrain seal");
    for (auto& p : world.roads[1].points) p.position.y -= 2;
    world.roads[1].Rebuild(); world.RefreshTerrainFills(); Filled(fill);
    // Editing the embankment width changes the seal corners and the land area.
    world.roads[1].style.railroad->width += 2; world.roads[1].Rebuild(); world.RefreshTerrainFills();
    Check(fill->Instance().mesh && std::abs(Area(*fill->Instance().mesh) - 88.7 * 89.2) < 1e-5,
          "Edited embankment did not update the terrain seal");
}

void HistoryAndSplits()
{
    WorldDocument world(AssetRoot::Project); const auto boundary = Populate(world);
    RoadEditorState state; state.SelectPoint(0,-1); EditHistory history;
    EditSnapshot before(state, &world.roads[0]);
    Connect(world);
    Check(history.Record(0, before, EditSnapshot(state, &world.roads[0])), "Record junction creation");
    auto* fill = world.PlaceTerrainFill(boundary); Filled(fill);
    Check(history.Undo(world, state) == EditHistory::Result::MapChanged, "Undo junction");
    world.UpdateIntersections(); Check(!fill->Instance().mesh, "Undo retained terrain seal");
    Check(history.Redo(world, state) == EditHistory::Result::MapChanged, "Redo junction");
    world.UpdateIntersections(); Filled(fill);
    Check(world.roads[1].SubdivideSegments({0}), "Subdivide referenced track");
    auto left = world.roads[1].Section(0,1), right = world.roads[1].Section(1,2);
    Check(history.ReplaceRoad(world, state, 1, {left, right}), "Split referenced track");
    world.UpdateIntersections(); Filled(fill);
    Check(world.roads[0].crossings[0].railroadId == world.roads[1].id &&
          world.roads[3].crossings[0].railroadId == world.roads[2].id, "Rail links did not follow split pieces");
    Check(history.Undo(world,state) == EditHistory::Result::MapChanged, "Undo track split");
    world.UpdateIntersections(); Filled(fill);
    Check(history.Redo(world,state) == EditHistory::Result::MapChanged, "Redo track split");
    world.UpdateIntersections(); Filled(fill);
    Check(world.roads[0].SubdivideSegments({0}), "Subdivide owning track");
    left = world.roads[0].Section(0,1); right = world.roads[0].Section(1,2);
    Check(left.crossings.empty() && right.crossings.size() == 1, "Owner split misplaced crossing station");
    Check(history.ReplaceRoad(world,state,0,{left,right}), "Split owning track");
    world.UpdateIntersections(); Filled(fill);
    Check(history.ReplaceRoad(world,state,2,{}), "Delete referenced track");
    world.UpdateIntersections(); Check(!fill->Instance().mesh, "Deleted track left a terrain seal");
    Check(history.Undo(world,state) == EditHistory::Result::MapChanged, "Undo track deletion");
    world.UpdateIntersections(); Filled(fill);
}

void SkewAndGrades()
{
    WorldDocument world(AssetRoot::Project); const auto boundary = Populate(world);
    auto& track = world.roads[1];
    track.points.front().position.x -= 15;
    track.points.back().position.x += 15;
    track.Rebuild(); Connect(world);
    auto* fill = world.PlaceTerrainFill(boundary);
    const double expected = (100 - 5.4 - 4.9 * std::hypot(1.0, 30.0 / 160)) * 89.2;
    Check(fill && fill->Instance().mesh && std::abs(Area(*fill->Instance().mesh) - expected) < 1e-5,
          "Skewed railroad crossing did not seal the land");
    // Each track can have a gentle grade. Match rail-head height at the actual
    // crossing, rather than comparing authored endpoint or embankment-toe Y.
    Road a = world.roads[0], b = track;
    a.points = {{{-80,-.8,0}}, {{80,.8,0}}}; a.Rebuild();
    b.points = {{{0,-.05,-80}}, {{0,1.55,80}}}; b.Rebuild();
    const auto sites = FindRailroadCrossings(a,b);
    Check(sites.size() == 1 && PlanRailroadJunction(a,b,sites[0]).has_value(), "Matching graded tracks rejected");
    b.points[0].position.y += 3; b.points[1].position.y += 3; b.Rebuild();
    Check(!PlanRailroadJunction(a,b,sites[0]), "Railroad overpass accepted as at grade");
    Check(FindRailroadCrossings(a,a).empty(), "Railroad detected a self-crossing");
    b.points = {{{-80,.75,2}}, {{80,.75,2}}}; b.Rebuild();
    Check(FindRailroadCrossings(a,b).empty(), "Parallel railroad detected a crossing");
    // Existing road/rail slab planning remains separate and unchanged.
    a.style.railroad.reset(); a.points = {{{-80,0,0}}, {{80,0,0}}}; a.Rebuild();
    b.points = {{{0,0,-80}}, {{0,0,80}}}; b.Rebuild();
    const auto roadSites = FindRailroadCrossings(a,b);
    Check(roadSites.size() == 1 && PlanRailroadCrossing(a,b,roadSites[0]).has_value(), "Road/rail slab planning regressed");
}

void SplitAtCrossing(const std::filesystem::path& root)
{
    int fixture = 0;
    for (bool splitOwner : {false, true})
    for (bool reverse : {false, true})
    for (double angle : {0.0, 0.37})
    {
        WorldDocument world(AssetRoot::Project);
        const auto path = (root / ("split-crossing-" + std::to_string(++fixture) + ".wld")).string();
        Check(world.Open(path.c_str()), "Open crossing split fixture");
        const auto boundary = Populate(world, {500000,20,-300000});
        for (auto& road : world.roads)
        {
            for (auto& point : road.points)
            {
                const auto p = point.position - Vector3{500000,20,-300000};
                point.position = Vector3{500000,20,-300000} +
                                 Vector3{p.x*std::cos(angle)-p.z*std::sin(angle),p.y,
                                         p.x*std::sin(angle)+p.z*std::cos(angle)};
            }
            if (reverse) std::ranges::reverse(road.points);
            road.Rebuild();
        }
        Connect(world);
        auto* fill = world.PlaceTerrainFill(boundary); Filled(fill);
        const int index = splitOwner ? 0 : 1;
        // Split precisely at the junction, not just somewhere along its railroad.
        const double fraction = splitOwner != reverse ? .8125 : .1875;
        Check(world.roads[index].InsertStation(0, fraction, world.NextRampStationId()),
              "Insert knot at railroad crossing");
        RoadEditorState state; state.SelectPoint(index,1); EditHistory history;
        Check(history.SplitRoad(world,state,index,1), "Split at railroad crossing");
        world.UpdateIntersections(); Filled(fill);
        Check(history.Undo(world,state) == EditHistory::Result::MapChanged, "Undo crossing split");
        world.UpdateIntersections(); Filled(fill);
        Check(history.Redo(world,state) == EditHistory::Result::MapChanged, "Redo crossing split");
        world.UpdateIntersections(); Filled(fill);
        if (!splitOwner)
        {
            Check(world.roads[0].crossings.size() == 2, "Crossing did not retain both split railroad pieces");
            // Earlier editors kept just one target, sometimes the half outside
            // the land pocket. Recover its adjoining selected piece on load.
            const int outsideId = world.roads[reverse ? 2 : 1].id;
            std::erase_if(world.roads[0].crossings, [&](const auto& link) { return link.railroadId != outsideId; });
            world.roads[0].Rebuild();
            world.RefreshTerrainFills(); Filled(fill);
            const auto unchanged = fill->Instance().mesh;
            world.UpdateIntersections(); world.RefreshTerrainFills();
            Check(fill->Instance().mesh == unchanged, "Unchanged split crossing rebuilt terrain");
            auto& continuation = world.roads[reverse ? 1 : 2];
            for (auto& p : continuation.points) p.position.y += 2;
            continuation.Rebuild(); world.RefreshTerrainFills();
            Check(!fill->Instance().mesh, "Disconnected raised railroad retained a crossing seal");
            for (auto& p : continuation.points) p.position.y -= 2;
            continuation.Rebuild(); world.RefreshTerrainFills(); Filled(fill);
        }
        Check(world.Save(), "Save railroad crossing at split");
        WorldDocument loaded(AssetRoot::Project); Check(loaded.Open(path.c_str()), "Reload railroad crossing at split");
        Filled(loaded.Terrain()[0].get());
    }
}

void Persistence(const std::filesystem::path& root)
{
    const auto source = (root / "rail-junction.wld").string(), cooked = (root / "rail-junction.bscworld").string();
    WorldDocument world(AssetRoot::Project); Check(world.Open(source.c_str()), "Open fixture");
    const auto boundary = Populate(world); Connect(world); auto* fill = world.PlaceTerrainFill(boundary); Filled(fill);
    Check(world.Save(), "Save rail junctions");
    WorldDocument loaded(AssetRoot::Project); Check(loaded.Open(source.c_str()), "Reload rail junctions");
    Filled(loaded.Terrain()[0].get());
    Check(loaded.roads[0].crossings == world.roads[0].crossings, "Reload lost rail crossing identity");
    Check(WorldPackage::Cook(loaded,cooked), "Cook rail junctions");
    WorldPackage::Reader reader; Check(reader.Open(cooked), "Open cooked rail junctions");
    for (const auto& entity : reader.Index().entities)
        if (entity.id == EntityId{SceneKind::Road, std::uint64_t(loaded.roads[0].id)})
            Check(std::ranges::find(entity.dependencies, EntityId{SceneKind::Road, std::uint64_t(loaded.roads[1].id)}) !=
                  entity.dependencies.end(), "Cook omitted railroad dependency");
    double cookedArea = 0;
    WorldCollision collision;
    for (std::size_t i = 0; i < reader.Index().cells.size(); ++i)
    {
        WorldPackage::Cell cell; Check(reader.ReadCell(i, cell), "Read rail junction cell");
        for (const auto& batch : cell.batches)
            if (batch.owner == EntityId{SceneKind::Terrain, fill->persistentId}) cookedArea += Area(*batch.nearMesh.mesh);
        MeshInstance mesh; Check(reader.ReadCollision(i,mesh), "Read rail junction collision"); collision.AddMesh(mesh);
    }
    Check(std::abs(cookedArea - 89.7 * 89.2) < 1e-5, "Cooked terrain lost its seal");
    collision.Finish(); Check(collision.Raycast({0,10,0},{0,-1,0},20).has_value(), "Cooked terrain collision missing");
}
}

int main()
{
    SDL_Time now; SDL_GetCurrentTime(&now);
    const auto root = std::filesystem::temp_directory_path() / ("bigsky-rail-junction-" + std::to_string(now));
    try
    {
        std::filesystem::create_directories(root);
        Geometry(); HistoryAndSplits(); SkewAndGrades(); SplitAtCrossing(root); Persistence(root);
        std::filesystem::remove_all(root); std::cout << "Railroad junction tests passed\n"; return 0;
    }
    catch (const std::exception& e) { std::cerr << e.what() << "\nFixture: " << root << '\n'; return 1; }
}
