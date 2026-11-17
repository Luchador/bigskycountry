#include "world_package.hpp"
#include "world_collision.hpp"
#include "../editor/edit_history.hpp"
#include "../editor/plan_selection.hpp"
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
        area += std::abs(Math::Cross(b-a,c-a).y) * .5;
    }
    return area;
}

RiverDefinition River(double width = 25)
{
    auto river = RiverDefinition::Straight(width,200);
    for (auto& bank : river.banks)
        for (auto& p : bank)
        {
            p.position = {p.position.z-100,0,p.position.x};
            p.incoming = {p.incoming.z,0,p.incoming.x};
            p.outgoing = {p.outgoing.z,0,p.outgoing.x};
        }
    return river;
}

TerrainBoundary Populate(WorldDocument& world, Vector3 origin = {})
{
    TerrainBoundary boundary;
    const std::array<std::array<Vector3,2>,3> lines{{
        {{{-40,0,-20},{-40,0,100}}}, {{{40,0,-20},{40,0,100}}}, {{{-40,0,100},{40,0,100}}}}};
    for (const auto& line : lines)
    {
        Road road; road.id = world.AllocateRoadId(); road.style.curved = false;
        road.points = {{origin+line[0]}, {origin+line[1]}}; road.Rebuild();
        boundary.roads.insert(road.id); world.roads.push_back(std::move(road));
    }
    auto* river = world.PlaceRiver(River(), ObjectTransform{origin}); Check(river,"Create river");
    boundary.waterBodies.insert(river->persistentId);
    return boundary;
}

void Filled(const TerrainChunk* fill, double area = 70 * 82.5)
{
    Check(fill && fill->Instance().mesh,"River and roads did not enclose terrain");
    Check(std::abs(Area(*fill->Instance().mesh)-area) < 1e-5,"Terrain covered water or left a land gap");
}

void BankEdges(const Mesh& fill, const TerrainChunk& river)
{
    const auto& shore = river.BuiltShoreline();
    int checked = 0;
    for (std::size_t i = 1; i < shore.size(); ++i)
    {
        auto a = shore[i-1].position, b = shore[i].position;
        if (a.z <= 0 || b.z <= 0 || std::abs(a.x) >= 35 || std::abs(b.x) >= 35) continue;
        a = river.Water().transform.Point(a); b = river.Water().transform.Point(b);
        int uses = 0;
        for (std::size_t t = 0; t < fill.indices.size(); t += 3)
            for (int j = 0; j < 3; ++j)
            {
                const auto p = fill.vertices[fill.indices[t+j]].position, q = fill.vertices[fill.indices[t+(j+1)%3]].position;
                if ((Math::Length(p-a)<1e-7 && Math::Length(q-b)<1e-7) ||
                    (Math::Length(p-b)<1e-7 && Math::Length(q-a)<1e-7)) ++uses;
            }
        Check(uses == 1,"Fill did not keep the exact riverbank edge"); ++checked;
    }
    Check(checked > 10,"Riverbank seam test had no samples");
}

void Selection()
{
    WorldDocument world(AssetRoot::Project); Populate(world);
    RoadEditorState state;
    state.PickElement({SceneKind::Terrain,0},false,false);
    state.PickElement({SceneKind::Road,0},true,false);
    const std::set<SceneKey> pair{{SceneKind::Road,0},{SceneKind::Terrain,0}};
    Check(state.SelectedRows()==pair && IsTerrainFillSelection(pair,&world),"River-first Shift selection lost the river");
    state.PickElement({SceneKind::Road,0},false,false);
    state.PickElement({SceneKind::Terrain,0},true,false);
    Check(state.SelectedRows()==pair,"Road-first Shift selection lost the road");
    Check(state.GeometrySelected({SceneKind::Terrain,0}) && state.GeometrySelected({SceneKind::Road,0}),"Mixed selection not highlighted");
    state.PickElement({SceneKind::Terrain,0},false,true);
    Check(state.SelectedRows()==std::set<SceneKey>{{SceneKind::Road,0}},"Ctrl-click did not remove river");
    state.PickElement({SceneKind::Terrain,0},false,true);
    Check(state.SelectedRows().size()==1,"Subtracting unselected river changed selection");
    state.PickElement({SceneKind::Terrain,0},true,false);
    state.PickElement({SceneKind::Object,0},true,false);
    Check(state.SelectedRows().size()==3,"Adding another scene kind lost mixed selection");
    state.PickElement({SceneKind::Object,0},false,true);
    Check(state.SelectedRows()==pair,"Ctrl-click did not remove other scene kind");
    const auto box = SelectPlanBox(world, {{-110,0,-30},{110,0,110}});
    Check(box.contains({SceneKind::Terrain,0}) && box.contains({SceneKind::Road,0}) && IsTerrainFillSelection(box,&world),
          "2D box omitted the river or could not fill");
    auto* fill = world.PlaceTerrainFill({{world.roads[0].id,world.roads[1].id,world.roads[2].id},{},{world.Terrain()[0]->persistentId}});
    Filled(fill);
    Check(!IsTerrainFillSelection({{SceneKind::Road,0},{SceneKind::Terrain,1}},&world),"Ordinary terrain counted as a bank");
    Check(!IsTerrainFillSelection({{SceneKind::Terrain,0}},&world),"One river alone counted as enclosed land");
}

void GeometryAndUpdates()
{
    WorldDocument world(AssetRoot::Project); const Vector3 origin{500000,15,-300000};
    const auto boundary = Populate(world,origin);
    auto* river = world.Terrain()[0].get(); const auto bed = river->Instance().mesh;
    auto* fill = world.PlaceTerrainFill(boundary); Filled(fill); BankEdges(*fill->Instance().mesh,*river);
    Check(river->Instance().mesh==bed,"Terrain fill rebuilt the river bottom");
    const auto unchanged = fill->Instance().mesh;
    for (int i=0;i<5;++i) world.RefreshTerrainFills();
    Check(fill->Instance().mesh==unchanged,"Idle river fill rebuilt");
    Check(river->BeginWaterEdit() && river->SetRiver(River(35)),"Edit river banks");
    world.RefreshTerrainFills(); Check(fill->Instance().mesh==unchanged,"Draft river edit rebuilt terrain");
    Check(!world.PlaceTerrainFill(boundary),"Unfinished selected river accepted for a new fill");
    Check(river->FinishWaterEdit(),"Finish river editing"); world.RefreshTerrainFills(); Filled(fill,70*77.5);
    BankEdges(*fill->Instance().mesh,*river);
    const auto reshaped = fill->Instance().mesh;
    auto* unrelated = world.PlaceRiver(River(),ObjectTransform{origin+Vector3{10000,0,0}});
    Check(unrelated,"Place unrelated river"); world.RefreshTerrainFills();
    Check(fill->Instance().mesh==reshaped,"Unrelated river rebuilt terrain");
    const auto id = river->persistentId;
    Check(world.RemoveTerrain(0),"Delete required river"); world.RefreshTerrainFills();
    Check(!fill->Instance().mesh,"Missing bank left stale terrain");
    auto* restored = world.PlaceRiver(River(35),ObjectTransform{origin}); Check(restored,"Restore river"); restored->persistentId=id;
    world.RefreshTerrainFills(); Filled(fill,70*77.5);
    world.MoveLastTerrainTo(0); world.RefreshTerrainFills(); Filled(fill,70*77.5);
    Check(fill->boundary->waterBodies==std::set<std::uint64_t>{id},"Outliner row changes broke stable river reference");
    Check(restored->SetTransform(ObjectTransform{origin+Vector3{0,0,5}}),"Move river");
    world.RefreshTerrainFills(); Filled(fill,70*72.5);
    const auto beforeRoad = fill->Instance().mesh;
    for (auto& p : world.roads[2].points) p.position.z += 5;
    for (int i : {0,1}) world.roads[i].points.back().position.z += 5;
    for (auto& road : world.roads) road.Rebuild();
    world.RefreshTerrainFills(); Filled(fill,70*77.5);
    Check(fill->Instance().mesh!=beforeRoad,"Road edits did not rebuild river fill");
}

void WaterOnly()
{
    WorldDocument world(AssetRoot::Project); TerrainBoundary boundary;
    for (int i=0;i<4;++i)
    {
        const bool vertical = i>=2;
        ObjectTransform transform{{vertical ? (i==2 ? -50.0 : 50.0) : 0.0,0,vertical ? 0.0 : (i==0 ? -50.0 : 50.0)},
                                   {0,vertical ? Math::DegreesToRadians(90) : 0,0}};
        auto* river = world.PlaceRiver(River(),transform); Check(river,"Place enclosing river");
        boundary.waterBodies.insert(river->persistentId);
    }
    Filled(world.PlaceTerrainFill(boundary),75*75);
    WorldDocument single(AssetRoot::Project);
    auto* river = single.PlaceRiver(River()); Check(river,"Place lone river");
    Check(!single.PlaceTerrainFill({{},{},{river->persistentId}}),"River bottom was filled as land");
}

void PersistenceAndHistory(const std::filesystem::path& root)
{
    const auto source=(root/"river-fill.wld").string(), cooked=(root/"river-fill.bscworld").string();
    WorldDocument world(AssetRoot::Project); Check(world.Open(source.c_str()),"Open fixture");
    const auto boundary=Populate(world); auto* fill=world.PlaceTerrainFill(boundary); Filled(fill);
    // Exercise a saved fill appearing before its referenced water in the TERR section.
    world.MoveLastTerrainTo(0); Check(world.Save(),"Save explicit river boundary");
    WorldDocument loaded(AssetRoot::Project); Check(loaded.Open(source.c_str()),"Reload river fill before its water");
    Filled(loaded.Terrain()[0].get()); Check(loaded.Terrain()[0]->boundary==boundary,"Reload lost explicit water boundary");
    RoadEditorState state; state.SelectRows({{SceneKind::Terrain,1}}); EditHistory history;
    auto& river=*loaded.Terrain()[1]; EditSnapshot before(state); before.water=EditSnapshot::WaterEdit{1,*river.WaterSource()};
    Check(river.SetRiver(River(35)),"Commit river edit");
    EditSnapshot after(state); after.water=EditSnapshot::WaterEdit{1,*river.WaterSource()};
    Check(history.Record(-1,before,after),"Record river edit"); loaded.RefreshTerrainFills(); Filled(loaded.Terrain()[0].get(),70*77.5);
    Check(history.Undo(loaded,state)==EditHistory::Result::MapChanged,"Undo river edit");
    loaded.RefreshTerrainFills(); Filled(loaded.Terrain()[0].get());
    Check(history.Redo(loaded,state)==EditHistory::Result::MapChanged,"Redo river edit");
    loaded.RefreshTerrainFills(); Filled(loaded.Terrain()[0].get(),70*77.5);
    Check(WorldPackage::Cook(loaded,cooked),"Cook river-bound terrain");
    WorldPackage::Reader reader; Check(reader.Open(cooked),"Read cooked river fill");
    const EntityId fillId{SceneKind::Terrain,loaded.Terrain()[0]->persistentId}, riverId{SceneKind::Terrain,river.persistentId};
    for (const auto& entity:reader.Index().entities)
        if (entity.id==fillId || entity.id==riverId)
            Check(std::ranges::find(entity.dependencies,entity.id==fillId ? riverId : fillId)!=entity.dependencies.end(),
                  "Cook lost river/fill dependency");
    WorldCollision collision; double area=0;
    for (std::size_t i=0;i<reader.Index().cells.size();++i)
    {
        MeshInstance mesh; Check(reader.ReadCollision(i,mesh),"Read collision"); collision.AddMesh(mesh);
        WorldPackage::Cell cell; Check(reader.ReadCell(i,cell),"Read render cell");
        for (const auto& batch:cell.batches) if (batch.owner==fillId) area+=Area(*batch.nearMesh.mesh);
    }
    Check(std::abs(area-70*77.5)<1e-5,"Cooked river fill lost area");
    collision.Finish();
    const auto bed=collision.Raycast({0,20,0},{0,-1,0},40), land=collision.Raycast({0,20,40},{0,-1,0},40);
    WorldCollision riverCollision; riverCollision.AddMesh(river.Instance()); riverCollision.Finish();
    const auto expectedBed=riverCollision.Raycast({0,20,0},{0,-1,0},40);
    Check(expectedBed && expectedBed->position.y < -2 && bed && Math::Length(bed->position-expectedBed->position)<1e-5,
          "Cooked fill covered or changed the riverbed collision");
    Check(land && std::abs(land->position.y)<1e-5,"Cooked fill lost land collision");
}
}

int main()
{
    SDL_Time now; SDL_GetCurrentTime(&now);
    const auto root=std::filesystem::temp_directory_path()/("bigsky-river-fill-"+std::to_string(now));
    try
    {
        std::filesystem::create_directories(root);
        Selection(); GeometryAndUpdates(); WaterOnly(); PersistenceAndHistory(root);
        std::filesystem::remove_all(root); std::cout<<"River terrain fill tests passed\n"; return 0;
    }
    catch (const std::exception& e) { std::cerr<<e.what()<<"\nFixture: "<<root<<'\n'; return 1; }
}
