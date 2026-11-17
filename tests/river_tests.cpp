#include "world_package.hpp"
#include "world_collision.hpp"
#include "../editor/edit_history.hpp"
#include "../editor/properties_inspector.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
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
    for (std::size_t i=0; i<mesh.indices.size(); i+=3)
    {
        const auto a=mesh.vertices[mesh.indices[i]].position, b=mesh.vertices[mesh.indices[i+1]].position, c=mesh.vertices[mesh.indices[i+2]].position;
        Check(Math::Cross(b-a,c-a).y > 0, "Inverted or degenerate river triangle");
        area += Math::Cross(b-a,c-a).y*.5;
    }
    return area;
}
void Seams(const TerrainChunk& river)
{
    const auto& rim = river.BuiltShoreline();
    for (const auto* mesh : {river.Instance().mesh.get(), river.Water().mesh.get()})
    {
        using P=std::array<double,3>;
        std::map<std::pair<P,P>,int> edges;
        const auto point=[](Vector3 p) { return P{p.x,p.y,p.z}; };
        for (std::size_t i=0; i<mesh->indices.size(); i+=3)
            for (int j=0; j<3; ++j)
                ++edges[std::minmax(point(mesh->vertices[mesh->indices[i+j]].position),point(mesh->vertices[mesh->indices[i+(j+1)%3]].position))];
        for (std::size_t i=1; i<rim.size(); ++i)
            Check(edges[std::minmax(point(rim[i-1].position),point(rim[i].position))]==1,"Water and bed do not share exact boundary edges");
    }
}
void Geometry()
{
    auto river=RiverDefinition::Straight();
    Check(river.Point(4)==Vector3{0,0,0} && river.Point(5)==Vector3{0,0,200}, "Default start/end handles");
    auto geometry=RiverGeometry::Build(river);
    Check(geometry && std::abs(Area(*geometry->water)-5000)<1e-6 && std::abs(Area(*geometry->bed)-5000)<1e-6, "Default 25 x 200 river");
    Check(geometry->bed->bounds.min.y < -2 && geometry->water->bounds.min.y==0 && geometry->water->bounds.max.y==0,"Riverbed or surface height");
    const auto original=river;
    Check(river.InsertPoint(0,0,.35)==1 && river.banks[0].size()==3 && river.banks[1]==original.banks[1],"Independent bank insertion");
    for (int i=0; i<=100; ++i)
    {
        const double t=i/100.0;
        const auto p=t<=.35 ? river.Evaluate(0,0,t/.35) : river.Evaluate(0,1,(t-.35)/.65);
        Check(Math::Length(p-original.Evaluate(0,0,t))<1e-10,"Insertion altered the curve");
    }
    Check(river.MovePoint(1,{-25,0,70}),"Bend one bank");
    Check(river.banks[1]==original.banks[1],"Bank edit changed opposite bank");
    const auto previous=river;
    Check(river.Extend(1,{0,0,300}) && river.Extend(0,{0,0,-100}),"Extend both ends");
    Check(river.banks[0].size()==5 && river.banks[1].size()==4,"Extension accumulated/missed points");
    Check(river.Point(river.PointCount()-2)==Vector3{0,0,-100} && river.Point(river.PointCount()-1)==Vector3{0,0,300},"Extended handle positions");
    Check(Math::Length(river.banks[0].back().position-river.banks[1].back().position)==25,"Extension changed width");
    Check(river.banks[0][2].position==previous.banks[0][1].position,"Extension lost existing bank edits");
    Check(bool(RiverGeometry::Build(river)),"Curved unequal-count river did not triangulate");
    auto invalid=RiverDefinition::Straight();
    invalid.banks[0].back().position.x=30;
    Check(!RiverGeometry::Build(invalid),"Crossing banks accepted");
    invalid=RiverDefinition::Straight(); invalid.banks[0]=invalid.banks[1];
    Check(!RiverGeometry::Build(invalid),"Zero-width river accepted");
    Check(!original.banks.empty() && !river.RemovePoint(river.PointCount()-1),"End handle deleted as bank point");
    auto minimum=original; Check(!minimum.RemovePoint(0),"Removed last required bank point");
    std::stringstream saved; saved<<std::setprecision(17); Check(river.Write(saved),"Write river definition");
    RiverDefinition loaded; Check(loaded.Read(saved) && loaded==river,"River definition round trip");
}
void EditingAndHistory(const std::filesystem::path& root)
{
    WorldDocument world(AssetRoot::Project); Check(world.Open((root/"editing.wld").string().c_str()),"Open fixture");
    auto* chunk=world.PlaceRiver(RiverDefinition::Straight(),ObjectTransform{{500000,20,-300000},{0,.3,0}}); Check(chunk,"Place river");
    Seams(*chunk);
    const auto mesh=chunk->Instance().mesh, surface=chunk->Water().mesh;
    const auto revision=chunk->WaterRevision();
    Check(chunk->BeginWaterEdit(),"Begin river editing");
    RoadEditorState state; state.SelectRows({{SceneKind::Terrain,0}});
    const auto properties=InspectProperties(world,state,false);
    Check(properties.river && properties.water && properties.editingWater,"River inspector missing edit state");
    EditSnapshot before(state); before.water=EditSnapshot::WaterEdit{0,*chunk->WaterSource()};
    auto draft=*chunk->River(); Check(draft.InsertPoint(1,0,.6)>=0 && draft.Extend(1,{10,0,280}),"Draft shape");
    Check(chunk->SetRiver(draft),"Preview river");
    Check(chunk->Instance().mesh==mesh && chunk->Water().mesh==surface && chunk->WaterRevision()==revision,"Draft rebuilt geometry/dependencies");
    Check(!world.Save() && !WorldPackage::Cook(world,(root/"draft.bscworld").string()),"Saved/cooked unfinished draft");
    EditSnapshot after(state); after.water=EditSnapshot::WaterEdit{0,*chunk->WaterSource()};
    EditHistory history; Check(history.Record(-1,before,after),"Record river edit");
    Check(history.Undo(world,state)==EditHistory::Result::MapChanged && *chunk->River()==RiverDefinition::Straight(),"Undo draft");
    Check(history.Redo(world,state)==EditHistory::Result::MapChanged && *chunk->River()==draft,"Redo draft");
    Check(chunk->Instance().mesh==mesh,"Draft undo rebuilt geometry");
    Check(chunk->FinishWaterEdit() && chunk->Instance().mesh!=mesh && chunk->WaterRevision()!=revision,"Finish did not build");
    Seams(*chunk);
    const auto committed=chunk->Instance().mesh;
    Check(chunk->BeginWaterEdit(),"Begin invalid draft");
    auto invalid=*chunk->River(); invalid.banks[0].front().position=invalid.banks[1].front().position;
    Check(chunk->SetRiver(invalid) && !chunk->FinishWaterEdit() && chunk->EditingWater(),"Invalid edit should remain open");
    Check(chunk->Instance().mesh==committed,"Invalid edit destroyed committed mesh");
    chunk->CancelWaterEdit(); Check(*chunk->River()==draft && chunk->Instance().mesh==committed,"Cancel did not restore source");
    Check(history.Undo(world,state)==EditHistory::Result::MapChanged && *chunk->River()==RiverDefinition::Straight(),"Undo committed shape");
    Check(history.Redo(world,state)==EditHistory::Result::MapChanged && *chunk->River()==draft,"Redo committed shape");
    // Creation/deletion use the same persistent source payload as lakes.
    EditSnapshot exists(state); exists.addedTerrain=EditSnapshot::TerrainPlacement{{},chunk->GetTransform(),{},"",0,chunk->WaterSource(),chunk->persistentId};
    EditHistory deletion; const auto id=chunk->persistentId;
    Check(world.RemoveTerrain(0),"Delete river"); state.SelectRows({});
    Check(deletion.Record(-1,exists,EditSnapshot(state)),"Record deletion");
    Check(deletion.Undo(world,state)==EditHistory::Result::MapChanged && world.Terrain()[0]->persistentId==id && *world.Terrain()[0]->River()==draft,"Undo deletion lost river/identity");
    Check(world.Save(),"Save committed river");
    WorldDocument loaded(AssetRoot::Project); Check(loaded.Open((root/"editing.wld").string().c_str()),"Reload river");
    Check(loaded.Terrain()[0]->River()==draft && loaded.Terrain()[0]->persistentId==id,"Persistence lost river");
    Seams(*loaded.Terrain()[0]);
}
void CookAndFill(const std::filesystem::path& root)
{
    WorldDocument world(AssetRoot::Project); Check(world.Open((root/"cooking.wld").string().c_str()),"Open cooking fixture");
    TerrainBoundary boundary;
    const std::array<Vector3,4> corners{{{-150,0,-50},{150,0,-50},{150,0,250},{-150,0,250}}};
    for (int i=0;i<4;++i)
    {
        Road road; road.id=world.AllocateRoadId(); road.style.curved=false;
        road.points={{corners[i]},{corners[(i+1)%4]}}; road.Rebuild();
        boundary.roads.insert(road.id); world.roads.push_back(std::move(road));
    }
    auto* fill=world.PlaceTerrainFill(boundary); Check(fill,"Place fill"); const double land=Area(*fill->Instance().mesh);
    auto* river=world.PlaceRiver(RiverDefinition::Straight()); Check(river,"Place river in fill"); world.RefreshTerrainFills();
    Check(std::abs(Area(*fill->Instance().mesh)+5000-land)<1e-6,"Fill covered river basin");
    const auto fillMesh=fill->Instance().mesh;
    Check(river->BeginWaterEdit(),"Begin fill-linked river draft");
    auto draft=*river->River(); Check(draft.Extend(1,{0,0,220}) && river->SetRiver(draft),"Preview extension");
    world.RefreshTerrainFills(); Check(fill->Instance().mesh==fillMesh,"Preview rebuilt surrounding terrain");
    Check(river->FinishWaterEdit(),"Finish extension"); world.RefreshTerrainFills();
    Check(std::abs(Area(*fill->Instance().mesh)+5500-land)<1e-6,"Fill did not follow extended banks");
    Check(WorldPackage::Cook(world,(root/"river.bscworld").string()),"Cook river");
    WorldPackage::Reader reader; Check(reader.Open((root/"river.bscworld").string()),"Read package");
    double waterArea=0; WorldCollision collision;
    for (std::size_t i=0;i<reader.Index().cells.size();++i)
    {
        WorldPackage::Cell cell; Check(reader.ReadCell(i,cell),"Read river cell");
        for (const auto& batch : cell.batches)
            if (batch.owner==EntityId{SceneKind::Terrain,river->persistentId} && batch.material==LakeDefinition::waterMaterial)
                waterArea+=Area(*batch.nearMesh.mesh);
        MeshInstance mesh; Check(reader.ReadCollision(i,mesh),"Read river collision"); collision.AddMesh(mesh);
    }
    Check(std::abs(waterArea-5500)<1e-6,"Cook omitted river water");
    collision.Finish(); const auto floor=collision.Raycast({0,20,100},{0,-1,0},100);
    Check(floor && floor->position.y < -2,"Water/fill blocked riverbed collision");
}
}
int main()
{
    SDL_Time now; SDL_GetCurrentTime(&now);
    const auto root=std::filesystem::temp_directory_path()/("bigsky-rivers-"+std::to_string(now));
    try { std::filesystem::create_directories(root); Geometry(); EditingAndHistory(root); CookAndFill(root);
        std::filesystem::remove_all(root); std::cout<<"River tests passed\n"; }
    catch(const std::exception& error) { std::cerr<<error.what()<<"\nFixture: "<<root<<'\n'; return 1; }
}
