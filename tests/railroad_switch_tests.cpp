#include "railroad_switch.hpp"
#include "road_io.hpp"
#include "world_package.hpp"
#include "../editor/edit_history.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace
{
void Check(bool ok,const char* why) { if (!ok) throw std::runtime_error(std::string(why)+": "+SDL_GetError()); }
bool Near(Vector3 a,Vector3 b) { return Math::Length(a-b)<1e-7; }
Road Track(int id=1)
{
    Road r; r.id=id; r.style.railroad.emplace(); r.style.material.clear(); r.style.SetLaneCount(1);
    r.style.edge=RoadEdge::Transition; r.points={{{-100,0,0}},{{100,0,0}}};r.Rebuild();return r;
}
void Plant(WorldDocument& world,int host,int segment,double t,int side,int direction)
{
    auto plan=PlanRailroadSwitch(world.roads[host],segment,t,side,direction,world.NextRampStationId());
    Check(plan.has_value(),"Plan switch");
    plan->branch.id=world.AllocateRoadId();
    world.roads[host].points=plan->hostPoints;world.roads[host].Rebuild();
    world.roads.push_back(std::move(plan->branch));world.RefreshRampConnections(true);
}
std::size_t Size(const Road& r)
{
    std::size_t count=r.pavementTriangles.size()+r.edgeTriangles.size();
    for(const auto& [key,tris]:r.transitionStrips) count+=tris.size();
    for(const auto& detail:r.railroadSegments) count+=detail.rails.size()+detail.ties.size();
    return count;
}
void Basics()
{
    for(int direction:{-1,1}) for(int side:{-1,1})
    {
        WorldDocument w(AssetRoot::Project); auto main=Track(w.AllocateRoadId());
        main.points[0].position={51526.853,571.125,-4761.191};main.points[1].position=main.points[0].position+Vector3{200,4,40};main.Rebuild();
        w.roads.push_back(main);Plant(w,0,0,.5,side,direction);
        auto& host=w.roads[0];auto& branch=w.roads[1];
        const auto frame=RailroadSwitchFrame(host,1,direction);
        Check(Near(branch.trackSections.front().top,frame.origin),"Switch rail head mismatch on a grade");
        Check(Math::Dot(branch.Tangent(0),frame.x)>1-1e-9,"Switch tangent mismatch");
        Check(Math::Dot(branch.points.back().position-branch.points.front().position,frame.z)*side>0,"Wrong handedness");
        Check(!PlanRailroadSwitch(host,1,0,side,direction,w.NextRampStationId()),"Allowed second branch at single switch");
        Check(!branch.pavementTriangles.empty() && !branch.railroadSegments.empty(),"Missing turnout geometry");
        const auto rev=RoadGeometry::latestRevision;const auto count=Size(branch);
        for(int i=0;i<5;++i) w.RefreshRampConnections(true);
        Check(RoadGeometry::latestRevision==rev && Size(branch)==count,"Idle switch rebuilds/accumulates geometry");
        const auto before=branch.points.front().position;
        host.points[1].position.y+=2;host.Rebuild();w.UpdateIntersections();
        Check(!Near(before,branch.points.front().position) && Near(branch.trackSections.front().top,RailroadSwitchFrame(host,1,direction).origin),"Switch did not follow host edit");
        std::ostringstream out;out.precision(17);RoadIO::WriteRoad(out,branch);Check(bool(out),"Write switch");
        std::istringstream in(out.str());auto loaded=RoadIO::ReadRoad(in);Check(bool(in) && loaded.points==branch.points,"Switch round trip");
        branch.points.back().position.z+=2;branch.Rebuild();w.RefreshRampConnections(true);
        Check(!branch.pavementTriangles.empty(),"Edited switch lost ballast");
    }
    auto road=Track();road.style.railroad.reset();Check(!PlanRailroadSwitch(road,0,.5,1,1,1),"Road accepted rail switch");
    road=Track();Check(!PlanRailroadSwitch(road,0,1,1,1,1) && !PlanRailroadSwitch(road,0,0,1,-1,1),"Outward switch without continuing main track");
}
void History()
{
    WorldDocument w(AssetRoot::Project);w.roads.push_back(Track(w.AllocateRoadId()));Plant(w,0,0,.5,1,1);
    RoadEditorState state;state.SelectPoint(0,1);EditHistory history;
    const int station=w.roads[0].points[1].rampStation;
    Check(history.SplitRoad(w,state,0,1),"Split host at switch");w.UpdateIntersections();
    Check(w.roads.size()==3 && w.roads[1].points.front().rampStation==station && w.roads[2].points.front().railSwitch->station==station,"Split lost station owner");
    Check(history.Undo(w,state)==EditHistory::Result::MapChanged,"Undo split");w.UpdateIntersections();
    Check(w.roads.size()==2 && w.roads[0].points[1].rampStation==station,"Undo split lost switch");
    Check(history.Redo(w,state)==EditHistory::Result::MapChanged,"Redo split");w.UpdateIntersections();
    Check(history.ReplaceRoad(w,state,1,{}),"Delete switch host");w.UpdateIntersections();
    Check(!w.roads.back().points.front().railSwitch,"Deleting host left dangling link");
    Check(history.Undo(w,state)==EditHistory::Result::MapChanged,"Undo delete");w.UpdateIntersections();
    Check(w.roads.back().points.front().railSwitch.has_value(),"Undo delete did not restore link");
}
void BallastAndSleepers()
{
    WorldDocument w(AssetRoot::Project);w.roads.push_back(Track(w.AllocateRoadId()));Plant(w,0,0,.5,1,1);
    bool wide=false;
    for(const auto& segment:w.roads[0].railroadSegments) for(const auto& run:segment.ties)
        wide |= Math::Length(run.first.z)>1.1;
    Check(wide,"No common turnout sleepers");
    const auto old=Size(w.roads[0]);w.roads.pop_back();w.RefreshRampConnections(true);
    auto stock=Track();stock.points=w.roads[0].points;stock.Rebuild();
    Check(Size(w.roads[0])==Size(stock) && Size(w.roads[0])!=old,"Deleting branch did not restore stock geometry");
    Check(PlanRailroadSwitch(w.roads[0],1,0,-1,1,w.roads[0].points[1].rampStation).has_value(),"Cannot reuse a detached switch station");
}
void IndependentSwitches()
{
    WorldDocument w(AssetRoot::Project);w.roads.push_back(Track(w.AllocateRoadId()));Plant(w,0,0,.5,1,1);
    auto distant=Track(w.AllocateRoadId());for(auto& p:distant.points)p.position.z+=1000;distant.Rebuild();
    w.roads.push_back(distant);Plant(w,2,0,.5,-1,1);
    const auto unrelatedHost=w.roads[2].revision,unrelatedBranch=w.roads[3].revision;
    const auto original=w.roads[1].points.back();const auto count=Size(w.roads[1]);
    for(int i=0;i<3;++i)
    {
        w.roads[1].points.back().position.z+=1;w.roads[1].Rebuild();w.RefreshRampConnections(true);
        w.roads[1].points.back()=original;w.roads[1].Rebuild();w.RefreshRampConnections(true);
        Check(Size(w.roads[1])==count,"Repeated switch edits accumulated geometry");
    }
    Check(w.roads[2].revision==unrelatedHost && w.roads[3].revision==unrelatedBranch,"Edited an unrelated switch");
}
void CreationHistory()
{
    WorldDocument w(AssetRoot::Project);w.roads.push_back(Track(w.AllocateRoadId()));
    RoadEditorState state;state.SelectPoint(0,0);EditSnapshot before(state);
    const int id=w.roads[0].id;const auto points=w.roads[0].points;before.connectedRoadPoints.emplace(id,points);
    Plant(w,0,0,.4,1,1);state.SelectPoint(1,1);EditSnapshot after(state,&w.roads[1]);
    after.connectedRoadPoints.emplace(id,w.roads[0].points);EditHistory history;
    Check(history.Record(1,std::move(before),std::move(after)),"Record switch creation");
    Check(history.Undo(w,state)==EditHistory::Result::MapChanged,"Undo creation");w.UpdateIntersections();
    Check(w.roads.size()==1 && w.roads[0].points==points,"Undo creation left an attachment knot");
    Check(history.Redo(w,state)==EditHistory::Result::MapChanged,"Redo creation");w.UpdateIntersections();
    Check(w.roads.size()==2 && w.roads[1].points.front().railSwitch.has_value(),"Redo creation lost connection");
}
void PersistenceAndTerrain(const std::filesystem::path& root)
{
    const auto path=(root/"switch.wld").string();
    WorldDocument w(AssetRoot::Project);Check(w.Open(path.c_str()),"Open switch fixture");
    w.roads.push_back(Track(w.AllocateRoadId()));Plant(w,0,0,.25,1,1);
    auto& host=w.roads[0];
    Check(host.InsertStation(1,2.0/3.0,w.NextRampStationId()),"Second switch toe");
    const int station=host.points[2].rampStation;
    auto& branch=w.roads[1];
    branch.points={branch.points.front(),{{-20,0,30}},{{20,0,30}},{{50,0,0}}};
    branch.points.back().railSwitch=RailroadSwitchLink{station,-1};
    branch.Rebuild();w.UpdateIntersections();
    TerrainBoundary boundary;boundary.roads={host.id,branch.id};
    auto* fill=w.PlaceTerrainFill(boundary);
    Check(fill && fill->Instance().mesh,"Switch loop did not enclose terrain");
    const auto mesh=fill->Instance().mesh;
    w.RefreshTerrainFills();Check(fill->Instance().mesh==mesh,"Idle switch terrain rebuilt");
    Check(w.Save(),"Save switches");
    WorldDocument loaded(AssetRoot::Project);Check(loaded.Open(path.c_str()),"Load switches");
    Check(loaded.roads[1].points.front().railSwitch==branch.points.front().railSwitch &&
          loaded.Terrain()[0]->Instance().mesh,"Load lost switch/fill");
    const auto cooked=(root/"switch.bscworld").string();Check(WorldPackage::Cook(loaded,cooked),"Cook switches");
    WorldPackage::Reader reader;Check(reader.Open(cooked),"Open cooked switches");
    bool dependency=false,rails=false,ties=false;
    for(const auto& entity:reader.Index().entities)
        if(entity.id==EntityId{SceneKind::Road,std::uint64_t(branch.id)})
            dependency=std::ranges::find(entity.dependencies,EntityId{SceneKind::Road,std::uint64_t(host.id)})!=entity.dependencies.end();
    for(std::size_t i=0;i<reader.Index().cells.size();++i)
    {
        WorldPackage::Cell cell;Check(reader.ReadCell(i,cell),"Read switch cell");
        for(const auto& batch:cell.batches) rails |= batch.layer==WorldPackage::Layer::Rail;
        ties |= !cell.ties.empty();
    }
    Check(dependency && rails && ties,"Cook lost switch geometry/dependency");
    loaded.roads[1].points[1].position.y+=1;
    loaded.roads[1].points[2].position.y+=1;
    loaded.roads[1].Rebuild();loaded.UpdateIntersections();
    Check(loaded.Terrain()[0]->Instance().mesh!=nullptr,"Graded switch approach opened terrain");
    // Invalid links fail rather than silently attaching to another road.
    loaded.roads[1].points.front().railSwitch->station=999999;
    Check(!loaded.Save(),"Accepted missing switch station");
}

}
int main()
{
    SDL_Time now;SDL_GetCurrentTime(&now);
    const auto root=std::filesystem::temp_directory_path()/("bigsky-switch-"+std::to_string(now));
    try
    {
        std::filesystem::create_directories(root);
        Basics();History();BallastAndSleepers();IndependentSwitches();CreationHistory();PersistenceAndTerrain(root);
        std::filesystem::remove_all(root);
        std::cout<<"Railroad switch tests passed\n";
    }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
