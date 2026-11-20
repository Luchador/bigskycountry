#include "railroad_switch.hpp"
#include "road_io.hpp"
#include "world_package.hpp"
#include "../editor/edit_history.hpp"
#include "../editor/properties_inspector.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace
{
void Check(bool ok, const char* why) { if (!ok) throw std::runtime_error(std::string(why)+": "+SDL_GetError()); }
bool Near(double a,double b,double epsilon=1e-7) { return std::abs(a-b)<epsilon; }
Road Track(int count=1)
{
    Road r; r.id=1; r.style.railroad.emplace(); r.style.material.clear(); r.style.SetLaneCount(count);
    r.style.edge=RoadEdge::Transition; r.points={{{0,0,0}},{{100,0,0}}};r.Rebuild();return r;
}
std::vector<double> Centers(const Road& r,double x)
{
    std::vector<double> result;
    for(const auto& span:r.trackSpans)
    {
        const auto a=span[0].top,b=span[1].top;
        if(x<std::min(a.x,b.x)-1e-7 || x>std::max(a.x,b.x)+1e-7 || Near(a.x,b.x))continue;
        const double z=std::lerp(a.z,b.z,(x-a.x)/(b.x-a.x));
        if(std::ranges::none_of(result,[&](double old){return Near(old,z,1e-5);}))result.push_back(z);
    }
    std::ranges::sort(result);return result;
}
void Uniform()
{
    for(int count=1;count<=8;++count)
    {
        const auto r=Track(count);const auto centers=Centers(r,50);
        Check(centers.size()==static_cast<std::size_t>(count),"Wrong parallel track count");
        for(int i=0;i<count;++i)Check(Near(centers[i],(i-(count-1)*.5)*4),"Incorrect track spacing/centering");
        for(const auto& p:r.outline)Check(Near(std::abs(p.z),r.style.railroad->ToeHalfWidth()+RoadStyle::stripWidth),"Bed did not widen with tracks");
        int ties=0;for(const auto& d:r.railroadSegments)for(const auto& run:d.ties)ties+=run.count;
        Check(ties==166*count,"Missing or duplicated sleepers");
        Check(r.railroadSegments.size()==1 && r.railroadSegments[0].WithinDistance({50,1,centers.back()},1),"Detail culling ignored outer tracks");
        Check(!r.railroadSegments[0].WithinDistance({50,1,centers.back()+1000},300),"Distant rail detail not culled");
        // Check the two inner faces of the actual rail-head polygons, not just a setting.
        for(double center:centers)for(int side:{-1,1})
        {
            const double z=center+side*RailroadSettings::gauge*.5;
            bool found=false;
            for(const auto& tri:r.railroadSegments[0].rails)for(const auto& v:tri)
                found|=Near(v.position.z,z)&&Near(v.position.y,1.1);
            Check(found,"Actual rail mesh gauge differs from 1.435 m");
        }
    }
}
Road Fan(int from,int to)
{
    auto r=Track(from);r.points={{{0,0,0}},{{100,0,0}},{{250,0,0}},{{450,0,0}}};
    Check(r.SetSegmentLaneCounts({2},to),"Set tracks per segment");r.Rebuild();return r;
}
void Transitions()
{
    for(auto pair:{std::pair{1,2},std::pair{2,4},std::pair{4,8},std::pair{8,1},std::pair{3,2}})
    {
        auto r=Fan(pair.first,pair.second);
        Check(Centers(r,20).size()==static_cast<std::size_t>(pair.first),"Fan changed its starting count");
        Check(Centers(r,300).size()==static_cast<std::size_t>(pair.second),"Fan changed its ending count");
        Check(r.SegmentSamples(1).size()>30,"Straight fan was not sampled");
        double previous=std::abs(Centers(r,100).back());
        for(double x=101;x<250;x+=1.19)
        {
            const double width=std::abs(Centers(r,x).back());
            Check((width-previous)*(pair.second-pair.first)>=-1e-8,"Fan bows back in the wrong direction");previous=width;
        }
        for(const auto& span:r.trackSpans)
        {
            for(const auto& p:span) Check(Near(Math::Length(p.side),1)&&std::isfinite(Math::Length(p.top)),"Invalid track sweep frame");
            if(span[0].top.x>1e-6)
            {
                bool connected=false;
                for(const auto& other:r.trackSpans)connected|=Math::Length(other[1].top-span[0].top)<1e-6;
                Check(connected,"Open rail seam inside a fan");
            }
        }
        auto part=r.Section(1,3);part.Rebuild();
        for(double x:{100.0,125.0,175.0,249.9,300.0})
        {
            auto a=Centers(r,x),b=Centers(part,x);Check(a.size()==b.size(),"Splitting lost track correspondence");
            for(std::size_t i=0;i<a.size();++i)Check(Near(a[i],b[i],.002),"Splitting changed the fan shape");
        }
        const auto spans=r.trackSpans;r.Rebuild();Check(spans==r.trackSpans,"Rebuild accumulates/change tracks");
    }
    auto wide=Fan(1,8);
    for(const auto& detail:wide.railroadSegments)for(const auto& ties:detail.ties)
        for(int i=0;i<ties.count;++i)for(int sign:{-1,1})
            Check(Math::Length(ties.At(i).Point({0,0,sign*RailroadSettings::tieLength*.5})-ties.Center())<=ties.Radius(),
                  "Shared sleepers extend beyond their culling bounds");
    auto r=Fan(2,8);Check(r.SetSegmentLaneCounts({0},2),"Explicit source count");r.Rebuild();
    auto first=r.Section(0,1),last=r.Section(1,3);
    for(double x:{100.0,101.0,174.0,249.0})Check(Centers(r,x)==Centers(last,x),"Split neighbor context changed tracks");
    Check(!r.SetSegmentLaneCounts({1},9)&&!r.SetSegmentLaneCounts({1},0),"Accepted invalid track count");
    Check(r.SubdivideSegments({2}),"Subdivide railroad");Check(r.SegmentLaneCount(2)==8&&r.SegmentLaneCount(3)==8,"Subdivision lost track count");
    r.SetLaneCount(3);Check(!r.HasLaneOverrides()&&r.style.GetLaneCount()==3,"Whole railroad edit left overrides");
    // Shared sleepers must keep their width even when retrieved from an instanced run.
    MeshTransform a{},b{};a.x=b.x={1,0,0};a.y=b.y={0,1,0};a.z=b.z={0,0,2};b.origin={6,0,0};
    RailroadTieRun run{a,b,11};Check(Near(Math::Length(run.At(5).z),2),"Instanced shared sleeper lost width");
}
void CurvesAndCooking()
{
    auto curved=Fan(2,6);
    curved.points[1].position={100,3,20};curved.points[2].position={250,6,80};curved.points[3].position={450,8,100};
    curved.rotation={.02,0,.01};curved.Rebuild();
    for(const auto& span:curved.trackSpans)
    {
        const auto direction=Math::Normalize(span[1].top-span[0].top);
        for(const auto& row:span)
            Check(std::isfinite(Math::Length(row.top))&&std::abs(Math::Dot(row.side,direction))<.08,
                  "Curved/graded tracks lost their transverse gauge frame");
    }
    auto before=curved.Section(0,1),after=curved.Section(1,3);
    const auto head=RailroadSwitchFrame(curved,1,1);
    bool checked=false;
    for(const auto& span:before.trackSpans)
        if(std::abs(Math::Dot(span[1].top-head.origin,head.x))<.5)
        {
            double nearest=1e30;
            for(const auto& next:after.trackSpans)nearest=std::min(nearest,Math::Length(next[0].top-span[1].top));
            Check(nearest<.01,"Splitting a graded curve opened a track seam");checked=true;
        }
    Check(checked,"Curved split fixture did not reach seam");
    SDL_Time now;SDL_GetCurrentTime(&now);
    const auto root=std::filesystem::temp_directory_path()/("bigsky-parallel-"+std::to_string(now));
    std::filesystem::create_directories(root);
    WorldDocument world(AssetRoot::Project);Check(world.Open((root/"world.wld").string().c_str()),"Open multi-track fixture");
    world.roads.push_back(Track(8));Check(world.Save(),"Save parallel railways");
    WorldDocument loaded(AssetRoot::Project);Check(loaded.Open((root/"world.wld").string().c_str()),"Reload parallel railways");
    Check(loaded.roads[0].style.GetLaneCount()==8,"World reload lost track count");
    const auto package=(root/"world.bscworld").string();Check(WorldPackage::Cook(loaded,package),"Cook parallel railways");
    WorldPackage::Reader reader;Check(reader.Open(package),"Open cooked parallel railways");
    int count=0;bool rails=false;
    for(std::size_t i=0;i<reader.Index().cells.size();++i)
    {
        WorldPackage::Cell cell;Check(reader.ReadCell(i,cell),"Read multi-track cell");
        for(const auto& tie:cell.ties)count+=tie.run.count;
        for(const auto& batch:cell.batches)rails|=batch.layer==WorldPackage::Layer::Rail;
    }
    Check(rails&&count==166*8,"Cooking/streaming lost parallel rails or ties");
    std::filesystem::remove_all(root);
}
void PersistenceAndInspector()
{
    auto r=Fan(2,8);r.SetSegmentLaneCounts({0},3);r.Rebuild();
    auto split=r.Section(1,3);
    std::stringstream data;data<<std::setprecision(17);RoadIO::WriteRoad(data,split);auto loaded=RoadIO::ReadRoad(data);
    Check(bool(data)&&loaded.points==split.points&&loaded.style==split.style&&loaded.laneNeighbors==split.laneNeighbors,"Railroad counts failed round trip");
    loaded.Rebuild();Check(loaded.trackSpans==split.trackSpans,"Reload changed track geometry");
    // The pre-track-count format's unused lane value must not multiply legacy railways.
    auto old=Track();std::stringstream legacy;RoadIO::WriteRoad(legacy,old);auto bytes=legacy.str();bytes[0]='2';
    std::istringstream in(bytes);auto original=RoadIO::ReadRoad(in,28);Check(bool(in)&&original.style.GetLaneCount()==1,"Legacy railroad became double track");
    auto bad=Track();bad.points.front().segmentLanes=9;std::stringstream invalid;RoadIO::WriteRoad(invalid,bad);Check(!invalid,"Invalid tracks serialized");
    WorldDocument w(AssetRoot::Project);w.roads.push_back(r);RoadEditorState state;state.selected=0;state.point=-1;state.segments={0,2};
    auto p=InspectProperties(w,state,false);Check(p.page==PropertiesPage::Railroad&&p.segmentSelection&&p.minLanes==3&&p.maxLanes==8,"Mixed segment inspector");
    Check(!p.CanStepLanes(true)||p.StepLanes(true)==8,"Inspector allows ninth track");
    EditHistory history;EditSnapshot before(state,&w.roads[0]);w.roads[0].SetSegmentLaneCounts({0,2},4);w.roads[0].Rebuild();
    Check(history.Record(0,std::move(before),EditSnapshot(state,&w.roads[0])),"Record track edit");
    Check(history.Undo(w,state)==EditHistory::Result::MapChanged&&w.roads[0].SegmentLaneCount(2)==8,"Undo track count");
    Check(history.Redo(w,state)==EditHistory::Result::MapChanged&&w.roads[0].SegmentLaneCount(2)==4,"Redo track count");
}
bool Contains(const Road::Triangle& tri,Vector3 p)
{
    double lo=1e30,hi=-1e30;
    for(int i=0;i<3;++i){auto a=tri[(i+1)%3]-tri[i],b=p-tri[i];double d=a.x*b.z-a.z*b.x;lo=std::min(lo,d);hi=std::max(hi,d);}
    return (lo>=-1e-9||hi<=1e-9)&&std::abs(Math::Cross(tri[1]-tri[0],tri[2]-tri[0]).y)>1e-10;
}
void Crossings()
{
    auto rail=Track(4);rail.points={{{-100,0,0}},{{100,0,0}}};rail.Rebuild();
    Road road;road.id=2;road.points={{{0,0,-100}},{{0,0,100}}};road.Rebuild();
    auto sites=FindRailroadCrossings(road,rail);Check(sites.size()==1,"Find multi-track grade crossing");
    const auto profile=PlanRailroadCrossing(road,rail,sites[0]);Check(profile&&profile->tracks.size()>=4,"Missing crossing rail sweeps");
    road.crossingProfiles={*profile};road.Rebuild();Check(!road.crossingSlabs.empty(),"No multi-track concrete slab");
    for(double center:Centers(rail,0))for(int side:{-1,1})
    {
        Vector3 at{.13,1.1,center+side*(RailroadSettings::gauge*.5-.03)};double height=-1e20;
        for(const auto& t:road.edgeTriangles)
        {
            Road::Triangle tri{t[0].position,t[1].position,t[2].position};
            if(Contains(tri,at))height=std::max(height,tri[0].y);
        }
        Check(Near(height,1.1-RailroadSettings::grooveDepth),"Concrete obstructs an outer-track flangeway");
    }
    auto upper=road;upper.crossingProfiles.clear();for(auto& p:upper.points)p.position.y=8;upper.Rebuild();
    std::vector<Road> all{rail,upper};auto plan=PlanOverpass(all[1],{0},all);Check(plan.has_value(),"Overpass over parallel tracks");
    auto second=rail;second.id=3;for(auto& p:second.points)std::swap(p.position.x,p.position.z);second.Rebuild();
    auto junction=FindRailroadCrossings(rail,second);Check(!junction.empty()&&PlanRailroadJunction(rail,second,junction[0]).has_value(),"Widened rail junction lost terrain seal");
}
void Switches()
{
    for(int direction:{-1,1})for(int side:{-1,1})
    {
        WorldDocument w(AssetRoot::Project);w.roads.push_back(Track(4));
        auto plan=PlanRailroadSwitch(w.roads[0],0,.5,side,direction,1);Check(plan.has_value(),"Outer-track switch");
        Check(plan->branch.style.GetLaneCount()==1,"Switch branch inherited track bundle");
        Check(Near(plan->branch.points.front().position.z,6*side*direction),"Switch not attached to the outer track");
        w.roads[0].points=plan->hostPoints;w.roads[0].Rebuild();plan->branch.id=2;w.roads.push_back(std::move(plan->branch));w.UpdateIntersections();
        w.roads[0].SetLaneCount(6);w.roads[0].Rebuild();w.UpdateIntersections();
        Check(Near(w.roads[1].points.front().position.z,10*side*direction),"Switch did not follow changed track count");
    }
}
}
int main(int argc,char** argv)
{
    try
    {
        Uniform();Transitions();CurvesAndCooking();PersistenceAndInspector();Crossings();Switches();
        if(argc>1)
        {
            std::ofstream out(argv[1]);out<<std::setprecision(17);
            const auto r=Fan(2,8);
            for(const auto& span:r.trackSpans)out<<span[0].top.x<<','<<span[0].top.z<<','<<span[1].top.x<<','<<span[1].top.z<<'\n';
        }
        std::cout<<"Parallel railroad tests passed\n";return 0;
    }
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
