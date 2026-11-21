#include "curved_road.hpp"
#include "road_join_geometry.hpp"
#include "road_curve.hpp"
#include "world_document.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    using namespace RoadJoinGeometry;
    struct Sample { Vector3 p; double distance; int segment; double t; };
    struct HostPath
    {
        std::vector<Sample> samples;
        std::map<int, double> stations;
        std::vector<RoadSpline::Cubic> curves;
        explicit HostPath(const Road& road)
        {
            double distance = 0;
            for (int i = 0; i + 1 < static_cast<int>(road.points.size()); ++i)
            {
                if (road.points[i].rampStation) stations[road.points[i].rampStation] = distance;
                const auto& a = road.points[i]; const auto& b = road.points[i+1];
                const auto chord = (b.position-a.position)*(1.0/3.0);
                curves.push_back(road.style.curved ? RoadSpline::SegmentCurve(a,b,road.Tangent(i),road.Tangent(i+1)) :
                                                     RoadSpline::Cubic{a.position,a.position+chord,b.position-chord,b.position});
                if (samples.empty()) samples.push_back({a.position,0,i,0});
                auto previous = a.position;
                RoadSpline::SampleCubic(curves.back(),0.02,0,0,1,[&](Vector3 p,Vector3,double t)
                {
                    distance += Math::Length(p-previous);
                    samples.push_back({p,distance,i,t}); previous=p;
                });
            }
            if (!road.points.empty() && road.points.back().rampStation)
                stations[road.points.back().rampStation] = distance;
        }
    };
    struct Frame
    {
        Vector3 center, forward, left, up;
        RoadStyle style;
    };
    std::optional<Frame> Attachment(const Road& host, const HostPath& path, const CurvedRoadLink& link,
                                    const RoadStyle& branch)
    {
        if (path.samples.size() < 2 || !path.stations.contains(link.station)) return {};
        const double distance = std::clamp(path.stations.at(link.station) + link.offset, 0.0, path.samples.back().distance);
        auto next = std::upper_bound(path.samples.begin(), path.samples.end(), distance,
                                     [](double d, const Sample& s) { return d < s.distance; });
        if (next == path.samples.end()) --next;
        const auto& a = *(next - 1); const auto& b = *next;
        const double mix = b.distance > a.distance ? (distance-a.distance)/(b.distance-a.distance) : 0;
        const double t=std::lerp(a.segment==b.segment ? a.t : 0,b.t,mix),u=1-t;
        const auto& curve=path.curves[b.segment];
        const auto position=curve.a*(u*u*u)+curve.b*(3*u*u*t)+curve.c*(3*u*t*t)+curve.d*(t*t*t);
        const auto forward=Math::Normalize((curve.b-curve.a)*(u*u)+(curve.c-curve.b)*(2*u*t)+(curve.d-curve.c)*(t*t));
        const auto transform=ObjectTransform{{},host.rotation}.ToMeshTransform();
        const auto up=Math::Normalize(transform.Direction(host.SurfaceUp(b.segment,t)));
        const auto left=Math::Normalize(Math::Cross(forward,up));
        if (left==Vector3{}) return {};
        const auto profile=host.SegmentSurfaceBlend(b.segment,t);
        const auto outer = [](const RoadStyle& s) { return s.HalfSeparation()+s.GetLaneCount()*s.TravelLaneWidth()*.5; };
        const double half = std::lerp(outer(host.SurfaceStyle(profile.from)),outer(host.SurfaceStyle(profile.to)),profile.mix);
        const double branchHalf = branch.GetLaneCount()*branch.TravelLaneWidth()*.5;
        // Put the branch inside the outer travel edge. The shared pavement is
        // clipped below, rather than manufacturing a new auxiliary/taper lane.
        const auto center = position+left*(link.side*(half-branchHalf));
        return Frame{center,forward,left,up,host.SegmentStyle(b.segment)};
    }
    void Align(RoadPoint& point, const Frame& frame, bool first)
    {
        const auto forward = frame.forward*(point.curveJoin->direction*(first ? 1.0 : -1.0));
        point.position = frame.center;
        point.heading = std::atan2(forward.z,forward.x);
        point.pitch = std::atan2(forward.y,std::hypot(forward.x,forward.z));
        point.deadEndLength = 0;
    }
    bool ClearsMouths(const Road& host, Vector3 p)
    {
        for (bool first : {true,false})
            if (const auto mouth=host.EndpointMouth(first); mouth && Math::Dot(p-mouth->center,mouth->outward)<RoadStyle::laneWidth)
                return false;
        return true;
    }
    Box JoinRegion(const Road& branch, bool first)
    {
        const std::size_t end = first ? 0 : branch.points.size()-1, next = first ? 1 : end-1;
        const auto p = branch.points[end].position;
        const double reach = std::max(4*(branch.style.LeftWidth()+branch.style.RightWidth()),
                                      Math::Length(branch.points[next].position-p)*.75);
        Box box(p); box.Add(p+Vector3{reach,0,reach}); box.Add(p-Vector3{reach,0,reach});
        return box;
    }
    template<class V> std::vector<Cutter> SurfaceCutters(const std::vector<std::array<V,3>>& triangles, const Box& region)
    {
        std::vector<Cutter> result;
        for (const auto& tri : triangles)
        {
            const std::array<Vector3,3> p{tri[0].position,tri[1].position,tri[2].position};
            Box box(p[0]); box.Add(p[1]); box.Add(p[2]);
            if (!box.Overlaps(region) || std::abs(Math::Cross(p[1]-p[0],p[2]-p[0]).y)<1e-10) continue;
            Cutter cut{box,{}};
            for (int i=0;i<3;++i)
            {
                const auto edge=p[(i+1)%3]-p[i]; auto n=Vector3{-edge.z,0,edge.x};
                if (Math::Dot(p[(i+2)%3]-p[i],n)>0) n=n*-1;
                cut.planes.push_back({p[i],Math::Normalize(n)});
            }
            result.push_back(std::move(cut));
        }
        return result;
    }
    void CutPaint(std::vector<RoadGeometry::Triangle>& paint, const std::vector<Cutter>& cutters)
    {
        RoadGeometry::ColoredSurface surface;
        surface.reserve(paint.size());
        for (const auto& t : paint) surface.push_back({{{t[0],{}},{t[1],{}},{t[2],{}}}});
        Cut(surface,cutters);
        paint.clear(); paint.reserve(surface.size());
        for (const auto& t : surface) paint.push_back({t[0].position,t[1].position,t[2].position});
    }
    // Nearest point in a triangle's XZ projection, retaining its exact height.
    Vector3 Nearest(Vector3 p, const std::array<RoadGeometry::PavementVertex,3>& t)
    {
        const auto a=t[0].position,b=t[1].position,c=t[2].position;
        const auto cross=[](Vector3 u,Vector3 v){return u.x*v.z-u.z*v.x;};
        const double d=cross(b-a,c-a);
        if (std::abs(d)>1e-12)
        {
            const double u=cross(p-a,c-a)/d,v=cross(b-a,p-a)/d;
            if (u>=0 && v>=0 && u+v<=1) return {p.x,a.y+(b.y-a.y)*u+(c.y-a.y)*v,p.z};
        }
        Vector3 best=a; double distance=std::numeric_limits<double>::infinity();
        for (int i=0;i<3;++i)
        {
            const auto from=t[i].position, delta=t[(i+1)%3].position-from;
            const double l=delta.x*delta.x+delta.z*delta.z;
            const double f=l>0 ? std::clamp(((p.x-from.x)*delta.x+(p.z-from.z)*delta.z)/l,0.0,1.0) : 0;
            const auto q=from+delta*f; const double r=std::hypot(p.x-q.x,p.z-q.z);
            if (r<distance) { distance=r; best=q; }
        }
        return best;
    }
    void Conform(Road& branch, const Road& host, const Box& region, bool boundaryOnly)
    {
        std::vector<const std::array<RoadGeometry::PavementVertex,3>*> nearby;
        for (const auto& t : host.pavementTriangles)
        {
            Box box(t[0].position); box.Add(t[1].position); box.Add(t[2].position);
            if (box.Overlaps(region)) nearby.push_back(&t);
        }
        const auto fit=[&](Vector3& p,double lift)
        {
            if (p.x<region.lo.x || p.x>region.hi.x || p.z<region.lo.z || p.z>region.hi.z) return;
            const double blend=std::max(5.0,branch.style.LeftWidth()+branch.style.RightWidth());
            double best=boundaryOnly ? 1e-6 : blend; Vector3 at{}; bool found=false;
            for (const auto* t : nearby)
            {
                const auto q=Nearest(p,*t); const double d=std::hypot(p.x-q.x,p.z-q.z);
                if (d<best) { best=d; at=q; found=true; if (d==0) break; }
            }
            if (!found) return;
            const double f=std::clamp(1-best/blend,0.0,1.0);
            p.y=std::lerp(p.y,at.y+lift,f*f*(3-2*f));
        };
        for (auto& t : branch.pavementTriangles) for (auto& v : t) fit(v.position,0);
        for (auto& t : branch.edgeTriangles) for (auto& v : t) fit(v.position,0);
        for (auto& [material,tris] : branch.transitionStrips) for (auto& t : tris) for (auto& v : t) fit(v.position,0);
        for (auto* paint : {&branch.whiteStripes,&branch.yellowStripes}) for (auto& t : *paint) for (auto& p : t) fit(p,RoadStyle::stripeHeight);
        for (auto& p : branch.outline) fit(p,0);
    }
}

bool CurvedRoadLink::Valid() const
{
    return station>0 && (side==1 || side==-1) && (direction==1 || direction==-1) && std::isfinite(offset);
}

bool CanHostCurvedConnector(const Road& road)
{
    return !road.style.railroad && road.points.size()>=2 &&
           std::ranges::none_of(road.points,[](const auto& p){return p.curveJoin.has_value();});
}

std::optional<CurvedConnectorPlan> PlanCurvedConnector(const Road& first, int firstSegment, double firstT,
    const Road& second, int secondSegment, double secondT, int nextStation)
{
    if (!CanHostCurvedConnector(first) || !CanHostCurvedConnector(second) || first.id==second.id ||
        nextStation<=0 || nextStation==std::numeric_limits<int>::max())
    { SDL_SetError("Choose two different ordinary roads for the curved connector"); return {}; }
    if (firstSegment < 0 || secondSegment < 0 || firstSegment+1 >= static_cast<int>(first.points.size()) ||
        secondSegment+1 >= static_cast<int>(second.points.size()) || first.points[firstSegment].bridge || second.points[secondSegment].bridge)
    { SDL_SetError("Choose ground-level road segments for the curved connector"); return {}; }
    Road a=first.NewSection(),b=second.NewSection(); a.points=first.points; b.points=second.points;
    a.SetMouths({first.EndpointMouth(true),first.EndpointMouth(false)});
    b.SetMouths({second.EndpointMouth(true),second.EndpointMouth(false)});
    const auto insert=[&](Road& road,int segment,double t,int station)->int
    {
        if (segment<0 || segment+1>=static_cast<int>(road.points.size()) || !(t>=0 && t<=1)) return 0;
        if (t==0 || t==1)
        {
            const auto& p=road.points[segment+(t==1)];
            if (p.Joined()) return 0;
            if (p.rampStation) station=p.rampStation;
        }
        return road.InsertStation(segment,t,station) ? station : 0;
    };
    const int stationA=insert(a,firstSegment,firstT,nextStation),stationB=insert(b,secondSegment,secondT,nextStation+1);
    if (!stationA || !stationB) { SDL_SetError("Choose points farther from existing road endpoints or joins"); return {}; }
    Road road; road.style=first.style; road.style.curved=true; road.style.SetDivided(false);
    road.style.leftShoulder=road.style.rightShoulder=false;
    road.style.oneWay=first.style.oneWay || second.style.oneWay || first.style.IsDivided() || second.style.IsDivided();
    road.style.SetLaneCount(road.style.oneWay ? 1 : std::min({2,first.SegmentLaneCount(firstSegment),second.SegmentLaneCount(secondSegment)}));
    road.style.SetSingleLaneExtraWidth(0);
    road.paintMarkings=first.paintMarkings; road.points.resize(2);
    const HostPath pathA(a),pathB(b);
    CurvedRoadLink la{stationA},lb{stationB};
    auto fa=Attachment(a,pathA,la,road.style),fb=Attachment(b,pathB,lb,road.style);
    if (!fa || !fb) { SDL_SetError("The road is too short or steep for a curved connector"); return {}; }
    auto chord=fb->center-fa->center;
    la.side=Math::Dot(chord,fa->left)>=0 ? 1 : -1;
    lb.side=Math::Dot(chord*-1,fb->left)>=0 ? 1 : -1;
    fa=Attachment(a,pathA,la,road.style); fb=Attachment(b,pathB,lb,road.style);
    chord=fb->center-fa->center;
    if (!ClearsMouths(first,fa->center) || !ClearsMouths(second,fb->center))
    { SDL_SetError("Choose attachment points farther from the intersection"); return {}; }
    la.direction=Math::Dot(chord,fa->forward)>=0 ? 1 : -1;
    lb.direction=Math::Dot(chord,fb->forward)>=0 ? -1 : 1;
    if (road.style.oneWay && (la.direction!=fa->style.TravelDirection(la.side) || -lb.direction!=fb->style.TravelDirection(lb.side)))
    { SDL_SetError("Choose the connector start and end in the direction of traffic"); return {}; }
    road.points.front().curveJoin=la; road.points.back().curveJoin=lb;
    Align(road.points.front(),*fa,true); Align(road.points.back(),*fb,false);
    const double length=Math::Length(chord),width=road.style.LeftWidth()+road.style.RightWidth();
    if (length<4*width) { SDL_SetError("Move the attachment points farther apart to leave room for a smooth curve"); return {}; }
    const auto ta=fa->forward*la.direction,tb=fb->forward*(-lb.direction);
    const double angle=std::acos(std::clamp(Math::Dot(Math::Normalize(ta),Math::Normalize(tb)),-1.0,1.0));
    if (angle>Math::DegreesToRadians(150)) { SDL_SetError("Choose approach points that do not require a hairpin turn"); return {}; }
    const double handle=angle<1e-5 ? 1.0/3.0 : (2.0/3.0)*std::tan(angle*.25)/std::sin(angle*.5);
    road.points.front().outgoingHandle=road.points.back().incomingHandle=handle;
    road.Rebuild();
    return CurvedConnectorPlan{std::move(a.points),std::move(b.points),std::move(road)};
}

bool SlideCurvedConnector(WorldDocument& world, int index, int point, Vector3 destination)
{
    if (index<0 || index>=static_cast<int>(world.roads.size()) || !std::isfinite(Math::Length(destination))) return false;
    auto& road=world.roads[index];
    if (point<0 || point>=static_cast<int>(road.points.size()) || !road.points[point].curveJoin) return false;
    auto& link=*road.points[point].curveJoin;
    for (const auto& host : world.roads)
    {
        if (host.id==road.id) continue;
        const auto knot=std::ranges::find(host.points,link.station,&RoadPoint::rampStation);
        if (knot==host.points.end()) continue;
        const HostPath path(host);
        double best=std::numeric_limits<double>::infinity(),distance=0;
        for (std::size_t i=1;i<path.samples.size();++i)
        {
            const auto& a=path.samples[i-1]; const auto& b=path.samples[i];
            auto at=link; at.offset=a.distance-path.stations.at(link.station);
            const auto first=Attachment(host,path,at,road.EndpointStyle(point==0));
            at.offset=b.distance-path.stations.at(link.station);
            const auto last=Attachment(host,path,at,road.EndpointStyle(point==0));
            if (!first || !last) continue;
            const auto delta=last->center-first->center; const double length=Math::Dot(delta,delta);
            if (length<=0) continue;
            const double t=std::clamp(Math::Dot(destination-first->center,delta)/length,0.0,1.0);
            const double d=Math::Length(destination-(first->center+delta*t));
            if (d<best) { best=d; distance=std::lerp(a.distance,b.distance,t); }
        }
        if (path.samples.size()<2 || !std::isfinite(best)) return false;
        const auto previous=link;
        link.offset=distance-path.stations.at(link.station);
        const auto frame=Attachment(host,path,link,road.EndpointStyle(point==0));
        if (!frame || !ClearsMouths(host,frame->center))
        { link=previous; return SDL_SetError("Keep the attachment outside the intersection"); }
        road.Rebuild(); world.RefreshRampConnections(true);
        const std::size_t neighbor=point==0 ? 1 : road.points.size()-2;
        if (Math::Length(road.points[point].position-road.points[neighbor].position)<2*(road.style.LeftWidth()+road.style.RightWidth()))
        {
            link=previous; road.Rebuild(); world.RefreshRampConnections(true);
            return SDL_SetError("Keep enough space between connector control points for the curve");
        }
        return true;
    }
    return false;
}

void WorldDocument::RefreshCurvedConnectors()
{
    std::map<int,std::size_t> owners,indices;
    for (std::size_t i=0;i<roads.size();++i)
    {
        indices[roads[i].id]=i;
        for (const auto& p : roads[i].points) if (p.rampStation) owners[p.rampStation]=i;
    }
    std::map<std::size_t,HostPath> paths;
    struct Join { std::size_t host,branch; bool first; };
    std::vector<Join> joins;
    std::set<std::pair<int,int>> links;
    std::map<int,std::uint64_t> current;
    for (std::size_t i=0;i<roads.size();++i)
    {
        auto& road=roads[i]; if (road.points.size()<2) continue;
        bool changed=false;
        for (bool first : {true,false})
        {
            auto& p=first ? road.points.front() : road.points.back();
            if (!p.curveJoin || !owners.contains(p.curveJoin->station)) continue;
            const auto h=owners.at(p.curveJoin->station);
            if (h==i || !CanHostCurvedConnector(roads[h])) continue;
            const auto [entry,inserted]=paths.try_emplace(h,roads[h]);
            const auto frame=Attachment(roads[h],entry->second,*p.curveJoin,road.EndpointStyle(first));
            if (!frame) continue;
            const auto before=p;
            Align(p,*frame,first);
            const double direction=p.curveJoin->direction*(first ? 1.0 : -1.0);
            const RoadMouthFrame cap{frame->left*direction,frame->up};
            auto& existing=road.rampCaps[first ? 0 : 1];
            changed |= p!=before || existing!=cap; existing=cap;
            joins.push_back({h,i,first}); links.emplace(roads[h].id,road.id);
        }
        if (changed) road.Rebuild();
    }
    for (const auto& j : joins) { current[roads[j.host].id]=roads[j.host].revision; current[roads[j.branch].id]=roads[j.branch].revision; }
    if (current==curvedConnectorSources && links==curvedConnectorLinks) return;
    std::set<int> affected;
    for (const auto& [id,revision] : current)
        if (!curvedConnectorSources.contains(id) || curvedConnectorSources.at(id)!=revision) affected.insert(id);
    for (const auto& [id,revision] : curvedConnectorSources) if (!current.contains(id)) affected.insert(id);
    auto allLinks=links; allLinks.insert(curvedConnectorLinks.begin(),curvedConnectorLinks.end());
    for (const auto& pair : allLinks) if (links.contains(pair)!=curvedConnectorLinks.contains(pair))
    { affected.insert(pair.first); affected.insert(pair.second); }
    bool expanded;
    do
    {
        expanded=false;
        for (const auto& [a,b] : allLinks) if (affected.contains(a) || affected.contains(b))
        { expanded |= affected.insert(a).second; expanded |= affected.insert(b).second; }
    } while (expanded);
    for (int id : affected) if (indices.contains(id)) roads[indices.at(id)].Rebuild();
    // Always derive joins from fresh authored geometry. Idle frames do nothing,
    // and deleting a connector restores the main road's paint and edge treatment.
    for (const auto& j : joins)
    {
        auto& host=roads[j.host]; auto& branch=roads[j.branch];
        if (!affected.contains(host.id)) continue;
        const auto region=JoinRegion(branch,j.first);
        Conform(branch,host,region,false);
        const auto hostCut=SurfaceCutters(host.pavementTriangles,region);
        const auto branchCut=SurfaceCutters(branch.pavementTriangles,region);
        Cut(host.edgeTriangles,branchCut);
        for (auto& [material,tris] : host.transitionStrips) Cut(tris,branchCut);
        CutPaint(host.whiteStripes,branchCut);
        TrimBed(branch,hostCut);
        CutPaint(branch.whiteStripes,hostCut); CutPaint(branch.yellowStripes,hostCut);
        Conform(branch,host,region,true);
    }
    curvedConnectorSources.clear();
    for (const auto& [id,revision] : current)
    {
        auto& road=roads[indices.at(id)];
        if (affected.contains(id)) { road.MarkRebuilt(); road.UpdateBounds(); }
        curvedConnectorSources[id]=road.revision;
    }
    curvedConnectorLinks=std::move(links);
}

std::vector<std::array<Vector3, 2>> CurvedConnectorTerrainSeams(const Road& host, const Road& branch)
{
    std::vector<std::array<Vector3,2>> result;
    std::vector<Box> regions;
    for (bool first : {true,false})
    {
        const auto& p=first ? branch.points.front() : branch.points.back();
        if (p.curveJoin && std::ranges::any_of(host.points,[&](const auto& station){return station.rampStation==p.curveJoin->station;}))
            regions.push_back(JoinRegion(branch,first));
    }
    const auto cross=[](Vector3 a,Vector3 b){return a.x*b.z-a.z*b.x;};
    for (std::size_t i=0;i<host.outline.size();++i) for (std::size_t j=0;j<branch.outline.size();++j)
    {
        const auto a=host.outline[i],b=branch.outline[j];
        const auto u=host.outline[(i+1)%host.outline.size()]-a,v=branch.outline[(j+1)%branch.outline.size()]-b;
        const double d=cross(u,v); if (std::abs(d)<1e-10) continue;
        const double t=cross(b-a,v)/d,s=cross(b-a,u)/d;
        if (t<0 || t>1 || s<0 || s>1) continue;
        auto p=a+u*t,q=b+v*s; q.x=p.x; q.z=p.z;
        if (std::ranges::none_of(regions,[&](const Box& box){return box.Overlaps(Box(p));})) continue;
        result.push_back(p.y<q.y ? std::array{p,q} : std::array{q,p});
    }
    return result;
}
