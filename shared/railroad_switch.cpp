#include "railroad_switch.hpp"
#include "world_document.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <set>

namespace
{
    constexpr double headRise = RailroadSettings::tieHeight + RailroadSettings::railHeight;
    void SetDirection(RoadPoint& point, Vector3 forward)
    {
        point.heading = std::atan2(forward.z, forward.x);
        point.pitch = std::atan2(forward.y, std::hypot(forward.x, forward.z));
    }

    struct Box
    {
        Vector3 lo, hi;
        explicit Box(Vector3 p) : lo(p), hi(p) {}
        void Add(Vector3 p)
        {
            lo = {std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
            hi = {std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
        }
        bool Overlaps(const Box& b) const
        { return hi.x >= b.lo.x && lo.x <= b.hi.x && hi.z >= b.lo.z && lo.z <= b.hi.z; }
    };
    template<class V> Box Bounds(const std::vector<std::array<V,3>>& triangles)
    {
        Box box(triangles.empty() ? Vector3{} : triangles.front()[0].position);
        for (const auto& tri : triangles) for (const auto& v : tri) box.Add(v.position);
        return box;
    }
    struct Plane { Vector3 origin, normal; double bias = 0; };
    template<class V> V Mix(const V& a, const V& b, double t)
    {
        V v = a;
        v.position = a.position + (b.position-a.position)*t;
        if constexpr (requires { v.uv; })
            v.uv = {std::lerp(a.uv.x,b.uv.x,static_cast<float>(t)),std::lerp(a.uv.y,b.uv.y,static_cast<float>(t))};
        if constexpr (requires { v.color; })
            v.color = {std::lerp(a.color.r,b.color.r,static_cast<float>(t)), std::lerp(a.color.g,b.color.g,static_cast<float>(t)),
                       std::lerp(a.color.b,b.color.b,static_cast<float>(t)), std::lerp(a.color.a,b.color.a,static_cast<float>(t))};
        return v;
    }
    template<class V> std::vector<V> Half(const std::vector<V>& polygon, const Plane& plane, bool inside)
    {
        std::vector<V> out;
        if (polygon.empty()) return out;
        auto a = polygon.back();
        double da = Math::Dot(a.position-plane.origin,plane.normal)+plane.bias;
        for (const auto& b : polygon)
        {
            const double db = Math::Dot(b.position-plane.origin,plane.normal)+plane.bias;
            if ((da <= 0) != (db <= 0)) out.push_back(Mix(a,b,std::clamp(da/(da-db),0.0,1.0)));
            if ((db <= 0) == inside) out.push_back(b);
            a=b; da=db;
        }
        return out;
    }
    template<class V> void Emit(std::vector<std::array<V,3>>& out, const std::vector<V>& polygon)
    {
        for (std::size_t i=2;i<polygon.size();++i)
            if (Math::Length(Math::Cross(polygon[i-1].position-polygon[0].position,polygon[i].position-polygon[0].position))>1e-12)
                out.push_back({polygon[0],polygon[i-1],polygon[i]});
    }
    // Difference against a convex prism. Each outside piece is final; only the
    // remaining inside polygon proceeds to the next plane, so no cracks/overlaps.
    template<class V> void Subtract(std::vector<std::vector<V>>& output, const std::vector<V>& polygon, std::span<const Plane> planes)
    {
        // Reject a cutter before partitioning if it does not actually reach
        // this triangle. Merely crossing its infinite planes must not tessellate
        // the rest of a long railroad into thousands of tiny fragments.
        auto overlap=polygon;
        for (const auto& plane : planes)
        {
            overlap=Half(overlap,plane,true);
            if (overlap.empty()) { output.push_back(polygon); return; }
        }
        auto inside=polygon;
        for (const auto& plane : planes)
        {
            auto outside=Half(inside,plane,false);
            if (outside.size()>=3) output.push_back(std::move(outside));
            inside=Half(inside,plane,true);
            if (inside.empty()) break;
        }
    }
    struct Cutter { Box box; std::vector<Plane> planes; };
    template<class V> void Cut(std::vector<std::array<V,3>>& surface, const std::vector<Cutter>& cutters)
    {
        if (surface.empty() || cutters.empty()) return;
        std::vector<std::array<V,3>> out;
        for (const auto& tri : surface)
        {
            Box box(tri[0].position); box.Add(tri[1].position); box.Add(tri[2].position);
            std::vector<std::vector<V>> pieces{{tri.begin(),tri.end()}};
            for (const auto& cutter : cutters)
            {
                if (!box.Overlaps(cutter.box)) continue;
                std::vector<std::vector<V>> next;
                for (const auto& piece : pieces) Subtract(next,piece,cutter.planes);
                pieces=std::move(next);
                if (pieces.empty()) break;
            }
            for (const auto& polygon : pieces) Emit(out,polygon);
        }
        surface=std::move(out);
    }
    std::vector<Cutter> BedCutters(const Road& source, const Box& region, bool equal)
    {
        std::vector<Cutter> result;
        const auto append = [&](const auto& tris)
        {
            for (const auto& tri : tris)
            {
                const auto a=tri[0].position,b=tri[1].position,c=tri[2].position;
                Box box(a); box.Add(b); box.Add(c);
                if (!box.Overlaps(region)) continue;
                auto normal=Math::Cross(b-a,c-a);
                if (std::abs(normal.y)<1e-10) continue;
                if (normal.y<0) normal=normal*-1;
                Cutter cut{box,{}};
                const std::array<Vector3,3> p{a,b,c};
                for (int i=0;i<3;++i)
                {
                    auto edge=p[(i+1)%3]-p[i];
                    auto n=Vector3{-edge.z,0,edge.x};
                    if (Math::Dot(p[(i+2)%3]-p[i],n)>0) n=n*-1;
                    cut.planes.push_back({p[i],Math::Normalize(n)});
                }
                cut.planes.push_back({a,Math::Normalize(normal),equal ? -1e-7 : 1e-7});
                result.push_back(std::move(cut));
            }
        };
        append(source.pavementTriangles);
        for (const auto& [material,triangles] : source.transitionStrips) append(triangles);
        return result;
    }
    void TrimBed(Road& road, const std::vector<Cutter>& cutters)
    {
        Cut(road.pavementTriangles,cutters);
        Cut(road.edgeTriangles,cutters);
        for (auto& [material,triangles] : road.transitionStrips) Cut(triangles,cutters);
    }

    // Intersect the other track with the sleeper's transverse plane. This keeps
    // shared timbers perpendicular to the main track even through a curved turnout.
    std::optional<Vector3> Across(const Road& other, const MeshTransform& sleeper)
    {
        std::optional<Vector3> result;
        double nearest=RailroadSettings::tieLength*1.1;
        for (std::size_t i=1;i<other.trackSections.size();++i)
        {
            const auto a=other.trackSections[i-1].top,b=other.trackSections[i].top;
            const double da=Math::Dot(a-sleeper.origin,sleeper.x),db=Math::Dot(b-sleeper.origin,sleeper.x);
            if ((da<0)==(db<0) || std::abs(da-db)<1e-10) continue;
            const auto p=a+(b-a)*(da/(da-db))-sleeper.y*headRise;
            const auto delta=p-sleeper.origin;
            const double lateral=std::abs(Math::Dot(delta,sleeper.z));
            if (lateral<nearest && std::abs(Math::Dot(delta,sleeper.y))<0.08) { nearest=lateral; result=p; }
        }
        return result;
    }
    void JoinSleepers(Road& host, Road& branch)
    {
        for (auto& detail : host.railroadSegments)
        {
            std::vector<RailroadTieRun> ties;
            for (const auto& run : detail.ties)
            {
                // Preserve instanced runs outside the small turnout footprint.
                bool nearby=false;
                for (int i=0;i<run.count && !nearby;++i) nearby=Across(branch,run.At(i)).has_value();
                if (!nearby) { ties.push_back(run); continue; }
                for (int i=0;i<run.count;++i)
                {
                    auto frame=run.At(i);
                    if (const auto p=Across(branch,frame))
                    {
                        const double side=Math::Dot(*p-frame.origin,frame.z);
                        frame.origin=frame.origin+frame.z*(side*.5);
                        frame.z=frame.z*(1+std::abs(side)/RailroadSettings::tieLength);
                    }
                    ties.push_back({frame,frame,1});
                }
            }
            detail.ties=std::move(ties);
        }
        for (auto& detail : branch.railroadSegments)
        {
            std::vector<RailroadTieRun> ties;
            for (const auto& run : detail.ties)
            {
                int first=0;
                for (int i=0;i<=run.count;++i)
                {
                    const bool omit=i==run.count || Across(host,run.At(i)).has_value();
                    if (!omit) continue;
                    if (i>first) ties.push_back({run.At(first),run.At(i-1),i-first});
                    first=i+1;
                }
            }
            detail.ties=std::move(ties);
        }
    }

    std::vector<Cutter> RailCutters(const Road& host, const Box& region)
    {
        std::vector<Cutter> cuts;
        const double center=(RailroadSettings::gauge+RailroadSettings::headWidth)*.5;
        for (std::size_t i=1;i<host.trackSections.size();++i)
        {
            const auto& a=host.trackSections[i-1]; const auto& b=host.trackSections[i];
            auto along=Math::Normalize(b.top-a.top);
            auto across=Math::Normalize(a.side+b.side);
            auto up=Math::Normalize(Math::Cross(across,along));
            if (along==Vector3{} || across==Vector3{} || up==Vector3{}) continue;
            for (int side : {-1,1})
            {
                const auto p=a.top+a.side*(side*center),q=b.top+b.side*(side*center);
                Box box(p); box.Add(q);
                box.lo=box.lo-Vector3{.2,.2,.2}; box.hi=box.hi+Vector3{.2,.2,.2};
                if (!box.Overlaps(region)) continue;
                // Foot, web and head remain actual solids, including the taper
                // where a branch blade emerges from its stock rail.
                for (const auto profile : {Vector3{.145,-.17,-.15},Vector3{.018,-.15,-.05},Vector3{.075,-.05,0}})
                {
                    cuts.push_back({box,{{p,along*-1},{q,along},
                        {p+across*(profile.x*.5),across},{p-across*(profile.x*.5),across*-1},
                        {p+up*profile.y,up*-1},{p+up*profile.z,up}}});
                }
            }
        }
        return cuts;
    }
}

MeshTransform RailroadSwitchFrame(const Road& road, std::size_t point, int direction)
{
    const auto reference=ObjectTransform{{},road.rotation}.ToMeshTransform();
    const auto forward=road.Tangent(point)*static_cast<double>(direction);
    const auto side=Math::Normalize(Math::Cross(forward,reference.y));
    const auto up=Math::Cross(side,forward);
    return {road.points[point].position+up*(road.style.railroad->height+headRise),forward,up,side};
}

std::optional<RailroadSwitchPlan> PlanRailroadSwitch(const Road& host, int segment, double t,
                                                    int side, int direction, int station)
{
    if (!host.style.railroad || host.HasOverpasses() || host.points.size()<2 || segment<0 ||
        segment+1>=static_cast<int>(host.points.size()) || !(t>=0 && t<=1) ||
        (side!=-1 && side!=1) || (direction!=-1 && direction!=1) || station<=0) return {};
    if ((segment==0 && t==0 && direction<0) ||
        (segment+2==static_cast<int>(host.points.size()) && t==1 && direction>0)) return {};
    if ((t==0 || t==1) && (host.points[segment+(t==1)].Joined() ||
        (host.points[segment+(t==1)].rampStation && host.points[segment+(t==1)].rampStation!=station))) return {};
    Road staged=host.NewSection(); staged.points=host.points;
    if (!staged.InsertStation(segment,t,station)) return {};
    const auto knot=std::find_if(staged.points.begin(),staged.points.end(),[&](const auto& p){return p.rampStation==station;});
    const auto frame=RailroadSwitchFrame(staged,static_cast<std::size_t>(knot-staged.points.begin()),direction);
    if (frame.z==Vector3{}) return {};
    Road branch=host.NewSection();
    branch.style.curved=true; branch.paintMarkings=false;
    branch.points={{knot->position},{knot->position+frame.x*50.0+frame.z*(side*8.0)}};
    branch.points.front().railSwitch=RailroadSwitchLink{station,direction};
    SetDirection(branch.points.front(),frame.x);
    branch.Rebuild();
    return RailroadSwitchPlan{std::move(staged.points),std::move(branch)};
}

void WorldDocument::AlignRailroadSwitches()
{
    struct Source { std::size_t road, point; };
    std::map<int,Source> stations;
    for (std::size_t i=0;i<roads.size();++i)
    {
        if (roads[i].style.railroad)
            for (std::size_t p=0;p<roads[i].points.size();++p)
                if (roads[i].points[p].rampStation) stations.emplace(roads[i].points[p].rampStation,Source{i,p});
    }
    // Resolve parents first; file/outliner order is immaterial. Guard malformed
    // cyclic input here too, before the document's connection validator runs.
    std::set<std::size_t> visiting,done;
    std::function<void(std::size_t)> align=[&](std::size_t index)
    {
        if (done.contains(index) || visiting.contains(index)) return;
        visiting.insert(index);
        auto& road=roads[index];
        bool changed=false;
        if (road.style.railroad && road.points.size()>=2)
            for (bool first : {true,false})
            {
                auto& point=first ? road.points.front() : road.points.back();
                if (!point.railSwitch) continue;
                const auto source=stations.find(point.railSwitch->station);
                if (source==stations.end() || source->second.road==index) continue;
                align(source->second.road);
                const auto frame=RailroadSwitchFrame(roads[source->second.road],source->second.point,point.railSwitch->direction);
                auto before=point;
                point.position=frame.origin-frame.y*(road.style.railroad->height+headRise);
                SetDirection(point,frame.x*(first ? 1.0 : -1.0));
                // Match bank as well as grade at the shared rail heads.
                if (road.rotation!=roads[source->second.road].rotation)
                { road.rotation=roads[source->second.road].rotation; changed=true; }
                changed |= before!=point;
            }
        if (changed) road.Rebuild();
        visiting.erase(index); done.insert(index);
    };
    for (std::size_t i=0;i<roads.size();++i) align(i);
}

void WorldDocument::RefreshRailroadSwitches()
{
    struct Source { std::size_t road, point; };
    std::map<int,Source> stations;
    std::map<int,std::size_t> indices;
    for (std::size_t i=0;i<roads.size();++i)
    {
        indices[roads[i].id]=i;
        if (roads[i].style.railroad)
            for (std::size_t p=0;p<roads[i].points.size();++p)
                if (roads[i].points[p].rampStation) stations.emplace(roads[i].points[p].rampStation,Source{i,p});
    }
    std::vector<std::pair<std::size_t,std::size_t>> joins;
    std::map<int,std::uint64_t> current;
    for (std::size_t i=0;i<roads.size();++i)
        for (const auto& point : roads[i].points)
            if (point.railSwitch)
                if (const auto s=stations.find(point.railSwitch->station);s!=stations.end() && s->second.road!=i)
                {
                    joins.emplace_back(s->second.road,i);
                    current[roads[i].id]=roads[i].revision;
                    current[roads[s->second.road].id]=roads[s->second.road].revision;
                }
    std::ranges::sort(joins);
    joins.erase(std::unique(joins.begin(),joins.end()),joins.end());
    std::set<std::pair<int,int>> links;
    for (const auto [host,branch] : joins) links.emplace(roads[host].id,roads[branch].id);
    if (current==switchSources && links==switchLinks) return;
    std::set<int> affected;
    for (const auto& [id,revision] : current)
        if (!switchSources.contains(id) || switchSources.at(id)!=revision) affected.insert(id);
    for (const auto& [id,revision] : switchSources)
        if (!current.contains(id)) affected.insert(id);
    auto allLinks=links;
    allLinks.insert(switchLinks.begin(),switchLinks.end());
    for (const auto& link : allLinks)
        if (links.contains(link)!=switchLinks.contains(link)) { affected.insert(link.first); affected.insert(link.second); }
    // A changed turnout invalidates its connected geometry, not unrelated
    // switches elsewhere in the world. Removed links participate in restoration.
    bool expanded;
    do
    {
        expanded=false;
        for (const auto [a,b] : allLinks)
            if (affected.contains(a) || affected.contains(b))
            { expanded |= affected.insert(a).second; expanded |= affected.insert(b).second; }
    } while (expanded);
    // Rebuild from authored splines before trimming. Repeated edits never add
    // triangles to a previously trimmed mesh, and deletion restores the stock track.
    for (int id : affected)
        if (indices.contains(id)) roads[indices.at(id)].Rebuild();
    for (const auto [hostIndex,branchIndex] : joins)
    {
        auto& host=roads[hostIndex]; auto& branch=roads[branchIndex];
        if (!affected.contains(host.id)) continue;
        const auto region=Bounds(branch.pavementTriangles);
        const auto hostCut=BedCutters(host,region,true);
        const auto branchCut=BedCutters(branch,Bounds(host.pavementTriangles),false);
        TrimBed(branch,hostCut); TrimBed(host,branchCut);
        const auto rails=RailCutters(host,region);
        for (auto& detail : branch.railroadSegments) Cut(detail.rails,rails);
        JoinSleepers(host,branch);
    }
    switchSources.clear();
    for (const auto& [id,revision] : current)
    {
        auto& road=roads[indices.at(id)];
        if (affected.contains(id)) { road.MarkRebuilt(); road.UpdateBounds(); }
        switchSources[id]=road.revision;
    }
    switchLinks=std::move(links);
}

std::vector<std::array<Vector3, 2>> RailroadSwitchTerrainSeams(const Road& host, const Road& branch)
{
    std::vector<std::array<Vector3, 2>> result;
    const auto cross=[](Vector3 a,Vector3 b){return a.x*b.z-a.z*b.x;};
    for (std::size_t i=0;i<host.outline.size();++i)
        for (std::size_t j=0;j<branch.outline.size();++j)
        {
            const auto a=host.outline[i],b=branch.outline[j];
            const auto u=host.outline[(i+1)%host.outline.size()]-a,v=branch.outline[(j+1)%branch.outline.size()]-b;
            const double denominator=cross(u,v);
            if (std::abs(denominator)<1e-10) continue;
            const double t=cross(b-a,v)/denominator,s=cross(b-a,u)/denominator;
            if (t<0 || t>1 || s<0 || s>1) continue;
            auto p=a+u*t,q=b+v*s;
            // Only the connected approach, not an unrelated crossing farther along a looping branch.
            bool near=false;
            for (bool first : {true,false})
            {
                const auto k=first ? 0 : branch.points.size()-1;
                const auto& end=branch.points[k];
                if (!end.railSwitch) continue;
                if (std::ranges::none_of(host.points,[&](const auto& knot){return knot.rampStation==end.railSwitch->station;})) continue;
                const double reach=Math::Length(branch.points[first ? 1 : k-1].position-end.position)+
                                   host.style.railroad->ToeHalfWidth()+branch.style.railroad->ToeHalfWidth();
                near |= std::hypot(p.x-end.position.x,p.z-end.position.z)<=reach;
            }
            if (!near) continue;
            q.x=p.x; q.z=p.z;
            result.push_back(p.y<q.y ? std::array{p,q} : std::array{q,p});
        }
    return result;
}
