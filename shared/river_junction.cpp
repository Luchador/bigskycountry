#include "river_junction.hpp"
#include <CDT.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace
{
    double Cross(Vector3 a, Vector3 b) { return a.x*b.z-a.z*b.x; }
    double Distance(Vector3 a, Vector3 b) { return std::hypot(a.x-b.x,a.z-b.z); }
    double Along(Vector3 p, Vector3 a, Vector3 b)
    {
        auto d=b-a; const double length=d.x*d.x+d.z*d.z;
        return length>1e-16 ? std::clamp(((p.x-a.x)*d.x+(p.z-a.z)*d.z)/length,0.0,1.0) : 0;
    }
    bool Inside(Vector3 p, const std::vector<LakeSample>& rim)
    {
        bool inside=false;
        for (std::size_t i=1;i<rim.size();++i)
        {
            const auto a=rim[i-1].position,b=rim[i].position;
            if ((a.z>p.z)!=(b.z>p.z) && p.x<(b.x-a.x)*(p.z-a.z)/(b.z-a.z)+a.x) inside=!inside;
        }
        return inside;
    }
    std::vector<Vector3> Bank(const RiverJunction::Source& source, int bank)
    {
        std::vector<Vector3> result;
        // Stable, dense arc-length parameterization, independent of render subdivision.
        for (int segment=0;segment+1<static_cast<int>(source.definition.banks[bank].size());++segment)
            for (int j=0;j<64;++j) result.push_back(source.transform.Point(source.definition.Evaluate(bank,segment,j/64.0)));
        result.push_back(source.transform.Point(source.definition.banks[bank].back().position));
        return result;
    }
    double Length(const std::vector<Vector3>& bank)
    {
        double length=0; for (std::size_t i=1;i<bank.size();++i) length+=Distance(bank[i-1],bank[i]); return length;
    }
    Vector3 At(const std::vector<Vector3>& bank, double distance, Vector3* tangent=nullptr)
    {
        for (std::size_t i=1;i<bank.size();++i)
        {
            const double length=Distance(bank[i-1],bank[i]);
            if (distance<=length || i+1==bank.size())
            {
                if (tangent) *tangent=Math::Normalize(Vector3{bank[i].x-bank[i-1].x,0,bank[i].z-bank[i-1].z});
                return bank[i-1]+(bank[i]-bank[i-1])*(length>0 ? std::clamp(distance/length,0.0,1.0) : 0);
            }
            distance-=length;
        }
        return bank.back();
    }
    struct Surface
    {
        std::vector<LakeSample> rim;
        CDT::Triangulation<double> cdt;
        double depth;
        std::optional<double> Height(Vector3 p) const
        {
            for (const auto& tri:cdt.triangles)
            {
                const auto a=rim[tri.vertices[0]].position,b=rim[tri.vertices[1]].position,c=rim[tri.vertices[2]].position;
                if (p.x<std::min({a.x,b.x,c.x})-1e-6 || p.x>std::max({a.x,b.x,c.x})+1e-6 ||
                    p.z<std::min({a.z,b.z,c.z})-1e-6 || p.z>std::max({a.z,b.z,c.z})+1e-6) continue;
                const double area=Cross(b-a,c-a);
                if (std::abs(area)<1e-12) continue;
                const double u=Cross(b-p,c-p)/area,v=Cross(c-p,a-p)/area,w=1-u-v;
                if (u>=-1e-7 && v>=-1e-7 && w>=-1e-7) return a.y*u+b.y*v+c.y*w;
            }
            return {};
        }
    };
}

std::optional<RiverConnection> RiverJunction::Locate(const Source& tributary, int end, const Source& receiver,
                                                     std::uint64_t receiverId, Vector3 position)
{
    if (end<0 || end>1 || !receiverId) return {};
    RiverConnection result; result.riverId=receiverId; result.end=end;
    const auto& banks=tributary.definition.banks;
    result.width=Distance(tributary.transform.Point(end?banks[0].back().position:banks[0].front().position),
                          tributary.transform.Point(end?banks[1].back().position:banks[1].front().position));
    double closest=std::numeric_limits<double>::max();
    for (int bank=0;bank<2;++bank)
    {
        auto samples=Bank(receiver,bank); double distance=0,total=Length(samples);
        for (std::size_t i=1;i<samples.size();++i)
        {
            const auto a=samples[i-1],b=samples[i]; const double t=Along(position,a,b),score=Distance(position,a+(b-a)*t);
            if (score<closest) { closest=score; result.bank=bank; result.fraction=(distance+Distance(a,b)*t)/total; }
            distance+=Distance(a,b);
        }
    }
    if (!std::isfinite(result.width) || result.width<.5 || result.width>10000 || result.fraction<=0 || result.fraction>=1)
    { SDL_SetError("Choose a receiving bank away from the river ends"); return {}; }
    return result;
}

bool RiverJunction::Attach(Source& tributary, const Source& receiver)
{
    const auto& link=*tributary.definition.connection;
    auto bank=Bank(receiver,link.bank); Vector3 tangent;
    const double total=Length(bank),distance=total*link.fraction;
    if (distance<link.width || total-distance<link.width)
        return SDL_SetError("Leave at least one tributary width between the junction and the receiving river ends");
    auto mouth=At(bank,distance,&tangent);
    auto opposite=Bank(receiver,1-link.bank);
    double closest=std::numeric_limits<double>::max(); Vector3 across;
    for (std::size_t i=1;i<opposite.size();++i)
    {
        const auto p=opposite[i-1]+(opposite[i]-opposite[i-1])*Along(mouth,opposite[i-1],opposite[i]);
        if (Distance(p,mouth)<closest) { closest=Distance(p,mouth); across=p-mouth; }
    }
    Vector3 inward{-tangent.z,0,tangent.x}; if (Math::Dot(inward,across)<0) inward=inward*-1;
    // The mouth overlaps the receiving channel; the common triangulation removes
    // the overlap and its internal bank. Only the tributary's terminal points move.
    const double inset=std::min(closest*.35,std::max(1.0,link.width*.2));
    auto& banks=tributary.definition.banks;
    const auto old0=tributary.transform.Point(link.end?banks[0].back().position:banks[0].front().position);
    const auto old1=tributary.transform.Point(link.end?banks[1].back().position:banks[1].front().position);
    if (Math::Dot(tangent,old1-old0)<0) tangent=tangent*-1;
    const auto receiverRim=RiverGeometry::Sample(receiver.definition);
    if (!receiverRim) return false;
    for (int side=0;side<2;++side)
    {
        auto world=mouth+inward*inset+tangent*((side?1:-1)*link.width*.5);
        const auto localReceiver=receiver.transform.Unrotate(world-receiver.transform.origin);
        if (!Inside(localReceiver,*receiverRim)) return SDL_SetError("The tributary mouth does not fit here; choose a wider or straighter receiving bank");
        auto& points=banks[side]; const auto index=link.end?points.size()-1:0;
        auto& point=points[index]; const auto neighbor=points[link.end?index-1:1].position;
        point.position=tributary.transform.Unrotate(world-tributary.transform.origin);
        const auto handle=(neighbor-point.position)*(1.0/3);
        point.incoming=link.end?handle:Vector3{};
        point.outgoing=link.end?Vector3{}:handle;
    }
    return true;
}

std::optional<std::vector<LakeGeometry>> RiverJunction::Build(std::span<const Source> sources)
{
    using Index=CDT::VertInd; using Edge=CDT::Edge; using Triangle=std::array<Index,3>;
    if (sources.empty()) { SDL_SetError("No rivers to connect"); return {}; }
    try
    {
        const auto origin=sources.front().transform.origin;
        std::vector<Surface> surfaces;
        std::vector<CDT::V2d<double>> vertices;
        std::map<std::pair<double,double>,Index> lookup;
        const auto add=[&](Vector3 p) {
            // Weld topology only, in local coordinates, to avoid continent-scale precision loss.
            auto key=std::pair{std::round(p.x*1e7),std::round(p.z*1e7)};
            const auto [it,inserted]=lookup.try_emplace(key,static_cast<Index>(vertices.size()));
            if (inserted) vertices.push_back({key.first/1e7,key.second/1e7});
            return it->second;
        };
        std::vector<Edge> edges;
        for (const auto& source:sources)
        {
            auto sampled=RiverGeometry::Sample(source.definition);
            if (!sampled) return {};
            auto& surface=surfaces.emplace_back(); surface.depth=source.definition.depth; surface.rim=std::move(*sampled);
            std::vector<CDT::V2d<double>> points; std::vector<Edge> boundary;
            Bounds3 bounds;
            for (auto& p:surface.rim) { p.position=source.transform.Point(p.position)-origin; bounds.Include(p.position); }
            const auto count=static_cast<Index>(surface.rim.size()-1);
            for (Index i=0;i<count;++i)
            {
                auto a=surface.rim[i].position,b=surface.rim[i+1].position;
                points.push_back({a.x,a.z}); boundary.emplace_back(i,(i+1)%count);
                const auto first=add(a),second=add(b); if (first!=second) edges.emplace_back(first,second);
            }
            surface.cdt.insertVertices(points); surface.cdt.insertEdges(boundary); surface.cdt.eraseOuterTrianglesAndHoles();
            const double spacing=std::max(5.0,std::max(bounds.max.x-bounds.min.x,bounds.max.z-bounds.min.z)/96);
            for (double z=bounds.min.z+spacing*.5;z<bounds.max.z;z+=spacing)
                for (double x=bounds.min.x+spacing*.5;x<bounds.max.x;x+=spacing)
                    if (Inside({x,0,z},surface.rim)) add({x,0,z});
        }
        CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::Auto,CDT::IntersectingConstraintEdges::TryResolve,1e-8);
        cdt.insertVertices(vertices); cdt.insertEdges(edges); cdt.eraseSuperTriangle();
        std::vector<Vector3> positions;
        for (const auto p:cdt.vertices) positions.push_back({p.x,0,p.y});
        std::vector<int> owners(cdt.triangles.size(),-1);
        for (std::size_t i=0;i<cdt.triangles.size();++i)
        {
            const auto& t=cdt.triangles[i]; const auto p=(positions[t.vertices[0]]+positions[t.vertices[1]]+positions[t.vertices[2]])*(1.0/3);
            for (int j=0;j<static_cast<int>(surfaces.size());++j) if (Inside(p,surfaces[j].rim)) { owners[i]=j; break; }
        }
        std::vector<std::array<Vector3,2>> rim;
        std::set<Edge> exposed;
        for (std::size_t i=0;i<cdt.triangles.size();++i) if (owners[i]>=0)
        {
            const auto& t=cdt.triangles[i];
            for (int e=0;e<3;++e) if (t.neighbors[e]==CDT::noNeighbor || owners[t.neighbors[e]]<0)
            {
                auto a=t.vertices[e],b=t.vertices[(e+1)%3]; exposed.emplace(a,b); rim.push_back({positions[a],positions[b]});
            }
        }
        // Subdivide every internal edge and face. Even narrow arms have submerged
        // interior vertices, including the former bank across the tributary mouth.
        std::map<Edge,Index> midpoints;
        std::vector<std::pair<int,Triangle>> triangles;
        for (std::size_t i=0;i<cdt.triangles.size();++i) if (owners[i]>=0)
        {
            const auto& t=cdt.triangles[i]; std::vector<Index> ring;
            for (int e=0;e<3;++e)
            {
                auto a=t.vertices[e],b=t.vertices[(e+1)%3]; ring.push_back(a);
                if (exposed.contains(Edge(a,b))) continue;
                const auto [it,added]=midpoints.try_emplace(Edge(a,b),static_cast<Index>(positions.size()));
                if (added) positions.push_back((positions[a]+positions[b])*.5);
                ring.push_back(it->second);
            }
            const auto center=static_cast<Index>(positions.size());
            positions.push_back((positions[t.vertices[0]]+positions[t.vertices[1]]+positions[t.vertices[2]])*(1.0/3));
            for (std::size_t e=0;e<ring.size();++e) triangles.push_back({owners[i],{ring[e],ring[(e+1)%ring.size()],center}});
        }
        std::vector<Vector3> bottom(positions.size()),waterNormals(positions.size()),bedNormals(positions.size());
        for (std::size_t i=0;i<positions.size();++i)
        {
            auto& p=positions[i]; double depth=0;
            for (const auto& surface:surfaces) if (auto height=surface.Height(p)) { p.y=*height; depth=surface.depth; break; }
            double shore=std::numeric_limits<double>::max();
            for (const auto& edge:rim) shore=std::min(shore,Distance(p,edge[0]+(edge[1]-edge[0])*Along(p,edge[0],edge[1])));
            bottom[i]=p; bottom[i].y-=std::min(depth,shore*.25);
        }
        for (auto& [owner,t]:triangles)
        {
            if (Math::Cross(positions[t[1]]-positions[t[0]],positions[t[2]]-positions[t[0]]).y<0) std::swap(t[1],t[2]);
            const auto wn=Math::Cross(positions[t[1]]-positions[t[0]],positions[t[2]]-positions[t[0]]);
            const auto bn=Math::Cross(bottom[t[1]]-bottom[t[0]],bottom[t[2]]-bottom[t[0]]);
            for (auto v:t) { waterNormals[v]=waterNormals[v]+wn; bedNormals[v]=bedNormals[v]+bn; }
        }
        std::vector<LakeGeometry> result(sources.size());
        for (std::size_t s=0;s<sources.size();++s)
        {
            auto water=std::make_shared<Mesh>(),bed=std::make_shared<Mesh>(); std::map<Index,Uint32> local;
            const auto& transform=sources[s].transform;
            for (const auto& [owner,t]:triangles) if (owner==static_cast<int>(s)) for (auto v:t)
            {
                const auto [it,added]=local.try_emplace(v,static_cast<Uint32>(water->vertices.size()));
                if (added)
                {
                    const auto p=transform.Unrotate(positions[v]+origin-transform.origin),b=transform.Unrotate(bottom[v]+origin-transform.origin);
                    const SDL_FPoint uv{float(p.x/4),float(p.z/4)};
                    water->vertices.push_back({p,transform.Unrotate(Math::Normalize(waterNormals[v])),uv,{1,1,1,1}});
                    bed->vertices.push_back({b,transform.Unrotate(Math::Normalize(bedNormals[v])),uv,{.24f,.20f,.12f,1}});
                }
                water->indices.push_back(it->second); bed->indices.push_back(it->second);
            }
            if (water->indices.empty()) throw std::runtime_error("A river is completely covered by another river");
            water->UpdateBounds(); bed->UpdateBounds(); result[s].water=water; result[s].bed=bed;
            // Terrain follows the same split constraint vertices and elevations,
            // including where two banks intersect. Overlapping interiors exclude land.
            const auto& sourceRim=surfaces[s].rim;
            for (std::size_t e=1;e<sourceRim.size();++e)
            {
                auto a=sourceRim[e-1].position,b=sourceRim[e].position;
                std::vector<std::pair<double,Index>> onEdge;
                for (Index v=0;v<cdt.vertices.size();++v)
                {
                    const double t=Along(positions[v],a,b);
                    if (t<1-1e-9 && Distance(positions[v],a+(b-a)*t)<2e-6) onEdge.push_back({t,v});
                }
                std::sort(onEdge.begin(),onEdge.end());
                for (const auto& [t,v]:onEdge)
                {
                    auto sample=sourceRim[e-1]; sample.position=transform.Unrotate(positions[v]+origin-transform.origin);
                    result[s].shoreline.push_back(sample);
                }
            }
            if (result[s].shoreline.empty()) throw std::runtime_error("Invalid river junction shoreline");
            result[s].shoreline.push_back(result[s].shoreline.front());
        }
        return result;
    }
    catch (const std::exception& e) { SDL_SetError("Cannot build river junction: %s",e.what()); return {}; }
}
