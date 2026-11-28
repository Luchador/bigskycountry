#include "terrain_sculpt.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <istream>
#include <map>
#include <numeric>
#include <ostream>
#include <set>

namespace
{
    constexpr std::size_t maxVertices = 500000, maxStamps = 100000;
    using Index = std::uint32_t;
    using Edge = std::pair<Index, Index>;
    Edge Key(Index a, Index b) { return std::minmax(a, b); }
    double Distance2(Vector3 p, Vector3 a, Vector3 b)
    {
        const double dx = b.x-a.x, dz = b.z-a.z, length = dx*dx+dz*dz;
        const double t = length > 0 ? std::clamp(((p.x-a.x)*dx+(p.z-a.z)*dz)/length, 0.0, 1.0) : 0;
        const double x = p.x-a.x-t*dx, z = p.z-a.z-t*dz;
        return x*x+z*z;
    }
    double BoxDistance2(const Bounds3& b, Vector3 p)
    {
        const double x = std::max({b.min.x-p.x, 0.0, p.x-b.max.x});
        const double z = std::max({b.min.z-p.z, 0.0, p.z-b.max.z});
        return x*x+z*z;
    }
    std::optional<double> HeightInTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        const double det = (b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x);
        if (std::abs(det) < 1e-16) return {};
        const double u = ((p.x-a.x)*(c.z-a.z)-(p.z-a.z)*(c.x-a.x))/det;
        const double v = ((b.x-a.x)*(p.z-a.z)-(b.z-a.z)*(p.x-a.x))/det;
        if (u < -1e-10 || v < -1e-10 || u+v > 1+1e-10) return {};
        return a.y+u*(b.y-a.y)+v*(c.y-a.y);
    }
    // XZ BVH used for boundary distances and surface sampling; no full triangle
    // scan for each point of the brush circle.
    struct Tree
    {
        struct Node { Bounds3 bounds; int left=-1, right=-1; std::size_t first=0, end=0; };
        std::vector<Bounds3> bounds;
        std::vector<std::size_t> order;
        std::vector<Node> nodes;
        void Build()
        {
            order.resize(bounds.size()); std::iota(order.begin(), order.end(), 0);
            nodes.clear();
            if (!order.empty()) BuildNode(0, order.size());
        }
        int BuildNode(std::size_t first, std::size_t end)
        {
            Node n; n.first=first; n.end=end;
            for (auto i=first;i<end;++i) { n.bounds.Include(bounds[order[i]].min); n.bounds.Include(bounds[order[i]].max); }
            const int index=static_cast<int>(nodes.size()); nodes.push_back(n);
            if (end-first > 8)
            {
                const bool x=n.bounds.max.x-n.bounds.min.x > n.bounds.max.z-n.bounds.min.z;
                const auto middle=first+(end-first)/2;
                std::nth_element(order.begin()+first, order.begin()+middle, order.begin()+end, [&](auto a, auto b) {
                    return x ? bounds[a].min.x+bounds[a].max.x < bounds[b].min.x+bounds[b].max.x :
                               bounds[a].min.z+bounds[a].max.z < bounds[b].min.z+bounds[b].max.z;
                });
                const int left=BuildNode(first,middle), right=BuildNode(middle,end);
                nodes[index].left=left; nodes[index].right=right;
            }
            return index;
        }
        template<class F> void Visit(int index, Vector3 p, double& distance2, F&& visit) const
        {
            if (index < 0 || nodes.empty()) return;
            const auto& n=nodes[index];
            if (BoxDistance2(n.bounds,p)>distance2) return;
            if(n.left<0) { for(auto i=n.first;i<n.end;++i) if(BoxDistance2(bounds[order[i]],p)<=distance2) visit(order[i],distance2); }
            else
            {
                int a=n.left,b=n.right;
                if(BoxDistance2(nodes[a].bounds,p)>BoxDistance2(nodes[b].bounds,p)) std::swap(a,b);
                Visit(a,p,distance2,visit); Visit(b,p,distance2,visit);
            }
        }
    };
    bool NearTriangle(const TerrainStamp& s, Vector3 a, Vector3 b, Vector3 c)
    {
        if (HeightInTriangle(s.center,a,b,c)) return true;
        return std::min({Distance2(s.center,a,b),Distance2(s.center,b,c),Distance2(s.center,c,a)}) < s.radius*s.radius;
    }
    MeshVertex Midpoint(const MeshVertex& a, const MeshVertex& b)
    {
        return {(a.position+b.position)*0.5, {}, {(a.uv.x+b.uv.x)*0.5f,(a.uv.y+b.uv.y)*0.5f},
                {(a.color.r+b.color.r)*0.5f,(a.color.g+b.color.g)*0.5f,(a.color.b+b.color.b)*0.5f,(a.color.a+b.color.a)*0.5f}};
    }
    // Split shared edges consistently on both triangles, including the neighbors
    // outside the brush. Midpoints on the original perimeter remain on that seam.
    bool Refine(Mesh& mesh, const TerrainStamp& stamp)
    {
        const double spacing=stamp.radius/3;
        for(int pass=0;pass<32;++pass)
        {
            std::map<Edge,Index> split;
            for(std::size_t i=0;i<mesh.indices.size();i+=3)
            {
                const std::array v{mesh.indices[i],mesh.indices[i+1],mesh.indices[i+2]};
                if(!NearTriangle(stamp,mesh.vertices[v[0]].position,mesh.vertices[v[1]].position,mesh.vertices[v[2]].position)) continue;
                for(int j=0;j<3;++j)
                {
                    const auto a=mesh.vertices[v[j]].position,b=mesh.vertices[v[(j+1)%3]].position;
                    if(std::hypot(b.x-a.x,b.z-a.z)>spacing) split.try_emplace(Key(v[j],v[(j+1)%3]),0);
                }
            }
            if(split.empty()) return true;
            if(mesh.vertices.size()+split.size()>maxVertices) return SDL_SetError("Terrain detail limit reached; use a larger brush or undo some strokes");
            for(auto& [edge,index]:split)
            {
                index=static_cast<Index>(mesh.vertices.size());
                const auto v=Midpoint(mesh.vertices[edge.first],mesh.vertices[edge.second]);
                mesh.vertices.push_back(v);
            }
            std::vector<Index> indices; indices.reserve(mesh.indices.size()+split.size()*6);
            const auto emit=[&](Index a,Index b,Index c){indices.insert(indices.end(),{a,b,c});};
            for(std::size_t i=0;i<mesh.indices.size();i+=3)
            {
                const std::array v{mesh.indices[i],mesh.indices[i+1],mesh.indices[i+2]};
                std::array<Index,3> m{}; std::array<bool,3> cut{}; int count=0;
                for(int j=0;j<3;++j) if(auto it=split.find(Key(v[j],v[(j+1)%3]));it!=split.end()) {m[j]=it->second;cut[j]=true;++count;}
                if(count==0) emit(v[0],v[1],v[2]);
                else if(count==3) {emit(v[0],m[0],m[2]);emit(m[0],v[1],m[1]);emit(m[2],m[1],v[2]);emit(m[0],m[1],m[2]);}
                else if(count==1)
                {
                    int j=0;while(!cut[j])++j;
                    emit(v[j],m[j],v[(j+2)%3]);emit(m[j],v[(j+1)%3],v[(j+2)%3]);
                }
                else
                {
                    int j=0;while(cut[j])++j; // Unsplit edge j.
                    const int k=(j+1)%3,l=(j+2)%3;
                    emit(v[l],m[l],m[k]);emit(v[j],v[k],m[k]);emit(v[j],m[k],m[l]);
                }
            }
            mesh.indices=std::move(indices);
        }
        return SDL_SetError("Terrain brush refinement did not converge");
    }
}

TerrainSculptLayer::~TerrainSculptLayer()
{
    // Long editing sessions must not recursively destroy thousands of stamps.
    auto tail = std::move(previous);
    while (tail && tail.use_count() == 1)
    {
        auto next = std::move(tail->previous);
        tail.reset();
        tail = std::move(next);
    }
}

bool TerrainStamp::Valid() const
{
    return tool>=TerrainTool::Raise && tool<=TerrainTool::Flatten &&
        std::isfinite(center.x)&&std::isfinite(center.y)&&std::isfinite(center.z)&&
        std::isfinite(radius)&&radius>=0.1&&radius<=1000&&std::isfinite(amount)&&amount>0&&amount<=100&&
        (tool!=TerrainTool::Flatten||amount<=1)&&std::isfinite(seamFalloff)&&seamFalloff>=0.1&&seamFalloff<=1000;
}
bool ReadTerrainSculpt(std::istream& in, TerrainSculptEdits& edits)
{
    std::size_t count;
    if(!(in>>count)||count>maxStamps) return SDL_SetError("Invalid terrain sculpt count");
    TerrainSculptEdits result;
    for(std::size_t i=0;i<count;++i)
    {
        TerrainStamp s; int tool;
        if(!(in>>tool>>s.center.x>>s.center.y>>s.center.z>>s.radius>>s.amount>>s.seamFalloff)||tool<0||tool>2)
            return SDL_SetError("Invalid terrain sculpt stamp");
        s.tool=static_cast<TerrainTool>(tool);
        if(!s.Valid()) return SDL_SetError("Invalid terrain sculpt settings");
        result=std::make_shared<TerrainSculptLayer>(TerrainSculptLayer{s,result,i+1});
    }
    edits=std::move(result);return true;
}
void WriteTerrainSculpt(std::ostream& out, TerrainSculptEdits edits)
{
    std::vector<const TerrainStamp*> stamps;
    for(auto p=edits;p;p=p->previous) stamps.push_back(&p->stamp);
    out<<' '<<stamps.size();
    for(auto i=stamps.rbegin();i!=stamps.rend();++i)
    {
        const auto& s=**i;
        out<<' '<<static_cast<int>(s.tool)<<' '<<s.center.x<<' '<<s.center.y<<' '<<s.center.z<<' '<<s.radius<<' '<<s.amount<<' '<<s.seamFalloff;
    }
}
struct TerrainSculptSurface::Impl
{
    std::shared_ptr<const Mesh> mesh;
    std::vector<std::array<Vector3,2>> seams;
    Tree seamTree;
    std::vector<double> distances;
    mutable Tree triangles;
    mutable bool trianglesDirty=true;
    explicit Impl(std::shared_ptr<const Mesh> source):mesh(std::move(source))
    {
        // Weld split normal/UV vertices when identifying the perimeter.
        std::map<std::array<double,3>,Index> positions;
        std::vector<Index> welded;
        for(Index i=0;i<mesh->vertices.size();++i)
        {
            const auto p=mesh->vertices[i].position;
            welded.push_back(positions.try_emplace({p.x,p.y,p.z},i).first->second);
        }
        std::map<Edge,unsigned> counts;
        for(std::size_t i=0;i<mesh->indices.size();i+=3)
            for(int j=0;j<3;++j) ++counts[Key(welded[mesh->indices[i+j]],welded[mesh->indices[i+(j+1)%3]])];
        for(const auto& [edge,count]:counts) if(count==1)
        {
            const auto a=mesh->vertices[edge.first].position,b=mesh->vertices[edge.second].position;
            seams.push_back({a,b}); Bounds3 bounds;bounds.Include(a);bounds.Include(b);seamTree.bounds.push_back(bounds);
        }
        seamTree.Build();
    }
    double SeamDistance(Vector3 p) const
    {
        double distance=Bounds3::inf;
        seamTree.Visit(0,p,distance,[&](std::size_t i,double& d){d=std::min(d,Distance2(p,seams[i][0],seams[i][1]));});
        return std::sqrt(distance);
    }
};
TerrainSculptSurface::TerrainSculptSurface(std::shared_ptr<const Mesh> source):impl(std::make_unique<Impl>(std::move(source))){}
TerrainSculptSurface::~TerrainSculptSurface()=default;
const std::shared_ptr<const Mesh>& TerrainSculptSurface::MeshData() const{return impl->mesh;}
bool TerrainSculptSurface::Apply(const TerrainStamp& stamp)
{
    if(!stamp.Valid()) return SDL_SetError("Invalid terrain brush settings");
    if(BoxDistance2(impl->mesh->bounds,stamp.center)>=stamp.radius*stamp.radius) return false;
    auto mesh=std::make_shared<Mesh>(*impl->mesh);
    if(!Refine(*mesh,stamp)) return false;
    auto distances=impl->distances;
    distances.reserve(mesh->vertices.size());
    for(auto i=distances.size();i<mesh->vertices.size();++i) distances.push_back(impl->SeamDistance(mesh->vertices[i].position));
    bool changed=false;
    for(std::size_t i=0;i<mesh->vertices.size();++i)
    {
        auto& p=mesh->vertices[i].position;
        const double d=std::hypot(p.x-stamp.center.x,p.z-stamp.center.z);
        // Explicitly pin original seams and new points interpolated along them.
        if(d>=stamp.radius||distances[i]<=1e-6) continue;
        const double weight=(1-d/stamp.radius)*std::min(1.0,distances[i]/stamp.seamFalloff);
        const double delta=stamp.tool==TerrainTool::Flatten ? (stamp.center.y-p.y)*stamp.amount*weight :
                           (stamp.tool==TerrainTool::Raise ? 1 : -1)*stamp.amount*weight;
        if(std::abs(delta)<1e-10) continue;
        p.y+=delta;changed=true;
    }
    if(!changed) return false;
    for(auto& v:mesh->vertices) v.normal={};
    for(std::size_t i=0;i<mesh->indices.size();i+=3)
    {
        auto& a=mesh->vertices[mesh->indices[i]];auto& b=mesh->vertices[mesh->indices[i+1]];auto& c=mesh->vertices[mesh->indices[i+2]];
        const auto n=Math::Cross(b.position-a.position,c.position-a.position);
        a.normal=a.normal+n;b.normal=b.normal+n;c.normal=c.normal+n;
    }
    for(auto& v:mesh->vertices) v.normal=Math::Normalize(v.normal);
    mesh->UpdateBounds();impl->mesh=std::move(mesh);impl->distances=std::move(distances);impl->trianglesDirty=true;
    return true;
}
std::optional<double> TerrainSculptSurface::Height(double x,double z) const
{
    if(impl->trianglesDirty)
    {
        auto& tree=impl->triangles;tree.bounds.clear();
        const auto& m=*impl->mesh;
        for(std::size_t i=0;i<m.indices.size();i+=3)
        {
            Bounds3 b;for(int j=0;j<3;++j)b.Include(m.vertices[m.indices[i+j]].position);
            tree.bounds.push_back(b);
        }
        tree.Build();impl->trianglesDirty=false;
    }
    std::optional<double> result;double distance=0;
    const auto& m=*impl->mesh;
    impl->triangles.Visit(0,{x,0,z},distance,[&](std::size_t i,double&){
        if(auto h=HeightInTriangle({x,0,z},m.vertices[m.indices[3*i]].position,m.vertices[m.indices[3*i+1]].position,m.vertices[m.indices[3*i+2]].position))
            if(!result||*h>*result)result=h;
    });
    return result;
}
