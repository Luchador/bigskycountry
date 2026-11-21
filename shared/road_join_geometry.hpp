#pragma once

#include "road.hpp"
#include <algorithm>
#include <cmath>

// Shared clipping for joined road and railroad surfaces.
namespace RoadJoinGeometry
{
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
    inline std::vector<Cutter> BedCutters(const Road& source, const Box& region, bool equal)
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
    inline void TrimBed(Road& road, const std::vector<Cutter>& cutters)
    {
        Cut(road.pavementTriangles,cutters);
        Cut(road.edgeTriangles,cutters);
        for (auto& [material,triangles] : road.transitionStrips) Cut(triangles,cutters);
    }

}
