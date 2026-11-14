#include "terrain_fill.hpp"

#include <CDT.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <map>

namespace
{
    using Point = CDT::V2d<double>;
    using Index = CDT::VertInd;
    using Edge = CDT::Edge;
    using Triangulation = CDT::Triangulation<double>;
    using Segment = std::array<Vector3, 2>;
    constexpr double topologyResolution = 1e-7; // Meters in the local XZ projection.

    double Turn(Point a, Point b, Point c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    // Only topology uses quantized local coordinates. The output keeps original
    // world-space rim vertices, including the elevation of a banked road or curb.
    struct Outline
    {
        Vector3 origin;
        std::vector<Point> points;
        std::vector<Vector3> positions;
        std::map<std::pair<double, double>, Index> lookup;
        std::map<Edge, Segment> segments;
        std::vector<std::vector<Index>> polygons;
        std::set<Index> verticalSteps;

        Index AddPoint(Vector3 p)
        {
            constexpr double unitsPerMeter = 1.0 / topologyResolution;
            const auto key = std::pair{std::round((p.x - origin.x) * unitsPerMeter),
                                       std::round((p.z - origin.z) * unitsPerMeter)};
            const auto [it, inserted] = lookup.try_emplace(key, static_cast<Index>(points.size()));
            if (inserted)
            {
                points.emplace_back(key.first / unitsPerMeter, key.second / unitsPerMeter);
                positions.push_back(p);
            }
            return it->second;
        }

        void AddEdge(Vector3 a, Vector3 b)
        {
            const Index i = AddPoint(a), j = AddPoint(b);
            if (i != j) segments.try_emplace(Edge(i, j), i < j ? Segment{a, b} : Segment{b, a});
            else if (std::abs(a.y - b.y) > 1e-7) verticalSteps.insert(i);
        }

        bool OnRoad(Point p) const
        {
            for (const auto& polygon : polygons)
            {
                bool inside = false;
                Index last = polygon.back();
                for (Index index : polygon)
                {
                    const Point a = points[last], b = points[index];
                    if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)
                        inside = !inside;
                    last = index;
                }
                if (inside) return true;
            }
            return false;
        }

        Vector3 PositionOn(Edge source, Point p) const
        {
            const Point a = points[source.v1()], b = points[source.v2()];
            const auto& ends = segments.at(source);
            if (p.x == a.x && p.y == a.y) return ends[0];
            if (p.x == b.x && p.y == b.y) return ends[1];
            const double dx = b.x - a.x, dz = b.y - a.y;
            const double t = ((p.x - a.x) * dx + (p.y - a.y) * dz) / (dx * dx + dz * dz);
            return ends[0] + (ends[1] - ends[0]) * std::clamp(t, 0.0, 1.0);
        }
    };

    Point Center(const Triangulation& cdt, const CDT::Triangle& tri)
    {
        const auto a = cdt.vertices[tri.vertices[0]], b = cdt.vertices[tri.vertices[1]], c = cdt.vertices[tri.vertices[2]];
        return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
    }

    // Flood up to the actual footprint edges. A concave pocket may touch the
    // convex hull, but a path across an unconstrained hull edge leads outdoors.
    std::vector<std::vector<CDT::TriInd>> LandPockets(const Triangulation& cdt, const Outline& outline)
    {
        std::vector<bool> visited(cdt.triangles.size());
        std::vector<std::vector<CDT::TriInd>> pockets;
        for (CDT::TriInd start = 0; start < visited.size(); ++start)
        {
            if (visited[start]) continue;
            std::vector<CDT::TriInd> region{start};
            visited[start] = true;
            bool outside = false;
            for (std::size_t i = 0; i < region.size(); ++i)
            {
                const auto& t = cdt.triangles[region[i]];
                for (int side = 0; side < 3; ++side)
                {
                    if (cdt.fixedEdges.count(Edge(t.vertices[side], t.vertices[(side + 1) % 3]))) continue;
                    const auto n = t.neighbors[side];
                    if (n == CDT::noNeighbor) outside = true;
                    else if (!visited[n])
                    {
                        visited[n] = true;
                        region.push_back(n);
                    }
                }
            }
            if (!outside && !outline.OnRoad(Center(cdt, cdt.triangles[start])))
                pockets.push_back(std::move(region));
        }
        return pockets;
    }

    struct Seam
    {
        Index i, j;
        Segment rim;
    };

    // Refinement uses the same boundary constraints as the coarse pocket, including holes.
    struct Pocket
    {
        std::map<Index, Index> local;
        std::vector<Point> points;
        std::vector<Vector3> positions;
        std::vector<Edge> boundary;
        std::vector<Seam> seams;

        Index AddRimVertex(Index coarseIndex, Point point, Vector3 position)
        {
            const auto [it, added] = local.try_emplace(coarseIndex, static_cast<Index>(points.size()));
            if (added)
            {
                points.push_back(point);
                positions.push_back(position);
            }
            else positions[it->second].y = std::max(positions[it->second].y, position.y);
            return it->second;
        }
    };

    bool CollectRim(Pocket& out, const Triangulation& coarse, const Outline& outline,
                    const std::vector<CDT::TriInd>& region)
    {
        std::set<Index> steps;
        double twiceArea = 0, perimeter = 0;
        for (auto index : region)
        {
            const auto& t = coarse.triangles[index];
            twiceArea += Turn(coarse.vertices[t.vertices[0]], coarse.vertices[t.vertices[1]], coarse.vertices[t.vertices[2]]);
            for (int side = 0; side < 3; ++side)
            {
                const Index a = t.vertices[side], b = t.vertices[(side + 1) % 3];
                Edge source(a, b);
                if (!coarse.fixedEdges.count(source)) continue;
                if (!outline.segments.contains(source))
                {
                    // A crossing or a collinear mouth vertex split the original constraint.
                    const auto split = coarse.pieceToOriginals.find(source);
                    if (split == coarse.pieceToOriginals.end() || split->second.empty()) return false;
                    source = split->second.front();
                }
                Segment rim{outline.PositionOn(source, coarse.vertices[a]), outline.PositionOn(source, coarse.vertices[b])};
                const Index i = out.AddRimVertex(a, coarse.vertices[a], rim[0]);
                const Index j = out.AddRimVertex(b, coarse.vertices[b], rim[1]);
                if (outline.verticalSteps.contains(a)) steps.insert(i);
                if (outline.verticalSteps.contains(b)) steps.insert(j);
                perimeter += std::hypot(rim[1].x - rim[0].x, rim[1].z - rim[0].z);
                out.boundary.emplace_back(i, j);
                out.seams.push_back({i, j, rim});
            }
        }
        for (const auto& seam : out.seams)
        {
            // Only an actual vertical step in the source mesh can contribute two
            // elevations at one XZ. An overpass cannot close a terrain boundary.
            const auto fits = [&](Index i, double y)
            {
                const double height = steps.contains(i) ? RoadStyle::curbHeight : 0.0;
                return out.positions[i].y - y <= height + 1e-6;
            };
            if (!fits(seam.i, seam.rim[0].y) || !fits(seam.j, seam.rim[1].y)) return false;
        }
        // Projecting a rotated road mouth can leave a numerical sliver between
        // two collinear segments. It has no interior wider than the weld precision.
        return out.points.size() >= 3 && twiceArea > perimeter * topologyResolution;
    }

    void SampleInterior(Pocket& out, const Triangulation& coarse, const Outline& outline,
                        const std::vector<CDT::TriInd>& region)
    {
        const auto& points = out.points;
        const auto [xmin, xmax] = std::minmax_element(points.begin(), points.end(), [](Point a, Point b){ return a.x < b.x; });
        const auto [zmin, zmax] = std::minmax_element(points.begin(), points.end(), [](Point a, Point b){ return a.y < b.y; });
        const double width = xmax->x - xmin->x, depth = zmax->y - zmin->y;
        const double spacing = std::max(8.0, std::max(width, depth) / 128);
        // Cell centers give even very narrow blocks an interior row.
        const double dx = width / std::max(1.0, std::ceil(width / spacing));
        const double dz = depth / std::max(1.0, std::ceil(depth / spacing));
        if (dx <= 0 || dz <= 0) return;
        const Point origin{xmin->x + dx * 0.5, zmin->y + dz * 0.5};
        std::set<std::pair<int, int>> sampled;
        double largestArea = 0;
        Vector3 fallback{};
        Point fallbackXZ;
        const auto position = [&](Index i)
        {
            const auto it = out.local.find(i);
            return it == out.local.end() ? outline.positions.at(i) : out.positions[it->second];
        };
        for (auto index : region)
        {
            const auto& v = coarse.triangles[index].vertices;
            const Vector3 pa = position(v[0]), pb = position(v[1]), pc = position(v[2]);
            const Point a = coarse.vertices[v[0]], b = coarse.vertices[v[1]], c = coarse.vertices[v[2]];
            const double area = Turn(a, b, c);
            if (area <= 0) continue;
            if (area > largestArea)
            {
                largestArea = area;
                fallback = (pa + pb + pc) * (1.0 / 3);
                fallbackXZ = Center(coarse, coarse.triangles[index]);
            }
            const int x0 = static_cast<int>(std::ceil((std::min({a.x, b.x, c.x}) - origin.x) / dx));
            const int x1 = static_cast<int>(std::floor((std::max({a.x, b.x, c.x}) - origin.x) / dx));
            const int z0 = static_cast<int>(std::ceil((std::min({a.y, b.y, c.y}) - origin.y) / dz));
            const int z1 = static_cast<int>(std::floor((std::max({a.y, b.y, c.y}) - origin.y) / dz));
            for (int z = z0; z <= z1; ++z)
            {
                for (int x = x0; x <= x1; ++x)
                {
                    const Point p{origin.x + x * dx, origin.y + z * dz};
                    const std::array weights{Turn(b, c, p) / area, Turn(c, a, p) / area, Turn(a, b, p) / area};
                    if (*std::min_element(weights.begin(), weights.end()) < -1e-12) continue;
                    bool onRim = false;
                    for (int k = 0; k < 3; ++k)
                    {
                        if (weights[k] <= 1e-10 && coarse.fixedEdges.count(Edge(v[(k + 1) % 3], v[(k + 2) % 3])))
                            onRim = true;
                        if (weights[k] >= 1 - 1e-10 && out.local.contains(v[k])) onRim = true;
                    }
                    if (onRim || !sampled.emplace(x, z).second) continue;
                    out.points.push_back(p);
                    out.positions.push_back(pa + (pb - pa) * weights[1] + (pc - pa) * weights[2]);
                }
            }
        }
        if (sampled.empty() && largestArea > 0)
        {
            out.points.push_back(fallbackXZ);
            out.positions.push_back(fallback);
        }
    }

    void AppendPocket(Mesh& mesh, const Pocket& pocket)
    {
        Triangulation fine;
        fine.insertVertices(pocket.points);
        fine.insertEdges(pocket.boundary);
        fine.eraseOuterTrianglesAndHoles();
        if (fine.triangles.empty()) return;
        const auto base = static_cast<Index>(mesh.vertices.size());
        for (Vector3 p : pocket.positions) mesh.vertices.push_back({p, {}});
        const auto emit = [&](Index a, Index b, Index c)
        {
            mesh.indices.insert(mesh.indices.end(), {a, b, c});
            auto& va = mesh.vertices[a];
            auto& vb = mesh.vertices[b];
            auto& vc = mesh.vertices[c];
            const auto n = Math::Cross(vb.position - va.position, vc.position - va.position);
            va.normal = va.normal + n;
            vb.normal = vb.normal + n;
            vc.normal = vc.normal + n;
        };
        for (const auto& t : fine.triangles)
            emit(base + t.vertices[0], base + t.vertices[2], base + t.vertices[1]);
        std::map<std::pair<Index, double>, Index> lowerVertices;
        const auto atRoadHeight = [&](Index i, Vector3 rim)
        {
            if (pocket.positions[i].y - rim.y <= 1e-7) return base + i;
            const auto [it, added] = lowerVertices.try_emplace(std::pair{i, rim.y}, static_cast<Index>(mesh.vertices.size()));
            if (added) mesh.vertices.push_back({rim, {}});
            return it->second;
        };
        for (const auto& seam : pocket.seams)
        {
            // Close the vertical step at an exposed curb end against the authored road cap.
            const Index a = base + seam.i, b = base + seam.j;
            const Index lowerA = atRoadHeight(seam.i, seam.rim[0]), lowerB = atRoadHeight(seam.j, seam.rim[1]);
            if (lowerA != a) emit(a, b, lowerA);
            if (lowerB != b) emit(lowerA, b, lowerB);
        }
    }
}

std::shared_ptr<const Mesh> BuildTerrainFill(std::span<const RoadGeometry* const> footprints, std::span<const Segment> links)
{
    const auto first = std::find_if(footprints.begin(), footprints.end(), [](const auto* g){ return g->outline.size() >= 3; });
    if (first == footprints.end()) return {};
    Outline outline{(*first)->outline.front()};
    for (const RoadGeometry* footprint : footprints)
    {
        const auto& rim = footprint->outline;
        if (rim.size() < 3) continue;
        auto& polygon = outline.polygons.emplace_back();
        Vector3 previous = rim.back();
        for (Vector3 p : rim)
        {
            polygon.push_back(outline.AddPoint(p));
            outline.AddEdge(previous, p);
            previous = p;
        }
    }
    for (const auto& link : links) outline.AddEdge(link[0], link[1]);
    if (outline.points.size() < 3) return {};
    try
    {
        Triangulation coarse(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::TryResolve, 1e-8);
        coarse.insertVertices(outline.points);
        std::vector<Edge> edges;
        for (const auto& [edge, segment] : outline.segments) edges.push_back(edge);
        coarse.insertEdges(edges);
        coarse.eraseSuperTriangle();
        auto mesh = std::make_shared<Mesh>();
        for (const auto& region : LandPockets(coarse, outline))
        {
            Pocket pocket;
            if (!CollectRim(pocket, coarse, outline, region)) continue;
            SampleInterior(pocket, coarse, outline, region);
            AppendPocket(*mesh, pocket);
        }
        for (auto& vertex : mesh->vertices) vertex.normal = Math::Normalize(vertex.normal);
        if (!mesh->indices.empty())
        {
            mesh->UpdateBounds();
            return mesh;
        }
    }
    catch (const CDT::Error& e)
    {
        SDL_LogDebug(SDL_LOG_CATEGORY_APPLICATION, "Terrain boundary: %s", e.what());
    }
    return {};
}
