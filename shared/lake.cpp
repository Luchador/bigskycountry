#include "lake.hpp"
#include <CDT.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <istream>
#include <numbers>
#include <ostream>

namespace
{
    constexpr double handleRatio = 4.0 * (std::numbers::sqrt2 - 1.0) / 3.0;
    Vector3 Mix(Vector3 a, Vector3 b, double t) { return a + (b - a) * t; }
    double Side(Vector3 a, Vector3 b, Vector3 p) { return (b.x - a.x) * (p.z - a.z) - (b.z - a.z) * (p.x - a.x); }
    bool FlatFinite(Vector3 p)
    {
        return std::isfinite(p.x) && std::isfinite(p.z) && p.y == 0 && std::abs(p.x) <= 200000 && std::abs(p.z) <= 200000;
    }
    double Distance(Vector3 p, Vector3 a, Vector3 b)
    {
        const auto d = b - a;
        const double length2 = Math::Dot(d, d);
        return Math::Length(p - (a + d * (length2 > 0 ? std::clamp(Math::Dot(p - a, d) / length2, 0.0, 1.0) : 0)));
    }
    bool Crosses(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        if (std::max(a.x, b.x) < std::min(c.x, d.x) || std::max(c.x, d.x) < std::min(a.x, b.x) ||
            std::max(a.z, b.z) < std::min(c.z, d.z) || std::max(c.z, d.z) < std::min(a.z, b.z)) return false;
        return Side(a, b, c) * Side(a, b, d) <= 0 && Side(c, d, a) * Side(c, d, b) <= 0;
    }
    bool Inside(Vector3 p, const std::vector<LakeSample>& rim)
    {
        bool inside = false;
        for (std::size_t i = 1; i < rim.size(); ++i)
        {
            const auto a = rim[i - 1].position, b = rim[i].position;
            if ((a.z > p.z) != (b.z > p.z) && p.x < (b.x - a.x) * (p.z - a.z) / (b.z - a.z) + a.x) inside = !inside;
        }
        return inside;
    }
    double ShoreDistance(Vector3 p, const std::vector<LakeSample>& rim)
    {
        double distance = std::numeric_limits<double>::max();
        for (std::size_t i = 1; i < rim.size(); ++i)
            distance = std::min(distance, Distance(p, rim[i - 1].position, rim[i].position));
        return distance;
    }
    std::shared_ptr<Mesh> MakeMesh(const CDT::Triangulation<double>& triangulation, bool bed,
                                   double depth, const std::vector<LakeSample>& rim)
    {
        auto mesh = std::make_shared<Mesh>();
        for (const auto& p : triangulation.vertices)
        {
            Vector3 position{p.x, 0, p.y};
            if (bed) position.y = -std::min(depth, ShoreDistance(position, rim) * 0.25);
            mesh->vertices.push_back({position, {}, {float(p.x / 4), float(p.y / 4)},
                                      bed ? SDL_FColor{0.24f, 0.20f, 0.12f, 1} : SDL_FColor{1, 1, 1, 1}});
        }
        for (const auto& triangle : triangulation.triangles)
        {
            auto a = triangle.vertices[0], b = triangle.vertices[1], c = triangle.vertices[2];
            auto normal = Math::Cross(mesh->vertices[b].position - mesh->vertices[a].position,
                                      mesh->vertices[c].position - mesh->vertices[a].position);
            if (normal.y < 0) { std::swap(b, c); normal = normal * -1; }
            if (normal.y < 1e-10) continue;
            mesh->indices.insert(mesh->indices.end(), {a, b, c});
            for (auto i : {a, b, c}) mesh->vertices[i].normal = mesh->vertices[i].normal + normal;
        }
        for (auto& vertex : mesh->vertices) vertex.normal = Math::Normalize(vertex.normal);
        mesh->UpdateBounds();
        return mesh;
    }
}

LakeDefinition LakeDefinition::Circle(double diameter)
{
    LakeDefinition lake;
    const double radius = diameter * 0.5;
    for (Vector3 p : std::array<Vector3, 4>{{{radius, 0, 0}, {0, 0, radius}, {-radius, 0, 0}, {0, 0, -radius}}})
    {
        const Vector3 handle{-p.z * handleRatio, 0, p.x * handleRatio};
        lake.points.push_back({p, handle * -1, handle});
    }
    return lake;
}

Vector3 LakeDefinition::Evaluate(int segment, double t) const
{
    const auto& a = points.at(segment);
    const auto& b = points.at((segment + 1) % points.size());
    const double s = 1 - t;
    return a.position * (s*s*s) + (a.position + a.outgoing) * (3*s*s*t) +
           (b.position + b.incoming) * (3*s*t*t) + b.position * (t*t*t);
}

void LakeDefinition::SmoothHandles()
{
    // Keep tangents continuous, with local chord lengths limiting overshoot.
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        auto& p = points[i];
        const auto previous = points[(i + points.size() - 1) % points.size()].position;
        const auto next = points[(i + 1) % points.size()].position;
        const auto tangent = Math::Normalize(next - previous);
        const double length = std::min(Math::Length(p.position - previous), Math::Length(next - p.position)) * handleRatio / std::numbers::sqrt2;
        p.outgoing = tangent * length;
        p.incoming = p.outgoing * -1;
    }
}

bool LakeDefinition::MovePoint(int index, Vector3 position)
{
    if (index < 0 || static_cast<std::size_t>(index) >= points.size() || !FlatFinite(position)) return false;
    if (points[index].position == position) return true;
    points[index].position = position;
    SmoothHandles();
    return true;
}

bool LakeDefinition::InsertPoint(int segment, double t)
{
    if (points.size() >= maxPoints || segment < 0 || static_cast<std::size_t>(segment) >= points.size() ||
        !std::isfinite(t) || t <= 0.001 || t >= 0.999) return SDL_SetError("Choose a position between lake control points");
    auto& a = points[segment];
    auto& b = points[(segment + 1) % points.size()];
    const auto p = Mix(a.position, a.position + a.outgoing, t);
    const auto q = Mix(a.position + a.outgoing, b.position + b.incoming, t);
    const auto r = Mix(b.position + b.incoming, b.position, t);
    const auto s = Mix(p, q, t), u = Mix(q, r, t), center = Mix(s, u, t);
    a.outgoing = p - a.position;
    b.incoming = r - b.position;
    points.insert(points.begin() + segment + 1, LakePoint{center, s - center, u - center});
    return true;
}

bool LakeDefinition::RemovePoint(int index)
{
    if (points.size() <= 3) return SDL_SetError("A lake needs at least three control points");
    if (index < 0 || static_cast<std::size_t>(index) >= points.size()) return false;
    points.erase(points.begin() + index);
    SmoothHandles();
    return true;
}

bool LakeDefinition::Read(std::istream& in)
{
    std::size_t count;
    if (!(in >> depth >> count) || count < 3 || count > maxPoints) return SDL_SetError("Invalid lake definition");
    points.resize(count);
    for (auto& p : points)
        if (!(in >> p.position.x >> p.position.z >> p.incoming.x >> p.incoming.z >> p.outgoing.x >> p.outgoing.z))
            return SDL_SetError("Invalid lake control point");
    return true;
}

bool LakeDefinition::Write(std::ostream& out) const
{
    out << depth << ' ' << points.size();
    for (const auto& p : points)
        out << ' ' << p.position.x << ' ' << p.position.z << ' ' << p.incoming.x << ' ' << p.incoming.z << ' ' << p.outgoing.x << ' ' << p.outgoing.z;
    return bool(out);
}

std::optional<std::vector<LakeSample>> LakeGeometry::Sample(const LakeDefinition& lake)
{
    const auto fail = [](const char* error) -> std::optional<std::vector<LakeSample>> { SDL_SetError("%s", error); return {}; };
    const auto count = lake.points.size();
    if (count < 3 || count > LakeDefinition::maxPoints || !std::isfinite(lake.depth) || lake.depth <= 0 || lake.depth > 1000)
        return fail("Invalid lake size or depth");
    double perimeter = 0;
    for (std::size_t i = 0; i < count; ++i)
    {
        const auto& a = lake.points[i];
        const auto& b = lake.points[(i + 1) % count];
        if (!FlatFinite(a.position) || !FlatFinite(a.incoming) || !FlatFinite(a.outgoing) || Math::Length(a.position - b.position) < 0.5)
            return fail("Lake points must be finite and at least 0.5 m apart");
        perimeter += Math::Length(a.outgoing) + Math::Length(b.incoming) + Math::Length(b.position + b.incoming - a.position - a.outgoing);
    }
    std::vector<LakeSample> rim;
    const double step = std::max(2.0, perimeter / 2048);
    for (int i = 0; i < static_cast<int>(count); ++i)
    {
        const auto& a = lake.points[i];
        const auto& b = lake.points[(i + 1) % count];
        const double length = Math::Length(a.outgoing) + Math::Length(b.incoming) + Math::Length(b.position + b.incoming - a.position - a.outgoing);
        const int steps = std::max(4, int(std::ceil(length / step)));
        for (int j = 0; j < steps; ++j)
        {
            const double t = double(j) / steps;
            const auto p = lake.Evaluate(i, t);
            rim.push_back({p, i, t});
        }
    }
    rim.push_back({rim.front().position, static_cast<int>(count) - 1, 1});
    return rim;
}

std::optional<LakeGeometry> LakeGeometry::Build(const LakeDefinition& lake)
{
    auto samples = Sample(lake);
    if (!samples) return {};
    return BuildShoreline(std::move(*samples), lake.depth);
}

std::optional<LakeGeometry> LakeGeometry::BuildShoreline(std::vector<LakeSample> samples, double depth)
{
    const auto fail = [](const char* error) -> std::optional<LakeGeometry> { SDL_SetError("%s", error); return {}; };
    if (samples.size() < 4 || !std::isfinite(depth) || depth <= 0 || depth > 1000) return fail("Invalid water boundary or depth");
    LakeGeometry result;
    result.shoreline = std::move(samples);
    const auto& rim = result.shoreline;
    Bounds3 bounds;
    for (const auto& sample : rim) bounds.Include(sample.position);
    double area = 0;
    for (std::size_t i = 1; i < rim.size(); ++i)
    {
        const auto a = rim[i - 1].position, b = rim[i].position;
        if (Math::Length(a - b) < 1e-6) return fail("Water boundary folds back on itself");
        area += a.x * b.z - b.x * a.z;
        for (std::size_t j = i + 2; j < rim.size(); ++j)
        {
            if (i == 1 && j + 1 == rim.size()) continue;
            if (Crosses(a, b, rim[j - 1].position, rim[j].position)) return fail("Water boundary cannot cross itself");
        }
    }
    if (std::abs(area) < 1) return fail("Water boundary encloses too little area");
    std::vector<CDT::V2d<double>> vertices;
    std::vector<CDT::Edge> edges;
    const auto rimCount = static_cast<CDT::VertInd>(rim.size() - 1);
    for (CDT::VertInd i = 0; i < rimCount; ++i)
    {
        vertices.push_back({rim[i].position.x, rim[i].position.z});
        edges.emplace_back(i, (i + 1) % rimCount);
    }
    try
    {
        CDT::Triangulation<double> water;
        water.insertVertices(vertices);
        water.insertEdges(edges);
        water.eraseOuterTrianglesAndHoles();
        result.water = MakeMesh(water, false, depth, rim);
        // Interior samples make a subdivided basin, including narrow areas where
        // a regular grid would miss the polygon altogether.
        const double spacing = std::max(5.0, std::max(bounds.max.x - bounds.min.x, bounds.max.z - bounds.min.z) / 96);
        for (double z = bounds.min.z + spacing * 0.5; z < bounds.max.z; z += spacing)
            for (double x = bounds.min.x + spacing * 0.5; x < bounds.max.x; x += spacing)
                if (Inside({x, 0, z}, rim) && ShoreDistance({x, 0, z}, rim) > 0.1) vertices.push_back({x, z});
        if (vertices.size() == rimCount)
            for (const auto& triangle : water.triangles)
            {
                Vector3 center{};
                for (auto i : triangle.vertices) center = center + Vector3{water.vertices[i].x, 0, water.vertices[i].y} * (1.0 / 3);
                vertices.push_back({center.x, center.z});
            }
        CDT::Triangulation<double> bed;
        bed.insertVertices(vertices);
        bed.insertEdges(edges);
        bed.eraseOuterTrianglesAndHoles();
        result.bed = MakeMesh(bed, true, depth, rim);
    }
    catch (const std::exception&)
    {
        return fail("Water boundary cannot be triangulated");
    }
    if (result.bed->indices.empty() || result.water->indices.empty()) return fail("Water boundary encloses no surface");
    return result;
}
