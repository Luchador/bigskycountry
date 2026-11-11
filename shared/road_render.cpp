#include "road_render.hpp"
#include "editor_colors.hpp"
#include "world_renderer.hpp"
#include "free_camera.hpp"
#include "sun.hpp"
#include "world_state.hpp"
#include "view_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr SDL_FColor roadColor{0.43f, 0.43f, 0.43f, 1.0f};
    constexpr SDL_FColor yellowStripeColor{0.96f, 0.85f, 0.12f, 1.0f};
    constexpr SDL_FColor whiteStripeColor{0.9f, 0.9f, 0.9f, 1.0f};

    using ViewGeometry::Clip;
    using ViewGeometry::ScreenPoint;
    using ViewGeometry::SidePlanes;

    template<typename ViewTransform, typename Emit>
    void AppendOutline(const Road& road, ViewTransform transform, double focal, SDL_Rect viewport, Emit emit)
    {
        if (road.outline.empty())
        {
            return;
        }
        const auto sides = SidePlanes(focal, viewport);
        Vector3 previous = transform(road.outline.back());
        for (Vector3 point : road.outline)
        {
            Vector3 a = previous;
            previous = transform(point);
            Vector3 b = previous;
            const auto stroke = ViewGeometry::ProjectStroke(a, b, sides, focal, viewport, worldNearDepth, 1.5f);
            if (!stroke)
            {
                continue;
            }
            for (int i : {0, 1, 2, 0, 2, 3})
            {
                emit(stroke->corners[i], i < 2 ? stroke->startDepth : stroke->endDepth);
            }
        }
    }

    template<typename Vertex, typename Transform, typename ClipPlane, typename Emit>
    void AppendMesh(const std::vector<std::array<Vertex, 3>>& triangles, Transform transform,
                    ClipPlane clip, Emit emit, double focal, SDL_Rect viewport)
    {
        if (triangles.empty())
        {
            return;
        }
        const auto sides = SidePlanes(focal, viewport);

        for (const auto& triangle : triangles)
        {
            ViewGeometry::VertexPolygon<Vertex> p{};

            for (const Vertex& point : triangle)
            {
                p.points[p.count++] = transform(point);
            }

            p = clip(p, {0, 0, 1}, worldNearDepth);

            for (Vector3 side : sides)
            {
                if (p.count < 3)
                {
                    break;
                }
                p = clip(p, side, 0.0);
            }

            for (int i = 2; i < p.count; ++i)
            {
                for (int index : {0, i - 1, i})
                {
                    emit(p.points[index]);
                }
            }
        }
    }

    template<typename ViewTransform>

    void AppendTriangles(const std::vector<Road::Triangle>& triangles, SDL_FColor color, ViewTransform transform, double focal, SDL_Rect viewport, std::vector<SDL_Vertex>& vertices)
    {
        AppendMesh(triangles, transform, Clip, [&](Vector3 v)
        {
            vertices.push_back({ScreenPoint(v, focal, viewport), color, {}});
        }, focal, viewport);
    }

    using EdgePolygon = ViewGeometry::VertexPolygon<Road::EdgeVertex>;

    EdgePolygon ClipEdge(const EdgePolygon& input, Vector3 normal, double offset)
    {
        return ViewGeometry::ClipPolygon(input, [&](const Road::EdgeVertex& v)
        {
            return Math::Dot(v.position, normal) - offset;
        }, [](const Road::EdgeVertex& a, const Road::EdgeVertex& b, double t)
        {
            const float f = static_cast<float>(t);
            return Road::EdgeVertex{a.position + (b.position - a.position) * t,
                                    {std::lerp(a.color.r, b.color.r, f), std::lerp(a.color.g, b.color.g, f),
                                     std::lerp(a.color.b, b.color.b, f), std::lerp(a.color.a, b.color.a, f)}};
        });
    }

    template <typename ViewTransform>

    void AppendRoad(const Road& road, RoadLayer layer, ViewTransform transform, double focal, SDL_Rect viewport, std::vector<SDL_Vertex>& vertices)
    {
        if (layer == RoadLayer::Pavement)
        {
            AppendTriangles(road.triangles, roadColor, transform, focal, viewport, vertices);
            AppendMesh(road.edgeTriangles, [&](const Road::EdgeVertex& v)
            {
                return Road::EdgeVertex{transform(v.position), v.color};
            }, ClipEdge, [&](const Road::EdgeVertex& v)
            {
                vertices.push_back({ScreenPoint(v.position, focal, viewport), v.color, {}});
            }, focal, viewport);
            return;
        }
        if (layer == RoadLayer::Outline)
        {
            AppendOutline(road, transform, focal, viewport, [&](SDL_FPoint p, double)
            {
                vertices.push_back({p, EditorColors::orange, {}});
            });
            return;
        }
        AppendTriangles(road.yellowStripes, yellowStripeColor, transform, focal, viewport, vertices);
        AppendTriangles(road.whiteStripes, whiteStripeColor, transform, focal, viewport, vertices);
    }

    // Draw a batch of road triangles and return whether the operation succeeded.
    bool Submit(SDL_Renderer* renderer, const std::vector<SDL_Vertex>& vertices)
    {
        return vertices.empty() || SDL_RenderGeometry(renderer, nullptr, vertices.data(), static_cast<int>(vertices.size()), nullptr, 0);
    }
}

void WorldRenderer::SubmitRoad(const Road& road, bool selected)
{
    const auto solid = [&](const std::vector<Road::Triangle>& triangles, SDL_FColor color)
    {
        for (const auto& t : triangles)
        {
            SubmitTriangle(t[0], t[1], t[2], color);
        }
    };
    const std::size_t start = vertices.size();
    if (!road.triangles.empty())
    {
        // Project the aggregate in road-local meters. Adjacent coplanar triangles
        // share UVs, and moving or rotating the road carries the material with it.
        const Vector3 origin = road.points.empty() ? road.triangles.front()[0] : road.points.front().position;
        const auto frame = ObjectTransform{origin, road.rotation}.ToMeshTransform();
        if (pavement.empty())
        {
            pavementSun = Sun::Direction(g_WorldState.GetTimeOfDayMinutes());
        }
        for (const auto& t : road.triangles)
        {
            const Vector3 normal = Math::Normalize(Math::Cross(t[1] - t[0], t[2] - t[0]));
            const Vector3 local = frame.Unrotate(normal);
            Vector3 u = frame.x, v = frame.z;
            if (std::abs(local.x) > std::abs(local.y) && std::abs(local.x) >= std::abs(local.z)) u = frame.y;
            else if (std::abs(local.z) > std::abs(local.y)) v = frame.y;
            Vector3 tangent = Math::Normalize(Math::Cross(normal, v));
            if (Math::Dot(tangent, u) < 0) tangent = tangent * -1;
            Vector3 bitangent = Math::Cross(normal, tangent);
            if (Math::Dot(bitangent, v) < 0) bitangent = bitangent * -1;
            const auto toTangent = [&](Vector3 d) -> std::array<float, 3>
            {
                return {float(Math::Dot(d, tangent)), float(Math::Dot(d, bitangent)), float(Math::Dot(d, normal))};
            };
            const auto light = toTangent(pavementSun);
            for (Vector3 p : t)
            {
                auto vertex = WorldToClip(p, roadColor);
                const Vector3 offset = p - frame.origin;
                vertex.uv = {float(Math::Dot(offset, u) / RoadMaterial::tileMeters),
                             float(Math::Dot(offset, v) / RoadMaterial::tileMeters)};
                vertex.lighting = {light, toTangent(camera->GetPosition() - p)};
                vertices.push_back(vertex);
            }
        }
        pavement.push_back({start, vertices.size(), road.style.material});
    }
    for (const auto& triangle : road.edgeTriangles)
    {
        SubmitTriangle({triangle[0].position, triangle[1].position, triangle[2].position},
                       {triangle[0].color, triangle[1].color, triangle[2].color});
    }
    solid(road.yellowStripes, yellowStripeColor);
    solid(road.whiteStripes, whiteStripeColor);
    if (selected && viewport.w > 0 && viewport.h > 0)
    {
        selectedStart = start;
        selectedEnd = vertices.size();
        const auto transform = [&](Vector3 p) { return camera->WorldToView(p); };
        AppendOutline(road, transform, focal, viewport, [&](SDL_FPoint p, double depth)
        {
            const double x = (2.0 * p.x / viewport.w - 1.0) * depth;
            const double y = (1.0 - 2.0 * p.y / viewport.h) * depth;
            const double biased = std::min(depth, worldNearDepth * 1.0001);
            outline.push_back({static_cast<float>(x), static_cast<float>(y),
                               static_cast<float>(biased), static_cast<float>(depth), EditorColors::orange});
        });
    }
}

// Draw a road in the Editor's 2D view
bool DrawPlanRoad(SDL_Renderer* renderer, const Road& road, Vector3 center, double scale, SDL_Rect viewport, RoadLayer layer)
{
    std::vector<SDL_Vertex> vertices;

    AppendRoad(road, layer, [&](Vector3 p) { return Vector3{p.x - center.x, p.z - center.z, 1.0}; }, scale, viewport, vertices);

    return Submit(renderer, vertices);
}
