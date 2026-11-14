#include "road_render.hpp"
#include "project_settings.hpp"
#include "editor_colors.hpp"
#include "world_renderer.hpp"
#include "free_camera.hpp"
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
    void AppendOutline(const RoadGeometry& road, ViewTransform transform, double focal, SDL_Rect viewport, Emit emit)
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
            ViewGeometry::VertexPolygon<decltype(transform(triangle[0]))> p{};

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

    template<typename Vertex, typename ViewTransform>
    void AppendTriangles(const std::vector<std::array<Vertex, 3>>& triangles, SDL_FColor color, ViewTransform transform, double focal, SDL_Rect viewport, std::vector<SDL_Vertex>& vertices)
    {
        AppendMesh(triangles, transform, Clip, [&](Vector3 v)
        {
            vertices.push_back({ScreenPoint(v, focal, viewport), color, {}});
        }, focal, viewport);
    }

    using EdgePolygon = ViewGeometry::VertexPolygon<RoadGeometry::EdgeVertex>;

    EdgePolygon ClipEdge(const EdgePolygon& input, Vector3 normal, double offset)
    {
        return ViewGeometry::ClipPolygon(input, [&](const RoadGeometry::EdgeVertex& v)
        {
            return Math::Dot(v.position, normal) - offset;
        }, [](const RoadGeometry::EdgeVertex& a, const RoadGeometry::EdgeVertex& b, double t)
        {
            const float f = static_cast<float>(t);
            return RoadGeometry::EdgeVertex{a.position + (b.position - a.position) * t,
                                    {std::lerp(a.color.r, b.color.r, f), std::lerp(a.color.g, b.color.g, f),
                                     std::lerp(a.color.b, b.color.b, f), std::lerp(a.color.a, b.color.a, f)}};
        });
    }

    template <typename ViewTransform>
    void AppendRoad(const RoadGeometry& road, RoadLayer layer, Vector3 eye, ViewTransform transform, double focal, SDL_Rect viewport, std::vector<SDL_Vertex>& vertices)
    {
        if (layer == RoadLayer::Pavement)
        {
            AppendTriangles(road.pavementTriangles, roadColor, [&](const RoadGeometry::PavementVertex& v)
            {
                return transform(v.position);
            }, focal, viewport, vertices);
            const auto appendEdges = [&](const auto& triangles)
            {
                AppendMesh(triangles, [&](const auto& v)
                {
                    return RoadGeometry::EdgeVertex{transform(v.position), v.color};
                }, ClipEdge, [&](const RoadGeometry::EdgeVertex& v)
                {
                    vertices.push_back({ScreenPoint(v.position, focal, viewport), v.color, {}});
                }, focal, viewport);
            };
            appendEdges(road.edgeTriangles);
            for (const auto& detail : road.railroadSegments)
            {
                if (!detail.WithinDistance(eye, g_ProjectSettings.RailroadDetailDistance())) continue;
                // Draw sleepers before rails in the top-down view.
                for (const auto& run : detail.ties)
                {
                    const auto center = ScreenPoint(transform(run.Center()), focal, viewport);
                    const double margin = run.Radius() * focal;
                    if (focal * RailroadSettings::tieSpacing < 0.5 ||
                        center.x < -margin || center.x > viewport.w + margin ||
                        center.y < -margin || center.y > viewport.h + margin) continue;
                    const auto& mesh = *RailroadTieMesh();
                    for (int i = 0; i < run.count; ++i)
                    {
                        const auto tie = run.At(i);
                        for (int corner : {0, 1, 2, 0, 2, 3})
                        {
                            const auto p = transform(tie.Point(mesh.vertices[corner].position));
                            vertices.push_back({ScreenPoint(p, focal, viewport), {0.36f, 0.27f, 0.20f, 1}, {}});
                        }
                    }
                }
                appendEdges(detail.rails);
            }
            for (const auto& [path, strip] : road.transitionStrips) appendEdges(strip);
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

void WorldRenderer::SubmitRoad(const RoadGeometry& road, const std::string& material, bool selected)
{
    if (!BoundsVisible(road.bounds)) return;
    // The 2D plan uses display colors. Convert those swatches to albedo for the 3D lighting pass.
    static const auto asphalt = Srgb::ToLinear(roadColor);
    static const auto yellowPaint = Srgb::ToLinear(yellowStripeColor);
    static const auto whitePaint = Srgb::ToLinear(whiteStripeColor);
    const auto solid = [&](const std::vector<RoadGeometry::Triangle>& triangles, SDL_FColor color)
    {
        for (const auto& t : triangles)
        {
            SubmitTriangle(t[0], t[1], t[2], color);
        }
    };
    const std::size_t start = vertices.size();
    if (!road.pavementTriangles.empty())
    {
        const std::array fallback{asphalt, asphalt, asphalt};
        for (const auto& t : road.pavementTriangles)
        {
            SubmitMaterialTriangle({t[0].position, t[1].position, t[2].position}, fallback,
                                   {t[0].uv, t[1].uv, t[2].uv});
        }
        if (!material.empty()) materialRanges.push_back({start, vertices.size(), material});
    }
    const auto submitEdges = [&](const RoadGeometry::ColoredSurface& surface)
    {
        for (const auto& triangle : surface)
        {
            SubmitTriangle({triangle[0].position, triangle[1].position, triangle[2].position},
                           {Srgb::ToLinear(triangle[0].color), Srgb::ToLinear(triangle[1].color), Srgb::ToLinear(triangle[2].color)});
        }
    };
    submitEdges(road.edgeTriangles);
    for (const auto& [path, strip] : road.transitionStrips)
    {
        const auto first = vertices.size();
        for (const auto& t : strip)
        {
            SubmitMaterialTriangle({t[0].position, t[1].position, t[2].position},
                                   {t[0].color, t[1].color, t[2].color}, {t[0].uv, t[1].uv, t[2].uv});
        }
        if (!path.empty() && vertices.size() > first)
            materialRanges.push_back({first, vertices.size(), path, nullptr, true});
    }
    for (const auto& detail : road.railroadSegments)
    {
        if (!detail.WithinDistance(camera->GetPosition(), g_ProjectSettings.RailroadDetailDistance())) continue;
        submitEdges(detail.rails);
        for (const auto& run : detail.ties)
        {
            const double radius = run.Radius();
            const auto center = run.Center();
            // Once the spacing projects to less than half a pixel, the bed and rails carry the silhouette.
            if (!SphereVisible(center, radius) || camera->WorldToView(center).z - radius > 2 * focal * RailroadSettings::tieSpacing)
                continue;
            for (int i = 0; i < run.count; ++i) SubmitMesh({RailroadTieMesh(), run.At(i)});
        }
    }
    solid(road.yellowStripes, yellowPaint);
    solid(road.whiteStripes, whitePaint);
    if (selected && start < vertices.size() && viewport.w > 0 && viewport.h > 0)
    {
        if (!selectedRanges.empty() && selectedRanges.back().end == start)
            selectedRanges.back().end = vertices.size();
        else
            selectedRanges.push_back({start, vertices.size()});
        const auto transform = [&](Vector3 p) { return camera->WorldToView(p); };
        AppendOutline(road, transform, focal, viewport, [&](SDL_FPoint p, double depth)
        {
            const double x = (2.0 * p.x / viewport.w - 1.0) * depth;
            const double y = (1.0 - 2.0 * p.y / viewport.h) * depth;
            // Keep the outline visible over raised edge paint at close range.
            const double strokeDepth = std::max(worldNearDepth, depth - RoadStyle::stripeHeight);
            const double biased = std::min(depth, worldNearDepth * 1.0001 * depth / strokeDepth);
            outline.push_back({static_cast<float>(x), static_cast<float>(y),
                               static_cast<float>(biased), static_cast<float>(depth), EditorColors::orange});
        });
    }
}

// Draw a road in the Editor's 2D view
bool DrawPlanRoad(SDL_Renderer* renderer, const RoadGeometry& road, Vector3 center, double scale, SDL_Rect viewport, RoadLayer layer)
{
    std::vector<SDL_Vertex> vertices;

    // The orthographic plan has no 3D camera. Use the height that gives the
    // same meters per pixel at its focus plane as the perspective viewport.
    const double height = 0.5 * std::min(viewport.w, viewport.h) / scale;
    AppendRoad(road, layer, center + Vector3{0, height, 0}, [&](Vector3 p) { return Vector3{p.x - center.x, p.z - center.z, 1.0}; }, scale, viewport, vertices);

    return Submit(renderer, vertices);
}
