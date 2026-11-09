#include "road_render.hpp"
#include "road_depth.hpp"
#include "free_camera.hpp"
#include "view_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr SDL_FColor roadColor{0.43f, 0.43f, 0.43f, 1.0f};
    constexpr SDL_FColor yellowStripeColor{0.96f, 0.85f, 0.12f, 1.0f};
    constexpr SDL_FColor whiteStripeColor{0.9f, 0.9f, 0.9f, 1.0f};
    constexpr SDL_FColor selectionColor{1.0f, 0.5f, 0.05f, 1.0f};

    using ViewGeometry::Clip;
    using ViewGeometry::ScreenPoint;
    using ViewGeometry::SidePlanes;

    bool ClipLine(Vector3& a, Vector3& b, Vector3 normal, double offset)
    {
        const double da = Math::Dot(a, normal) - offset;
        const double db = Math::Dot(b, normal) - offset;
        if (da < 0.0 && db < 0.0)
        {
            return false;
        }
        if ((da < 0.0) != (db < 0.0))
        {
            const Vector3 cut = a + (b - a) * (da / (da - db));
            if (da < 0.0)
            {
                a = cut;
            }
            else
            {
                b = cut;
            }
        }
        return true;
    }

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
            if (!ClipLine(a, b, {0, 0, 1}, roadNearDepth) ||
                !std::all_of(sides.begin(), sides.end(), [&](Vector3 side) { return ClipLine(a, b, side, 0.0); }))
            {
                continue;
            }

            const SDL_FPoint p = ScreenPoint(a, focal, viewport), q = ScreenPoint(b, focal, viewport);
            const float dx = q.x - p.x, dy = q.y - p.y;
            const float length = std::hypot(dx, dy);
            if (length == 0.0f)
            {
                continue;
            }
            // Square caps overlap at the joins. The width stays three pixels at any zoom.
            const float tx = 1.5f * dx / length, ty = 1.5f * dy / length;
            const SDL_FPoint corners[] = {{p.x - tx - ty, p.y - ty + tx}, {p.x - tx + ty, p.y - ty - tx},
                                          {q.x + tx + ty, q.y + ty - tx}, {q.x + tx - ty, q.y + ty + tx}};
            for (int i : {0, 1, 2, 0, 2, 3})
            {
                emit(corners[i], i < 2 ? a.z : b.z);
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

            p = clip(p, {0, 0, 1}, roadNearDepth);

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
                vertices.push_back({p, selectionColor, {}});
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

bool RoadDepthRenderer::Draw(SDL_Renderer* renderer, const FreeCamera& camera, const std::vector<Road>& roads, const Road* selected)
{
    SDL_Rect viewport{};
    if (!SDL_GetRenderViewport(renderer, &viewport))
    {
        return false;
    }
    if (roads.empty() || viewport.w <= 0 || viewport.h <= 0)
    {
        return true;
    }
    viewport.x = viewport.y = 0;
    const double focal = std::min(viewport.w, viewport.h) * 0.5;
    const double scaleX = 2.0 * focal / viewport.w, scaleY = 2.0 * focal / viewport.h;
    const auto transform = [&](Vector3 p) { return camera.WorldToView(p); };
    vertices.clear();
    const auto emit = [&](Vector3 point, SDL_FColor color)
    {
        const Vector3 v = transform(point);
        vertices.push_back({static_cast<float>(scaleX * v.x), static_cast<float>(scaleY * v.y),
                            static_cast<float>(roadNearDepth), static_cast<float>(v.z), color});
    };
    const auto emitSolid = [&](const std::vector<Road::Triangle>& mesh, SDL_FColor color)
    {
        for (const auto& triangle : mesh)
        {
            for (Vector3 p : triangle)
            {
                emit(p, color);
            }
        }
    };

    selectedStart = selectedEnd = 0;
    for (const Road& road : roads)
    {
        const std::size_t start = vertices.size();
        emitSolid(road.triangles, roadColor);
        for (const auto& triangle : road.edgeTriangles)
        {
            for (const auto& v : triangle)
            {
                emit(v.position, v.color);
            }
        }
        emitSolid(road.yellowStripes, yellowStripeColor);
        emitSolid(road.whiteStripes, whiteStripeColor);
        if (&road == selected)
        {
            selectedStart = start;
            selectedEnd = vertices.size();
        }
    }
    outlineStart = vertices.size();
    if (vertices.empty())
    {
        return true;
    }
    if (selected)
    {
        // Keep the three-pixel editor stroke in screen space, with the depth
        // of its original edge endpoints for occlusion by the road itself.
        AppendOutline(*selected, transform, focal, viewport, [&](SDL_FPoint p, double depth)
        {
            const double x = (2.0 * p.x / viewport.w - 1.0) * depth;
            const double y = (1.0 - 2.0 * p.y / viewport.h) * depth;
            const double biased = std::min(depth, roadNearDepth * 1.0001);
            vertices.push_back({static_cast<float>(x), static_cast<float>(y),
                                static_cast<float>(biased), static_cast<float>(depth), selectionColor});
        });
    }
    const SDL_FRect dest{0, 0, static_cast<float>(viewport.w), static_cast<float>(viewport.h)};
    return PrepareTarget(renderer, viewport.w, viewport.h) && (device ? DrawGPU() : DrawSoftware()) &&
           SDL_RenderTexture(renderer, colorTarget, nullptr, &dest);
}

// Draw a road in the Editor's 2D view
bool DrawPlanRoad(SDL_Renderer* renderer, const Road& road, Vector3 center, double scale, SDL_Rect viewport, RoadLayer layer)
{
    std::vector<SDL_Vertex> vertices;

    AppendRoad(road, layer, [&](Vector3 p) { return Vector3{p.x - center.x, p.z - center.z, 1.0}; }, scale, viewport, vertices);

    return Submit(renderer, vertices);
}
