#include "road_render.hpp"
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
    using ViewGeometry::Polygon;
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

    template <typename ViewTransform>
    void AppendOutline(const Road& road, ViewTransform transform, double focal, SDL_Rect viewport,
                       std::vector<SDL_Vertex>& vertices)
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
                vertices.push_back({corners[i], selectionColor, {}});
            }
        }
    }

    template <typename ViewTransform>
    void AppendTriangles(const std::vector<Road::Triangle>& triangles, SDL_FColor color, ViewTransform transform,
                         double focal, SDL_Rect viewport, std::vector<SDL_Vertex>& vertices)
    {
        const auto sides = SidePlanes(focal, viewport);

        for (const auto& triangle : triangles)
        {
            Polygon p{};

            for (const Vector3& point : triangle)
            {
                p.points[p.count++] = transform(point);
            }

            p = Clip(p, {0, 0, 1}, roadNearDepth);

            for (Vector3 side : sides)
            {
                if (p.count < 3)
                {
                    break;
                }
                p = Clip(p, side, 0.0);
            }

            for (int i = 2; i < p.count; ++i)
            {
                for (int index : {0, i - 1, i})
                {
                    const Vector3 v = p.points[index];
                    vertices.push_back({ScreenPoint(v, focal, viewport), color, {}});
                }
            }
        }
    }

    template <typename ViewTransform>
    void AppendRoad(const Road& road, RoadLayer layer, ViewTransform transform, double focal, SDL_Rect viewport,
                    std::vector<SDL_Vertex>& vertices)
    {
        if (layer == RoadLayer::Pavement)
        {
            AppendTriangles(road.triangles, roadColor, transform, focal, viewport, vertices);
            return;
        }
        if (layer == RoadLayer::Outline)
        {
            AppendOutline(road, transform, focal, viewport, vertices);
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

bool DrawWorldRoads(SDL_Renderer* renderer, const FreeCamera& camera, const RoadMap& map)
{
    if (map.roads.empty())
    {
        return true;
    }

    SDL_Rect viewport{};

    if (!SDL_GetCurrentRenderOutputSize(renderer, &viewport.w, &viewport.h))
    {
        return false;
    }

    std::vector<SDL_Vertex> vertices;

    const auto transform = [&](Vector3 p) { return camera.WorldToView(p); };
    const double focal = std::min(viewport.w, viewport.h) * 0.5;
    // SDL_RenderGeometry draws in submission order. Paint belongs above all pavement.
    for (RoadLayer layer : {RoadLayer::Pavement, RoadLayer::Stripes})
    {
        for (const Road& road : map.roads)
        {
            AppendRoad(road, layer, transform, focal, viewport, vertices);
        }
    }

    return Submit(renderer, vertices);
}

bool DrawWorldRoadOutline(SDL_Renderer* renderer, const FreeCamera& camera, const Road& road, SDL_Rect viewport)
{
    std::vector<SDL_Vertex> vertices;
    const double focal = std::min(viewport.w, viewport.h) * 0.5;
    AppendOutline(road, [&](Vector3 p) { return camera.WorldToView(p); }, focal, viewport, vertices);
    return Submit(renderer, vertices);
}

// Draw a road in the Editor's 2D view
bool DrawPlanRoad(SDL_Renderer* renderer, const Road& road, Vector3 center, double scale, SDL_Rect viewport, RoadLayer layer)
{
    std::vector<SDL_Vertex> vertices;

    AppendRoad(road, layer, [&](Vector3 p) { return Vector3{p.x - center.x, p.z - center.z, 1.0}; }, scale, viewport, vertices);

    return Submit(renderer, vertices);
}
