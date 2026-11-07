#include "road_render.hpp"
#include "free_camera.hpp"

#include <algorithm>

namespace
{
    constexpr SDL_FColor roadColor{0.43f, 0.43f, 0.43f, 1.0f};

    struct Polygon
    {
        // Clipping a triangle against five planes can introduce at most five vertices.
        std::array<Vector3, 8> points;
        int count = 0;
    };

    // Cut the polygon to prevent glitches when the near camera plane intersects the polygon.
    Polygon Clip(const Polygon& input, Vector3 normal, double offset)
    {
        Polygon result{};
        Vector3 previous = input.points[input.count - 1];

        double previousDistance = Math::Dot(previous, normal) - offset;

        for (int i = 0; i < input.count; ++i)
        {
            const Vector3 current = input.points[i];
            const double distance = Math::Dot(current, normal) - offset;
    
            if ((distance < 0.0 && previousDistance > 0.0) || (distance > 0.0 && previousDistance < 0.0))
            {
                const double t = previousDistance / (previousDistance - distance);
                result.points[result.count++] = previous + (current - previous) * t;
            }

            if (distance >= 0.0)
            {
                result.points[result.count++] = current;
            }

            previous = current;
            previousDistance = distance;
        }

        return result;
    }

    template <typename ViewTransform>

    void AppendRoad(const Road& road, ViewTransform transform, double focal, SDL_Rect viewport, std::vector<SDL_Vertex>& vertices)
    {

        const double halfWidth = viewport.w * 0.5, halfHeight = viewport.h * 0.5;
        const Vector3 sides[] = {{focal, 0, halfWidth}, {-focal, 0, halfWidth}, {0, focal, halfHeight}, {0, -focal, halfHeight}};

        for (const auto& triangle : road.triangles)
        {
            Polygon p{};

            for (const Vector3& point : triangle)
            {
                p.points[p.count++] = transform(point);
            }

            p = Clip(p, {0, 0, 1}, 0.1);

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
                    vertices.push_back({{static_cast<float>(viewport.x + halfWidth + v.x * focal / v.z),
                                         static_cast<float>(viewport.y + halfHeight - v.y * focal / v.z)},
                                        roadColor,
                                        {}});
                }
            }
        }
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

    for (const Road& road : map.roads)
    {
        AppendRoad(road, [&](Vector3 p) { return camera.WorldToView(p); }, std::min(viewport.w, viewport.h) * 0.5, viewport, vertices);
    }

    return Submit(renderer, vertices);
}

// Draw a road in the Editor's 2D view
bool DrawPlanRoad(SDL_Renderer* renderer, const Road& road, Vector3 center, double scale, SDL_Rect viewport)
{
    std::vector<SDL_Vertex> vertices;

    AppendRoad(road, [&](Vector3 p) { return Vector3{p.x - center.x, p.z - center.z, 1.0}; }, scale, viewport, vertices);

    return Submit(renderer, vertices);
}
