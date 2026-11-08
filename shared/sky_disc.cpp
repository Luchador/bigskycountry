#include "sky_disc.hpp"
#include "free_camera.hpp"
#include "view_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

SDL_Texture* SkyDisc::MakeFallback(SDL_Renderer* renderer, SDL_Rect& bounds)
{
    constexpr int size = 64;
    std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface(SDL_CreateSurface(size, size, SDL_PIXELFORMAT_RGBA32), SDL_DestroySurface);

    if (!surface)
    {
        return nullptr;
    }

    for (int y = 0; y < size; ++y)
    {
        auto* pixel = static_cast<Uint8*>(surface->pixels) + y * surface->pitch;
        for (int x = 0; x < size; ++x, pixel += 4)
        {
            const double radius = std::hypot(x + 0.5 - size * 0.5, y + 0.5 - size * 0.5);
            pixel[0] = pixel[1] = pixel[2] = 255;
            pixel[3] = static_cast<Uint8>(255 * std::clamp(size * 0.5 + 0.5 - radius, 0.0, 1.0));
        }
    }

    bounds = {0, 0, size, size};
    return SDL_CreateTextureFromSurface(renderer, surface.get());
}

SkyDisc::Mesh SkyDisc::Project(const FreeCamera& camera, Vector3 direction, SDL_Rect viewport, const SDL_Texture& texture,
                               SDL_Rect discBounds, double angularDiameter, SDL_FColor color)
{
    const Vector3 right = (direction.x == 0.0 && direction.z == 0.0) ? Vector3{1.0, 0.0, 0.0} : Math::Normalize(Math::Cross({0.0, 1.0, 0.0}, direction));
    const Vector3 up = Math::Cross(direction, right);
    // Keep transparent padding outside the specified angular diameter.
    const double unitsPerTexel = 2.0 * std::tan(angularDiameter * 0.5) / std::max(discBounds.w, discBounds.h);
    const double cx = discBounds.x + 0.5 * discBounds.w, cy = discBounds.y + 0.5 * discBounds.h;
    const double left = -cx * unitsPerTexel, rightEdge = (texture.w - cx) * unitsPerTexel;
    const double top = cy * unitsPerTexel;

    // A point on the sprite has world height direction.y + y * up.y. Cut the sprite at height zero.
    if (direction.y + top * up.y <= 0.0)
    {
        return {};
    }

    double bottom = (cy - texture.h) * unitsPerTexel;

    if (direction.y + bottom * up.y < 0.0)
    {
        bottom = -direction.y / up.y;
    }

    const Vector3 center = camera.WorldDirectionToView(direction);
    const Vector3 u = camera.WorldDirectionToView(right), v = camera.WorldDirectionToView(up);
    ViewGeometry::Polygon quad{};
    quad.points[0] = center + u * left + v * top;
    quad.points[1] = center + u * rightEdge + v * top;
    quad.points[2] = center + u * rightEdge + v * bottom;
    quad.points[3] = center + u * left + v * bottom;
    quad.count = 4;
    const double focal = std::min(viewport.w, viewport.h) * 0.5;

    for (Vector3 normal : ViewGeometry::SidePlanes(focal, viewport))
    {
        quad = ViewGeometry::Clip(quad, normal, 0.0);
        if (quad.count < 3)
        {
            return {};
        }
    }

    Mesh mesh;
    mesh.count = quad.count;
    for (int i = 0; i < quad.count; ++i)
    {
        const Vector3 p = quad.points[i];
        const float tx = static_cast<float>((cx + Math::Dot(p, u) / unitsPerTexel) / texture.w);
        const float ty = static_cast<float>((cy - Math::Dot(p, v) / unitsPerTexel) / texture.h);
        mesh.vertices[i] = {ViewGeometry::ScreenPoint(p, focal, viewport), color,
                            {std::clamp(tx, 0.0f, 1.0f), std::clamp(ty, 0.0f, 1.0f)}};
    }

    return mesh;
}
