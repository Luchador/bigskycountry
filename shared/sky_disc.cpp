#include "sky_disc.hpp"
#include "free_camera.hpp"
#include "view_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

SkyDisc::DiscMesh SkyDisc::Project(const FreeCamera& camera, Vector3 direction, SDL_Rect viewport, const SDL_Texture& texture, SDL_Rect discBounds, double angularDiameter, SDL_FColor color)
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
        // Return an empty mesh.
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

    DiscMesh mesh;
    mesh.count = quad.count;

    for (int i = 0; i < quad.count; ++i)
    {
        const Vector3 p = quad.points[i];
        const float tx = static_cast<float>((cx + Math::Dot(p, u) / unitsPerTexel) / texture.w);
        const float ty = static_cast<float>((cy - Math::Dot(p, v) / unitsPerTexel) / texture.h);

        mesh.vertices[i] = {ViewGeometry::ScreenPoint(p, focal, viewport), color, {std::clamp(tx, 0.0f, 1.0f), std::clamp(ty, 0.0f, 1.0f)}};
    }

    return mesh;
}
