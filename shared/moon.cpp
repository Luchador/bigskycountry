#include "moon.hpp"
#include "sky_disc.hpp"
#include "texture_loader.hpp"

#include <cmath>

namespace
{
    constexpr double angularDiameter = Math::DegreesToRadians(3.0);
    // The moon's center crosses the horizon when the sun is four degrees above it.
    constexpr double overlapElevation = Math::DegreesToRadians(4.0);
}

Vector3 Moon::Direction(Vector3 sunDirection)
{
    // Opposite azimuth puts moonrise in the east and moonset in the west.
    static const double horizonOffset = std::sin(overlapElevation);
    return Math::Normalize({-sunDirection.x, horizonOffset - sunDirection.y, -sunDirection.z});
}

bool Moon::Load(SDL_Renderer* renderer)
{
    texture.reset(TextureLoader::Load(renderer, "world/moon.tga", discBounds));

    if (!texture || !SDL_SetTextureScaleMode(texture.get(), SDL_SCALEMODE_LINEAR) || !SDL_SetTextureBlendMode(texture.get(), SDL_BLENDMODE_BLEND))
    {
        texture.reset();

        return false;
    }

    return true;
}

bool Moon::Draw(SDL_Renderer* renderer, const FreeCamera& camera, Vector3 direction, SDL_Rect viewport)
{
    if (!texture && !Load(renderer))
    {
        return false;
    }

    const SkyDisc::Mesh mesh = SkyDisc::Project(camera, direction, viewport, *texture, discBounds, angularDiameter, {1.0f, 1.0f, 1.0f, 1.0f});

    return mesh.count == 0 || SDL_RenderGeometry(renderer, texture.get(), mesh.vertices.data(), mesh.count,  mesh.fan, 3 * (mesh.count - 2));
}
