#include "sun.hpp"
#include "free_camera.hpp"
#include "texture_loader.hpp"
#include "view_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr double latitude = Math::DegreesToRadians(38.0);
    constexpr double longitudeDegrees = -95.0;
    constexpr double utcOffsetHours = -5.0; // Central Daylight Time on August 1.
    constexpr int dayOfYear = 213; // August 1 of a non-leap year, repeated every game day.
    constexpr double angularDiameter = Math::DegreesToRadians(3.0);

    // NOAA's approximate refraction correction, in arcseconds, for a geometric elevation in degrees.
    double Refraction(double h)
    {
        if (h > 85.0)
        {
            return 0.0;
        }

        if (h >= -0.575 && h <= 5.0)
        {
            return 1735.0 + h * (-518.2 + h * (103.4 + h * (-12.79 + h * 0.711)));
        }

        const double cot = 1.0 / std::tan(Math::DegreesToRadians(h));

        if (h < -0.575)
        {
            return -20.774 * cot;
        }

        const double cot2 = cot * cot;

        return cot * (58.1 + cot2 * (-0.07 + 0.000086 * cot2));
    }

    // The project's assets directory is optional in a fresh checkout.
    SDL_Texture* MakeFallback(SDL_Renderer* renderer, SDL_Rect& bounds)
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
}

Vector3 Sun::Direction(double minutes)
{
    // NOAA General Solar Position Calculations: https://gml.noaa.gov/grad/solcalc/solareqns.PDF
    constexpr double gamma = 2.0 * std::numbers::pi * (dayOfYear - 1) / 365.0;
    static const double equationOfTime = 229.18 * (0.000075 + 0.001868 * std::cos(gamma) - 0.032077 * std::sin(gamma) - 0.014615 * std::cos(2 * gamma) - 0.040849 * std::sin(2 * gamma));
    static const double declination = 0.006918 - 0.399912 * std::cos(gamma) + 0.070257 * std::sin(gamma)
        - 0.006758 * std::cos(2 * gamma) + 0.000907 * std::sin(2 * gamma)
        - 0.002697 * std::cos(3 * gamma) + 0.001480 * std::sin(3 * gamma);
    const double solarMinutes = minutes + equationOfTime + 4.0 * longitudeDegrees - 60.0 * utcOffsetHours;
    const double hourAngle = Math::DegreesToRadians(solarMinutes / 4.0 - 180.0);
    const double meridian = std::cos(declination) * std::cos(hourAngle);
    const double east = -std::cos(declination) * std::sin(hourAngle);
    const double up = std::sin(latitude) * std::sin(declination) + std::cos(latitude) * meridian;
    const double north = std::cos(latitude) * std::sin(declination) - std::sin(latitude) * meridian;

    const double elevation = std::asin(std::clamp(up, -1.0, 1.0));
    const double apparent = elevation + Math::DegreesToRadians(Refraction(elevation * 180.0 / std::numbers::pi) / 3600.0);
    const double horizontalScale = std::cos(apparent) / std::hypot(east, north);
    return {east * horizontalScale, std::sin(apparent), north * horizontalScale};
}

SDL_FColor Sun::Tint(double elevationRadians)
{
    // Orange at the horizon, warm yellow low in the sky, white above 30 degrees.
    const float low = static_cast<float>(1.0 - std::clamp(elevationRadians / Math::DegreesToRadians(30.0), 0.0, 1.0));
    const float low2 = low * low;
    return {1.0f, 1.0f - 0.29f * low2 * low2, 1.0f - 0.74f * low2, 1.0f};
}

bool Sun::Load(SDL_Renderer* renderer)
{
    texture.reset(TextureLoader::LoadMask(renderer, "world/sun.tga", discBounds));
    if (!texture)
    {
        SDL_LogWarn(SDL_LOG_CATEGORY_RENDER, "Using a built-in sun disc: %s", SDL_GetError());
        texture.reset(MakeFallback(renderer, discBounds));
    }
    return texture && SDL_SetTextureBlendMode(texture.get(), SDL_BLENDMODE_BLEND) &&
           SDL_SetTextureScaleMode(texture.get(), SDL_SCALEMODE_LINEAR);
}

bool Sun::Draw(SDL_Renderer* renderer, const FreeCamera& camera, Vector3 direction, SDL_Rect viewport)
{
    if (!texture && !Load(renderer))
    {
        return false;
    }

    const Vector3 right = (direction.x == 0.0 && direction.z == 0.0) ? Vector3{1.0, 0.0, 0.0} : Math::Normalize(Math::Cross({0.0, 1.0, 0.0}, direction));
    const Vector3 up = Math::Cross(direction, right);
    // Scale the bright disc to the requested angle while retaining its original soft fringe.
    const double unitsPerTexel = 2.0 * std::tan(angularDiameter * 0.5) / std::max(discBounds.w, discBounds.h);
    const double cx = discBounds.x + 0.5 * discBounds.w, cy = discBounds.y + 0.5 * discBounds.h;
    const double left = -cx * unitsPerTexel, rightEdge = (texture->w - cx) * unitsPerTexel;
    const double top = cy * unitsPerTexel;

    // A point on the sprite has world height direction.y + y * up.y. Cut the sprite at height zero.
    if (direction.y + top * up.y <= 0.0)
    {
        return true;
    }

    double bottom = (cy - texture->h) * unitsPerTexel;

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
            return true;
        }
    }

    std::array<SDL_Vertex, 8> vertices{};
    const SDL_FColor tint = Tint(std::asin(std::clamp(direction.y, -1.0, 1.0)));

    for (int i = 0; i < quad.count; ++i)
    {
        const Vector3 p = quad.points[i];
        const float tx = static_cast<float>((cx + Math::Dot(p, u) / unitsPerTexel) / texture->w);
        const float ty = static_cast<float>((cy - Math::Dot(p, v) / unitsPerTexel) / texture->h);
        vertices[i] = {ViewGeometry::ScreenPoint(p, focal, viewport), tint,
                       {std::clamp(tx, 0.0f, 1.0f), std::clamp(ty, 0.0f, 1.0f)}};
    }

    constexpr int fan[] = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 7};
    return SDL_RenderGeometry(renderer, texture.get(), vertices.data(), quad.count, fan, 3 * (quad.count - 2));
}
