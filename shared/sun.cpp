#include "sun.hpp"
#include "sky_disc.hpp"
#include "sky_location.hpp"
#include "texture_loader.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    constexpr double latitude = Math::DegreesToRadians(SkyLocation::latitudeDegrees);
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
}

Vector3 Sun::Direction(double minutes)
{
    // NOAA General Solar Position Calculations: https://gml.noaa.gov/grad/solcalc/solareqns.PDF
    constexpr double gamma = 2.0 * std::numbers::pi * (SkyLocation::dayOfYear - 1) / 365.0;
    static const double equationOfTime = 229.18 * (0.000075 + 0.001868 * std::cos(gamma) - 0.032077 * std::sin(gamma) - 0.014615 * std::cos(2 * gamma) - 0.040849 * std::sin(2 * gamma));
    static const double declination = 0.006918 - 0.399912 * std::cos(gamma) + 0.070257 * std::sin(gamma)
        - 0.006758 * std::cos(2 * gamma) + 0.000907 * std::sin(2 * gamma)
        - 0.002697 * std::cos(3 * gamma) + 0.001480 * std::sin(3 * gamma);
    const double solarMinutes = minutes + equationOfTime + 4.0 * SkyLocation::longitudeDegrees - 60.0 * SkyLocation::utcOffsetHours;
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

    if (!texture || !SDL_SetTextureScaleMode(texture.get(), SDL_SCALEMODE_LINEAR) || !discShader.Initialize(renderer, texture.get()))
    {
        texture.reset();

        return false;
    }

    return true;
}

bool Sun::Draw(SDL_Renderer* renderer, const CameraView& camera, Vector3 direction, SDL_Rect viewport, SDL_FColor baseSkyAtSun)
{
    if (!texture && !Load(renderer))
    {
        return false;
    }

    const SDL_FColor tint = Tint(std::asin(std::clamp(direction.y, -1.0, 1.0)));

    // Budget disc emission against the unbrightened SDR sky. The aureole is already
    // in the framebuffer and is intentionally excluded from this calculation.
    SDL_FColor emission{std::max(0.0f, tint.r - baseSkyAtSun.r), std::max(0.0f, tint.g - baseSkyAtSun.g), std::max(0.0f, tint.b - baseSkyAtSun.b), 1.0f};
    const auto* target = SDL_GetRenderTarget(renderer);
    const bool hdr = target && target->format == SDL_PIXELFORMAT_RGBA64_FLOAT;
    if (hdr) emission = tint;

    const SkyDisc::DiscMesh mesh = SkyDisc::Project(camera, direction, viewport, *texture, discBounds, angularDiameter, emission);

    if (mesh.count == 0) return true;
    float scale = 1;
    if (!SDL_GetRenderColorScale(renderer, &scale)) return false;
    if (hdr && !SDL_SetRenderColorScale(renderer, scale * 8.f)) return false;
    const bool drawn = discShader.Draw(renderer, texture.get(), mesh.vertices.data(), mesh.count, mesh.fan, 3 * (mesh.count - 2));
    const bool restored = !hdr || SDL_SetRenderColorScale(renderer, scale);
    return drawn && restored;
}
