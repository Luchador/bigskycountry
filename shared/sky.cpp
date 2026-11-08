#include "sky.hpp"
#include "free_camera.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    struct Palette
    {
        SDL_FColor zenith;
        SDL_FColor oppositeHorizon;
        SDL_FColor sunwardHorizon;
    };

    struct SkyStop
    {
        double sunElevationDegrees;
        Palette colors;
    };

    constexpr SDL_FColor Rgb(int r, int g, int b)
    {
        return {r / 255.0f, g / 255.0f, b / 255.0f, 1.0f};
    }

    // Display colors sampled as the sun climbs from below the horizon.
    constexpr std::array<SkyStop, 7> skyStops{{
        {-18.0, {Rgb( 6,  12,  29), Rgb( 21,  31,  52), Rgb( 21,  31,  52)}},
        {-10.0, {Rgb(16,  27,  62), Rgb( 45,  53,  83), Rgb( 73,  81, 117)}},
        { -5.0, {Rgb(31,  54, 105), Rgb(103, 105, 145), Rgb(223, 133, 114)}},
        {  0.0, {Rgb(43,  83, 143), Rgb(157, 148, 174), Rgb(255, 164,  93)}},
        {  7.0, {Rgb(46, 104, 173), Rgb(165, 185, 207), Rgb(255, 209, 157)}},
        { 20.0, {Rgb(39, 105, 181), Rgb(162, 198, 225), Rgb(221, 232, 237)}},
        { 45.0, {Rgb(32,  94, 174), Rgb(158, 199, 231), Rgb(198, 222, 240)}}
    }};

    SDL_FColor Blend(SDL_FColor a, SDL_FColor b, double t)
    {
        const float f = static_cast<float>(t);
        return {std::lerp(a.r, b.r, f), std::lerp(a.g, b.g, f), std::lerp(a.b, b.b, f), 1.0f};
    }

    Palette ColorsAt(double elevation)
    {
        for (std::size_t i = 1; i < skyStops.size(); ++i)
        {
            const SkyStop& lo = skyStops[i - 1];
            const SkyStop& hi = skyStops[i];
            if (elevation <= hi.sunElevationDegrees)
            {
                const double t = std::clamp((elevation - lo.sunElevationDegrees) /
                                            (hi.sunElevationDegrees - lo.sunElevationDegrees), 0.0, 1.0);
                const double weight = t * t * (3.0 - 2.0 * t);
                return {Blend(lo.colors.zenith, hi.colors.zenith, weight),
                        Blend(lo.colors.oppositeHorizon, hi.colors.oppositeHorizon, weight),
                        Blend(lo.colors.sunwardHorizon, hi.colors.sunwardHorizon, weight)};
            }
        }
        return skyStops.back().colors;
    }

    // Local solar time on an equinox. +X is east, +Y is up, and +Z is north.
    Vector3 SunDirection(double minutes)
    {
        constexpr double latitude = Math::DegreesToRadians(47.0);
        const double hourAngle = (minutes - 720.0) * (std::numbers::pi / 720.0);
        return {-std::sin(hourAngle), std::cos(latitude) * std::cos(hourAngle),
                -std::sin(latitude) * std::cos(hourAngle)};
    }

    SDL_FColor SampleSky(const Palette& colors, Vector3 horizontalSun, Vector3 ray)
    {
        const double facing = std::clamp(0.5 + 0.5 * Math::Dot(ray, horizontalSun), 0.0, 1.0);
        const SDL_FColor horizon = Blend(colors.oppositeHorizon, colors.sunwardHorizon, facing * facing);
        if (ray.y >= 0.0)
        {
            return Blend(horizon, colors.zenith, std::sqrt(ray.y));
        }
        // Continue the horizon into haze below eye level until terrain covers it.
        const SDL_FColor haze = Blend(colors.oppositeHorizon, colors.zenith, 0.55);
        const double depth = std::min(-ray.y * 2.0, 1.0);
        return Blend(horizon, haze, depth * (2.0 - depth));
    }
}

void Sky::Resize(int newWidth, int newHeight)
{
    width = newWidth;
    height = newHeight;
    // Tessellate in screen space. SDL interpolates the sampled colors across each triangle.
    columns = std::clamp(static_cast<int>(std::ceil(width / 40.0)), 16, 96);
    rows = std::clamp(static_cast<int>(std::ceil(height / 28.0)), 16, 80);
    vertices.resize((columns + 1) * (rows + 1));
    indices.clear();
    indices.reserve(6 * columns * rows);
    for (int y = 0; y <= rows; ++y)
    {
        for (int x = 0; x <= columns; ++x)
        {
            const int i = y * (columns + 1) + x;
            vertices[i].position = {static_cast<float>(static_cast<double>(width) * x / columns),
                                    static_cast<float>(static_cast<double>(height) * y / rows)};
            if (x < columns && y < rows)
            {
                const int below = i + columns + 1;
                indices.insert(indices.end(), {i, i + 1, below, i + 1, below + 1, below});
            }
        }
    }
}

bool Sky::Draw(SDL_Renderer* renderer, const FreeCamera& camera, double timeOfDayMinutes)
{
    int outputWidth, outputHeight;
    if (!SDL_GetCurrentRenderOutputSize(renderer, &outputWidth, &outputHeight))
    {
        return false;
    }
    if (outputWidth <= 0 || outputHeight <= 0)
    {
        return true;
    }
    if (outputWidth != width || outputHeight != height)
    {
        Resize(outputWidth, outputHeight);
    }

    const Vector3 sun = SunDirection(timeOfDayMinutes);
    const Palette colors = ColorsAt(std::asin(sun.y) * 180.0 / std::numbers::pi);
    const Vector3 horizontalSun = Math::Normalize({sun.x, 0.0, sun.z});

    // Match the road projection. View rays use camera orientation only, so the sky stays at infinity.
    const double focal = std::min(width, height) * 0.5;
    const double halfX = width * 0.5 / focal;
    const double halfY = height * 0.5 / focal;
    const Vector3 topLeft = camera.ViewRay(-halfX, halfY);
    const Vector3 stepX = (camera.ViewRay(halfX, halfY) - topLeft) * (1.0 / columns);
    const Vector3 stepY = (camera.ViewRay(-halfX, -halfY) - topLeft) * (1.0 / rows);
    for (int y = 0; y <= rows; ++y)
    {
        const Vector3 left = topLeft + stepY * y;
        for (int x = 0; x <= columns; ++x)
        {
            vertices[y * (columns + 1) + x].color = SampleSky(colors, horizontalSun, Math::Normalize(left + stepX * x));
        }
    }

    return SDL_SetRenderDrawColorFloat(renderer, colors.zenith.r, colors.zenith.g, colors.zenith.b, 1.0f) &&
           SDL_RenderClear(renderer) &&
           SDL_RenderGeometry(renderer, nullptr, vertices.data(), static_cast<int>(vertices.size()),
                              indices.data(), static_cast<int>(indices.size()));
}
