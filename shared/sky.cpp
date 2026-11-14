/**
 * The sky is a flat grid of triangles covering the 3D viewport. The vertices are screen-positioned.
 */

#include "sky.hpp"
#include "solar_aureole.hpp"
#include "camera_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    struct SkyStop
    {
        double sunElevationDegrees;
        SkyPalette colors;
    };

    constexpr SDL_FColor Rgb(int r, int g, int b)
    {
        return {r / 255.0f, g / 255.0f, b / 255.0f, 1.0f};
    }

    constexpr SDL_FColor lunarAureoleTint = Rgb(201, 215, 236);
    constexpr double lunarAureoleStrengthScale = 0.5;

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

    SkyPalette ColorsAt(double elevation)
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

    SDL_FColor SampleSky(const SkyPalette& colors, Vector3 horizontalSun, Vector3 ray)
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

    // Place extra sample lines through the projected light source and its close surroundings.
    // Their spacing is angular so it stays useful after resizes and near the field-of-view edges.
    void AddAureoleSamples(std::vector<double>& samples, double centerAngle, int size, double focal)
    {
        constexpr std::array<double, 11> offsets = {-24, -16, -10, -6, -3, 0, 3, 6, 10, 16, 24};
        const double halfFov = std::atan2(size * 0.5, focal);
        for (double degrees : offsets)
        {
            const double angle = centerAngle + Math::DegreesToRadians(degrees);
            if (angle > -halfFov && angle < halfFov)
            {
                samples.push_back(size * 0.5 + focal * std::tan(angle));
            }
        }
        std::sort(samples.begin(), samples.end());
        samples.erase(std::unique(samples.begin(), samples.end()), samples.end());
    }
}

SkyEnvironment SkyEnvironment::At(double timeOfDayMinutes)
{
    SkyEnvironment sky{};
    const Vector3 sun = Sun::Direction(timeOfDayMinutes);
    const Vector3 moon = Moon::Direction(sun);
    const double elevation = std::asin(std::clamp(sun.y, -1.0, 1.0));
    sky.sunTint = Sun::Tint(elevation);
    sky.colors = ColorsAt(elevation * 180.0 / std::numbers::pi);

    // Linear intensities relative to a white, fully illuminated diffuse surface.
    constexpr double sunlight = 0.90, moonlight = 0.12, skylight = 0.60;
    const auto linearRgb = [](SDL_FColor color) -> Vector3
    {
        const auto c = Srgb::ToLinear(color);
        return {c.r, c.g, c.b};
    };
    const auto direct = [&](Vector3 direction, SDL_FColor tint, double intensity) -> DirectionalLight
    {
        // Fade the direct light in over the first five degrees above its own horizon.
        static const double fullHeight = std::sin(Math::DegreesToRadians(5.0));
        const double t = std::clamp(direction.y / fullHeight, 0.0, 1.0);
        return {direction, linearRgb(tint) * (intensity * t * t * (3.0 - 2.0 * t))};
    };
    sky.lights.sun = direct(sun, sky.sunTint, sunlight);
    sky.lights.moon = direct(moon, lunarAureoleTint, moonlight);
    sky.lights.ambient = (linearRgb(sky.colors.zenith) * 0.5 + linearRgb(sky.colors.oppositeHorizon) * 0.25 +
                          linearRgb(sky.colors.sunwardHorizon) * 0.25) * skylight;
    return sky;
}

void Sky::UpdateGrid()
{
    const int newColumns = static_cast<int>(sampleX.size()) - 1;
    const int newRows = static_cast<int>(sampleY.size()) - 1;
    if (columns == newColumns && rows == newRows)
    {
        return;
    }
    columns = newColumns;
    rows = newRows;
    vertices.resize((columns + 1) * (rows + 1));
    indices.clear();
    indices.reserve(6 * columns * rows);

    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < columns; ++x)
        {
            const int i = y * (columns + 1) + x;
            const int below = i + columns + 1;
            indices.insert(indices.end(), {i, i + 1, below, i + 1, below + 1, below});
        }
    }
}

bool Sky::Draw(SDL_Renderer* renderer, const CameraView& camera, double timeOfDayMinutes)
{
    SDL_Rect content;
    if (!SDL_GetRenderViewport(renderer, &content))
    {
        return false;
    }

    const int width = content.w;
    const int height = content.h;
    if (width <= 0 || height <= 0)
    {
        return true;
    }

    // Translation cannot change the sky. Reuse the gradient while the editor
    // clock, camera orientation and viewport size are all unchanged.
    const Vector3 direction = camera.ViewRay(0, 0);
    const bool timeChanged = timeOfDayMinutes != lastTime;
    if (timeChanged)
    {
        environment = SkyEnvironment::At(timeOfDayMinutes);
        lastTime = timeOfDayMinutes;
    }
    if (timeChanged || direction != lastDirection || width != lastWidth || height != lastHeight)
    {
        BuildGradient(camera, width, height);
        lastDirection = direction;
        lastWidth = width;
        lastHeight = height;
    }
    const Vector3 sunDirection = environment.lights.sun.direction;
    const Vector3 moonDirection = environment.lights.moon.direction;
    const Vector3 horizontalSun = Math::Normalize({sunDirection.x, 0.0, sunDirection.z});
    const SkyPalette& colors = environment.colors;
    const SDL_Rect viewport{0, 0, width, height};
    return SDL_SetRenderDrawColorFloat(renderer, colors.zenith.r, colors.zenith.g, colors.zenith.b, 1.0f) &&
           SDL_RenderClear(renderer) &&
           SDL_RenderGeometry(renderer, nullptr, vertices.data(), static_cast<int>(vertices.size()), indices.data(), static_cast<int>(indices.size())) && // Draw sky mesh
           sun.Draw(renderer, camera, sunDirection, viewport, SampleSky(colors, horizontalSun, sunDirection)) && // Draw sun
           moon.Draw(renderer, camera, moonDirection, viewport); // Draw full moon
}

void Sky::BuildGradient(const CameraView& camera, int width, int height)
{
    const Vector3 sunDirection = environment.lights.sun.direction;
    const Vector3 moonDirection = environment.lights.moon.direction;
    const SkyPalette& colors = environment.colors;
    const SolarAureole solarAureole(sunDirection, environment.sunTint);
    const SolarAureole lunarAureole(moonDirection, lunarAureoleTint, lunarAureoleStrengthScale);
    const Vector3 horizontalSun = Math::Normalize({sunDirection.x, 0.0, sunDirection.z});

    // Match the road projection. View rays use camera orientation only, so the sky stays at infinity.
    const double focal = std::min(width, height) * 0.5;
    const double halfX = width * 0.5 / focal;
    const double halfY = height * 0.5 / focal;
    const Vector3 topLeft = camera.ViewRay(-halfX, halfY);
    const Vector3 leftToRight = camera.ViewRay(halfX, halfY) - topLeft;
    const Vector3 topToBottom = camera.ViewRay(-halfX, -halfY) - topLeft;

    // With a roll-free camera all rays on one screen row have the same vertical component.
    // Pin a row to the zero crossing so the brightest sky color always lands on the horizon.
    const double horizon = std::clamp(-topLeft.y / topToBottom.y, 0.0, 1.0);
    const int baseColumns = std::clamp(static_cast<int>(std::ceil(width / 40.0)), 16, 96);
    const int baseRows = std::clamp(static_cast<int>(std::ceil(height / 28.0)), 16, 80);

    sampleX.clear();
    sampleY.clear();

    for (int x = 0; x <= baseColumns; ++x)
    {
        sampleX.push_back(static_cast<double>(width) * x / baseColumns);
    }

    const int horizonRow = baseRows / 2;
    const int firstRow = horizon == 0.0 ? horizonRow : 0;
    const int lastRow = horizon == 1.0 ? horizonRow : baseRows;

    for (int y = firstRow; y <= lastRow; ++y)
    {
        // Quadratic spacing concentrates samples where the gradient changes fastest.
        double rowFraction;
    
        if (y <= horizonRow)
        {
            const double t = static_cast<double>(horizonRow - y) / horizonRow;
            rowFraction = horizon * (1.0 - t * t);
        }
        else
        {
            const double t = static_cast<double>(y - horizonRow) / (baseRows - horizonRow);
            rowFraction = horizon + (1.0 - horizon) * t * t;
        }

        sampleY.push_back(height * rowFraction);
    }

    auto sampleAround = [&](const SolarAureole& aureole, Vector3 direction)
    {
        if (solarAureole.Visible())
        {
            const Vector3 view = camera.WorldDirectionToView(direction);
            AddAureoleSamples(sampleX, std::atan2(view.x, view.z), width, focal);
            AddAureoleSamples(sampleY, std::atan2(-view.y, view.z), height, focal);
        }
    };

    sampleAround(solarAureole, sunDirection);
    sampleAround(lunarAureole, moonDirection);

    UpdateGrid();

    for (int y = 0; y <= rows; ++y)
    {
        const Vector3 left = topLeft + topToBottom * (sampleY[y] / height);

        for (int x = 0; x <= columns; ++x)
        {
            const Vector3 ray = Math::Normalize(left + leftToRight * (sampleX[x] / width));
            SDL_Vertex& vertex = vertices[y * (columns + 1) + x];
            vertex.position = {static_cast<float>(sampleX[x]), static_cast<float>(sampleY[y])};
            const SDL_FColor sunlitSky = solarAureole.Brighten(SampleSky(colors, horizontalSun, ray), ray);
            vertex.color = lunarAureole.Brighten(sunlitSky, ray);
        }
    }
}
