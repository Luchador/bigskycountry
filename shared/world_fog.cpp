#include "world_fog.hpp"

#include <algorithm>
#include <cmath>

double WorldFog::Transmittance(Vector3 ray, double eyeHeight) const
{
    const double distance = Math::Length(ray);
    if (distance <= startDistance) return 1.0;
    const double length = distance - startDistance;
    const double y0 = eyeHeight - baseHeight + ray.y * (startDistance / distance);
    const double y1 = eyeHeight - baseHeight + ray.y;
    // Integrate exp(-height * falloff) along the ray. Using the lower endpoint
    // avoids overflow/cancellation on downward rays from high-altitude cameras.
    const double span = heightFalloff * std::abs(y1 - y0);
    const double average = span < 0.001 ? 1.0 - span * 0.5 + span * span / 6.0 : -std::expm1(-span) / span;
    const double lowerDensity = density * std::exp(std::clamp(-heightFalloff * std::min(y0, y1), -80.0, 8.0));
    const double opticalDepth = length * (airDensity + lowerDensity * average);
    return std::exp(-std::clamp(opticalDepth, 0.0, 80.0));
}

SDL_FColor WorldFog::Apply(SDL_FColor surface, Vector3 view, double eyeHeight) const
{
    const double transmission = Transmittance(view * -1.0, eyeHeight);
    if (transmission == 1.0) return surface;
    const Vector3 ray = Math::Normalize(view * -1.0);
    const auto blend = [](SDL_FColor a, SDL_FColor b, double t) {
        const float f = static_cast<float>(t);
        return SDL_FColor{std::lerp(a.r,b.r,f), std::lerp(a.g,b.g,f), std::lerp(a.b,b.b,f), 1};
    };
    // Same directional gradient as Sky::SampleSky: warm toward the sun at dusk,
    // dark at night, and fading toward the zenith when looking up.
    const double facing = std::clamp(0.5 + 0.5 * Math::Dot(ray, horizontalSun), 0.0, 1.0);
    auto color = blend(oppositeHorizon, sunwardHorizon, facing * facing);
    if (ray.y >= 0.0) color = blend(color, zenith, std::sqrt(ray.y));
    else
    {
        const double depth = std::min(-ray.y * 2.0, 1.0);
        color = blend(color, blend(oppositeHorizon, zenith, 0.55), depth * (2.0 - depth));
    }
    auto result = blend(Srgb::ToLinear(color), surface, transmission);
    result.a = surface.a; // Fog changes radiance, never the surface's coverage.
    return result;
}
