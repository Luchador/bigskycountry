#include "world_lighting.hpp"

#include <algorithm>
#include <cmath>

float Srgb::ToLinear(float c)
{
    return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

float Srgb::ToDisplay(float c)
{
    c = std::clamp(c, 0.0f, 1.0f);
    return c <= 0.0031308f ? 12.92f * c : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
}

SDL_FColor Srgb::ToLinear(SDL_FColor c)
{
    return {ToLinear(c.r), ToLinear(c.g), ToLinear(c.b), c.a};
}

SDL_FColor WorldLighting::Shade(Vector3 normal, Vector3 view, SDL_FColor base, Vector3 orm, Vector3 emission) const
{
    return Srgb::ToDisplay(ShadeLinear(normal, view, base, orm, emission));
}

SDL_FColor Srgb::ToDisplay(SDL_FColor c)
{
    return {ToDisplay(c.r), ToDisplay(c.g), ToDisplay(c.b), c.a};
}

Vector3 SpotLight::Radiance(Vector3 surfaceToEye) const
{
    const Vector3 toLight = position + surfaceToEye;
    const double distanceSquared = Math::Dot(toLight, toLight);
    if (distanceSquared >= range * range || distanceSquared < 1e-8) return {};
    const double cosine = -Math::Dot(toLight, direction) / std::sqrt(distanceSquared);
    double cone = std::clamp((cosine - outerCos) / std::max(innerCos - outerCos, 1e-5), 0.0, 1.0);
    cone = cone * cone * (3 - 2 * cone);
    const double fade = 1 - distanceSquared / (range * range);
    return intensity * (cone * fade * fade / (1 + distanceSquared));
}

SDL_FColor WorldLighting::ShadeLinear(Vector3 normal, Vector3 view, SDL_FColor base, Vector3 orm, Vector3 emission) const
{
    const Vector3 surfaceToEye = view;
    const double gloss = 1.0 - orm.y;
    if (gloss > 0.0) view = Math::Normalize(view);
    Vector3 diffuse = ambient * orm.x;
    Vector3 specular{};
    const auto illuminate = [&](const DirectionalLight& light)
    {
        const double cosine = Math::Dot(normal, light.direction);
        if (cosine <= 0.0 || light.radiance == Vector3{}) return;
        diffuse = diffuse + light.radiance * ((1.0 - orm.z) * cosine);
        if (gloss > 0.0)
        {
            const Vector3 halfway = Math::Normalize(view + light.direction);
            const double highlight = std::pow(std::clamp(Math::Dot(normal, halfway), 0.0, 1.0), 4 + 124 * gloss * gloss);
            specular = specular + light.radiance * (gloss * cosine * highlight);
        }
    };
    illuminate(sun);
    illuminate(moon);
    for (const auto& spot : spots)
    {
        const auto radiance = spot.Radiance(surfaceToEye);
        if (radiance != Vector3{}) illuminate({Math::Normalize(spot.position + surfaceToEye), radiance});
    }
    const auto channel = [&](float albedo, double d, double s, double e)
    {
        return static_cast<float>(e + albedo * d + std::lerp(0.08, double(albedo), orm.z) * s);
    };
    return {channel(base.r, diffuse.x, specular.x, emission.x), channel(base.g, diffuse.y, specular.y, emission.y),
            channel(base.b, diffuse.z, specular.z, emission.z), base.a};
}
