#include "solar_aureole.hpp"
#include "world_lighting.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double asymmetry = 0.75;  // Higher values concentrate the glow closer to the disc.
    constexpr double peakStrength = 0.45;

    double SmoothStep(double lo, double hi, double x)
    {
        const double t = std::clamp((x - lo) / (hi - lo), 0.0, 1.0);

        return t * t * (3.0 - 2.0 * t);
    }

    // Henyey-Greenstein divided by its value at zero angular separation.
    double RelativePhase(double cosine)
    {
        constexpr double peak = (1.0 - asymmetry) * (1.0 - asymmetry);
        const double ratio = peak / (peak + 2.0 * asymmetry * (1.0 - std::clamp(cosine, -1.0, 1.0)));

        return ratio * std::sqrt(ratio);
    }

    float AddScatteredLight(float base, float linearLight, float weight)
    {
        const float linearBase = Srgb::ToLinear(base);

        if (linearLight <= linearBase)
        {
            return base;
        }

        // Use the available brightness headroom in the game's SDR sky palette.
        return Srgb::ToDisplay(std::lerp(linearBase, linearLight, weight));
    }
}

// Constructor
SolarAureole::SolarAureole(Vector3 lightDirection, SDL_FColor lightTint, double relativeStrength)
{
    direction = lightDirection;

    linearTint = Srgb::ToLinear(lightTint);

    strength = peakStrength * relativeStrength * SmoothStep(-0.10, 0.04, direction.y);
}

SDL_FColor SolarAureole::Brighten(SDL_FColor sky, Vector3 unitRay) const
{
    // Continue smoothly into the existing haze just below the horizon.
    const double visibility = strength * SmoothStep(-0.07, 0.0, unitRay.y);

    if (visibility == 0.0)
    {
        return sky;
    }

    const float weight = static_cast<float>(visibility * RelativePhase(Math::Dot(unitRay, direction)));

    return {AddScatteredLight(sky.r, linearTint.r, weight), AddScatteredLight(sky.g, linearTint.g, weight), AddScatteredLight(sky.b, linearTint.b, weight), sky.a};
}
