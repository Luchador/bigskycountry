#include "asset_paths.hpp"
#include "transform_settings.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <string>

namespace
{
    struct SnapStep
    {
        double meters;
        const char* label;
    };
    constexpr SnapStep steps[] = {{0.01, "1 cm"}, {0.1, "10 cm"}, {1, "1 m"},
                                  {10, "10 m"}, {100, "100 m"}, {1000, "1 km"}};
}

void TransformSettings::ChangeStep(int direction)
{
    step = std::clamp(step + direction, 0, static_cast<int>(std::size(steps)) - 1);
}

double TransformSettings::StepMeters() const { return steps[step].meters; }
const char* TransformSettings::StepLabel() const { return steps[step].label; }

double TransformSettings::SnapDistance(double meters) const
{
    return snapEnabled ? std::round(meters / StepMeters()) * StepMeters() : meters;
}

Vector3 TransformSettings::SnapPlanarOffset(Vector3 offset, std::optional<double> heading) const
{
    if (heading)
    {
        const double distance = SnapDistance(std::hypot(offset.x, offset.z));
        return {std::cos(*heading) * distance, 0, std::sin(*heading) * distance};
    }
    return {SnapDistance(offset.x), 0, SnapDistance(offset.z)};
}

SDL_Surface* LoadTransformIcon(TransformIcon icon)
{
    constexpr std::array names{"btn_transform_global.png", "btn_transform_local.png",
                               "btn_snap_to_grid_off.png", "btn_snap_to_grid_on.png"};
    const char* name = names[static_cast<size_t>(icon)];
    const std::string path = AssetPaths::Root(AssetRoot::Project) + "textures/editor/" + name;
    SDL_Surface* surface = SDL_LoadPNG(path.c_str());
    if (!surface)
    {
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Toolbar icon %s: %s", path.c_str(), SDL_GetError());
    }
    return surface;
}
