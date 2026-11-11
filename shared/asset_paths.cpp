#include "asset_paths.hpp"
#include <SDL3/SDL.h>

const std::string& AssetPaths::Root(AssetRoot location)
{
    if (location == AssetRoot::Project)
    {
        constexpr auto path = BIGSKY_PROJECT_ASSETS;
        static const std::string project(reinterpret_cast<const char*>(path));
        return project;
    }
    static const std::string cooked = []
    {
        const char* base = SDL_GetBasePath();
        if (!base) return std::string{};
        // Shipped builds keep cooked/ beside the executable. Multi-config builds
        // put executables in a configuration subfolder and share one cooked tree.
        const std::string adjacent = std::string(base) + "cooked/";
        SDL_PathInfo info{};
        if (SDL_GetPathInfo(adjacent.c_str(), &info) && info.type == SDL_PATHTYPE_DIRECTORY)
            return adjacent;
        constexpr auto relative = BIGSKY_COOKED_RELATIVE;
        return std::string(base) + reinterpret_cast<const char*>(relative);
    }();
    return cooked;
}
