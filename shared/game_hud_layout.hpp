#pragma once

#include <SDL3/SDL.h>
#include <algorithm>

namespace GameHUD
{
    inline constexpr float margin = 18, dialRadius = 106, mapWidth = 360, mapHeight = 240;

    inline float Scale(const SDL_Rect& viewport)
    {
        // Reserve both corners, including in narrow/resized windows.
        return std::max(0.f, std::min({1.f, (viewport.w - 3 * margin) / (2 * dialRadius + mapWidth),
                                     (viewport.h - 56 - 2 * margin) / mapHeight}));
    }

    inline SDL_FRect MapRect(const SDL_Rect& viewport)
    {
        const float scale = Scale(viewport);
        return {viewport.w - margin - mapWidth * scale, viewport.h - margin - mapHeight * scale,
                mapWidth * scale, mapHeight * scale};
    }
}
