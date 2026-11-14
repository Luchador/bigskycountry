#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>
#include <array>

class CameraView;

namespace SkyDisc
{
    struct DiscMesh
    {
        std::array<SDL_Vertex, 8> vertices{};
        int count = 0;
        static constexpr int fan[] = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 7};
    };

    // Project a textured disc at infinity, clipped to the horizon and viewport.
    // The visible disc bounds set the angular diameter independently of transparent padding.
    DiscMesh Project(const CameraView& camera, Vector3 direction, SDL_Rect viewport, const SDL_Texture& texture, SDL_Rect discBounds, double angularDiameter, SDL_FColor color);
}
