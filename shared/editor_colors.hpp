#pragma once

#include <SDL3/SDL_pixels.h>

namespace EditorColors
{
    inline constexpr SDL_FColor wire{0.90f, 0.92f, 0.95f, 1.0f}; // World triangle edges in wireframe view.
    inline constexpr SDL_FColor blue{0.39f, 0.90f, 0.88f, 1.0f};  // Editable points and segments.
    inline constexpr SDL_FColor orange{1.0f, 0.5f, 0.05f, 1.0f};   // Selected object outline.
    inline constexpr SDL_FColor amber{1.0f, 0.76f, 0.32f, 1.0f};   // Selected points, segments and handles.

    constexpr SDL_Color ToBytes(SDL_FColor color)
    {
        const auto channel = [](float value) { return static_cast<Uint8>(value * 255.0f + 0.5f); };
        return {channel(color.r), channel(color.g), channel(color.b), channel(color.a)};
    }
}
