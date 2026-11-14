#pragma once

#include "project_settings.hpp"
#include "transform_inspector.hpp"
#include <SDL3/SDL.h>
#include <charconv>

namespace RailroadDistanceField
{
    inline std::string Text()
    {
        char text[64];
        const auto result = std::to_chars(text, text + sizeof(text), g_ProjectSettings.RailroadDetailDistance());
        return {text, result.ptr};
    }

    inline bool Apply(std::string_view text)
    {
        const auto value = ParseTransformValue(text);
        if (!value || *value < 0) return SDL_SetError("Enter a distance of zero or more meters.");
        return g_ProjectSettings.SaveRailroadDetailDistance(*value);
    }
}
