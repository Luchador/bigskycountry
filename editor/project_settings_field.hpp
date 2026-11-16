#pragma once

#include "project_settings.hpp"
#include "transform_inspector.hpp"
#include <SDL3/SDL.h>
#include <charconv>

namespace ProjectDistanceFields
{
    inline std::string Text(bool viewDistance = false)
    {
        char text[64];
        const auto value = viewDistance ? g_ProjectSettings.ViewDistance() / 1000 : g_ProjectSettings.RailroadDetailDistance();
        const auto result = std::to_chars(text, text + sizeof(text), value);
        return {text, result.ptr};
    }

    inline bool Apply(std::string_view railText, std::string_view viewText)
    {
        const auto value = ParseTransformValue(railText);
        if (!value || *value < 0) return SDL_SetError("Enter a distance of zero or more meters.");
        const auto view = ParseTransformValue(viewText);
        if (!view) return SDL_SetError("Enter a view distance between 0.1 and 128 kilometers.");
        return g_ProjectSettings.SaveDistances(*value, *view * 1000);
    }
}
