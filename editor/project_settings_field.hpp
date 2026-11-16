#pragma once

#include "project_settings.hpp"
#include "transform_inspector.hpp"
#include <SDL3/SDL.h>
#include <charconv>

namespace ProjectDistanceFields
{
    inline std::string Text(int field = 0)
    {
        char text[64];
        const auto& budgets = g_ProjectSettings.Streaming();
        const double values[]{g_ProjectSettings.RailroadDetailDistance(), g_ProjectSettings.ViewDistance() / 1000,
            double(budgets.geometryBytes) / StreamingSettings::MiB, double(budgets.assetBytes) / StreamingSettings::MiB,
            double(budgets.collisionBytes) / StreamingSettings::MiB};
        const auto value = values[field];
        const auto result = std::to_chars(text, text + sizeof(text), value);
        return {text, result.ptr};
    }

    inline bool Apply(std::string_view railText, std::string_view viewText,
                      std::string_view geometryText, std::string_view assetsText, std::string_view collisionText)
    {
        const auto value = ParseTransformValue(railText);
        if (!value || *value < 0) return SDL_SetError("Enter a distance of zero or more meters.");
        const auto view = ParseTransformValue(viewText);
        if (!view) return SDL_SetError("Enter a view distance between 0.1 and 128 kilometers.");
        auto budgets = g_ProjectSettings.Streaming();
        const auto parseBudget = [](std::string_view text, std::size_t& bytes) {
            const auto parsed = ParseTransformValue(text);
            if (!parsed || *parsed < 64 || *parsed > 16384) return false;
            bytes = static_cast<std::size_t>(*parsed * StreamingSettings::MiB);
            return true;
        };
        if (!parseBudget(geometryText, budgets.geometryBytes) || !parseBudget(assetsText, budgets.assetBytes) ||
            !parseBudget(collisionText, budgets.collisionBytes)) return SDL_SetError("Streaming budgets must be between 64 and 16384 MiB.");
        return g_ProjectSettings.SaveOptions(*value, *view * 1000, budgets);
    }
}
