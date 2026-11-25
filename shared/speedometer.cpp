#include "speedometer.hpp"
#include "game_hud_layout.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    constexpr float radius = GameHUD::dialRadius;
    constexpr double maximumKmh = 240;
    constexpr float startAngle = 0.75f * std::numbers::pi_v<float>;
    constexpr float sweepAngle = 1.5f * std::numbers::pi_v<float>;

    SDL_FPoint Polar(SDL_FPoint center, float distance, float angle)
    {
        return {center.x + distance * std::cos(angle), center.y + distance * std::sin(angle)};
    }

    // Seven-segment digits stay crisp at any HUD size without a font texture.
    bool Digit(SDL_Renderer* renderer, char digit, float x, float y, float scale)
    {
        constexpr unsigned segments[]{0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
        constexpr SDL_FRect bars[]{
            {4,0,16,3}, {21,4,3,12}, {21,20,3,12}, {4,33,16,3},
            {0,20,3,12}, {0,4,3,12}, {4,16.5f,16,3}};
        const auto mask = segments[digit - '0'];
        for (std::size_t i = 0; i < std::size(bars); ++i)
        {
            if (!(mask & (1u << i))) continue;
            const auto& bar = bars[i];
            const SDL_FRect rect{x + bar.x * scale, y + bar.y * scale, bar.w * scale, bar.h * scale};
            if (!SDL_RenderFillRect(renderer, &rect)) return false;
        }
        return true;
    }

    bool DrawDial(SDL_Renderer* renderer, const SDL_Rect& viewport, double speed)
    {
        const float scale = GameHUD::Scale(viewport);
        if (scale < 0.5f) return true;
        const SDL_FPoint center{GameHUD::margin + radius * scale, viewport.h - GameHUD::margin - radius * scale};
        constexpr int sides = 96;
        std::array<SDL_Vertex, sides + 1> vertices{};
        std::array<int, sides * 3> indices{};
        std::array<SDL_FPoint, sides + 1> outline{};
        vertices[0] = {center, {0.055f, 0.065f, 0.08f, 0.88f}, {}};
        for (int i = 0; i < sides; ++i)
        {
            outline[i] = Polar(center, radius * scale, i * 2 * std::numbers::pi_v<float> / sides);
            vertices[i + 1] = {outline[i], vertices[0].color, {}};
            indices[i * 3] = 0;
            indices[i * 3 + 1] = i + 1;
            indices[i * 3 + 2] = (i + 1) % sides + 1;
        }
        outline.back() = outline.front();
        if (!SDL_RenderGeometry(renderer, nullptr, vertices.data(), int(vertices.size()), indices.data(), int(indices.size())) ||
            !SDL_SetRenderDrawColor(renderer, 101,111,126,230) ||
            !SDL_RenderLines(renderer, outline.data(), int(outline.size()))) return false;

        for (int kmh = 0; kmh <= int(maximumKmh); kmh += 10)
        {
            const float angle = startAngle + sweepAngle * float(kmh / maximumKmh);
            const bool major = kmh % 40 == 0;
            const auto outer = Polar(center, 96 * scale, angle);
            const auto inner = Polar(center, (major ? 83 : 89) * scale, angle);
            if (!SDL_SetRenderDrawColor(renderer, major ? 230 : 124, major ? 233 : 136, major ? 239 : 152, 255) ||
                !SDL_RenderLine(renderer, inner.x, inner.y, outer.x, outer.y)) return false;
            if (major && scale >= 0.8f)
            {
                char label[8];
                const int length = SDL_snprintf(label, sizeof(label), "%d", kmh);
                const auto at = Polar(center, 69 * scale, angle);
                if (!SDL_RenderDebugText(renderer, at.x - length * 4, at.y - 4, label)) return false;
            }
        }

        const float angle = startAngle + sweepAngle * float(std::clamp(speed / maximumKmh, 0.0, 1.0));
        const auto left = Polar(center, 3 * scale, angle - std::numbers::pi_v<float> / 2);
        const auto right = Polar(center, 3 * scale, angle + std::numbers::pi_v<float> / 2);
        const auto tip = Polar(center, 79 * scale, angle);
        constexpr SDL_FColor amber{1, 0.66f, 0.16f, 1};
        const SDL_Vertex needle[]{{left, amber, {}}, {tip, amber, {}}, {right, amber, {}}};
        const SDL_FRect hub{center.x - 4 * scale, center.y - 4 * scale, 8 * scale, 8 * scale};
        if (!SDL_RenderGeometry(renderer, nullptr, needle, 3, nullptr, 0) ||
            !SDL_SetRenderDrawColor(renderer, 255,168,41,255) || !SDL_RenderFillRect(renderer, &hub)) return false;

        char value[32];
        const int digits = SDL_snprintf(value, sizeof(value), "%.0f", speed);
        // Shrink long readouts rather than clipping them to the dial's range.
        const float digitScale = scale * std::min(0.8f, 2.4f / digits);
        const float width = (digits * 30 - 6) * digitScale;
        if (!SDL_SetRenderDrawColor(renderer, 244,246,250,255)) return false;
        for (int i = 0; i < digits; ++i)
            if (!Digit(renderer, value[i], center.x - width / 2 + i * 30 * digitScale,
                       center.y + 54 * scale, digitScale)) return false;
        return SDL_SetRenderDrawColor(renderer, 180,189,202,255) &&
               SDL_RenderDebugText(renderer, center.x - 16, center.y + 86 * scale, "km/h");
    }
}

bool DrawSpeedometer(SDL_Renderer* renderer, double metersPerSecond)
{
    SDL_Rect viewport{};
    SDL_BlendMode blend;
    if (!SDL_GetRenderViewport(renderer, &viewport) || !SDL_GetRenderDrawBlendMode(renderer, &blend)) return false;
    // The same positive readout applies in reverse. Reject invalid physics values.
    const double kmh = std::abs(metersPerSecond) * 3.6;
    const double speed = std::isfinite(kmh) ? std::min(kmh, 999999.0) : 0.0;
    const bool drawn = SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND) && DrawDial(renderer, viewport, speed);
    const bool restored = SDL_SetRenderDrawBlendMode(renderer, blend);
    return drawn && restored;
}
