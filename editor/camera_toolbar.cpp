#include "camera_toolbar.hpp"
#include "editor_theme.hpp"
#include "free_camera.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr float margin = 12.0f;
    constexpr float valueWidth = 88.0f;
    constexpr float buttonSize = 32.0f;
    constexpr float buttonGap = 4.0f;
    constexpr float stepLabelWidth = 52.0f;
    constexpr float buttonsWidth = 2 * buttonSize + buttonGap + stepLabelWidth;
    constexpr int minWidthForValue = 246;

    SDL_FRect Track(int width)
    {
        const float right = width - margin - (width >= minWidthForValue ? valueWidth : 0.0f);
        const float left = std::max(margin + buttonsWidth + 8, right - 176.0f);
        return {left, CameraToolbar::height * 0.5f - 2.0f, std::max(1.0f, right - left), 4.0f};
    }

    bool ShowSpeedLabel(SDL_FRect track)
    {
        return track.x >= margin + buttonsWidth + 112.0f;
    }

    SDL_FRect SpaceButton(SDL_FRect track)
    {
        const float gap = ShowSpeedLabel(track) ? 112.0f : 8.0f;
        return {track.x - gap - buttonSize, (CameraToolbar::height - buttonSize) / 2, buttonSize, buttonSize};
    }

    SDL_FRect SnapButton(SDL_FRect space)
    {
        space.x -= buttonSize + buttonGap;
        return space;
    }

    bool Fill(SDL_Renderer* renderer, SDL_FRect rect, SDL_Color color)
    {
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a) && SDL_RenderFillRect(renderer, &rect);
    }
}

void CameraToolbar::LoadIcons(SDL_Renderer* renderer)
{
    for (TransformIcon icon : transformIcons)
    {
        SDL_Surface* surface = LoadTransformIcon(icon);
        if (surface)
        {
            icons[static_cast<int>(icon)] = SDL_CreateTextureFromSurface(renderer, surface);
            SDL_DestroySurface(surface);
        }
    }
}

bool CameraToolbar::Draw(SDL_Renderer* renderer, int width, const FreeCamera& camera, const TransformSettings& settings) const
{
    const float w = static_cast<float>(width);
    const SDL_FRect track = Track(width);
    const float fill = track.w * static_cast<float>(camera.GetSpeedFraction());
    const float thumb = track.x + fill;

    if (!Fill(renderer, {0, 0, w, height}, EditorTheme::chrome) ||
        !Fill(renderer, {0, height - 1, w, 1}, EditorTheme::border) ||
        !Fill(renderer, track, EditorTheme::button) ||
        !Fill(renderer, {track.x, track.y, fill, track.h}, EditorTheme::accent) ||
        !Fill(renderer, {thumb - 4, 9, 8, 18}, dragging ? EditorTheme::accent : EditorTheme::text))
    {
        return false;
    }

    if (!EditorTheme::SetColor(renderer, EditorTheme::text))
    {
        return false;
    }
    const auto drawButton = [&](SDL_FRect rect, Button button, TransformIcon icon, const char* fallback)
    {
        if (auto* texture = icons[static_cast<int>(icon)])
        {
            const float offset = pressed == button ? 1.0f : 0.0f;
            rect.x += offset;
            rect.y += offset;
            return SDL_RenderTexture(renderer, texture, nullptr, &rect);
        }
        return SDL_RenderRect(renderer, &rect) &&
               SDL_RenderDebugText(renderer, rect.x + 4, rect.y + 12, fallback);
    };
    const SDL_FRect space = SpaceButton(track), snap = SnapButton(space);
    if (!drawButton(space, Button::Space, settings.SpaceIcon(), settings.space == GizmoSpace::World ? "W" : "L") ||
        !drawButton(snap, Button::Snap, settings.SnapIcon(), settings.snapEnabled ? "On" : "Off") ||
        !SDL_RenderDebugText(renderer, snap.x - stepLabelWidth, 14, settings.StepLabel()))
    {
        return false;
    }
    if (ShowSpeedLabel(track) && !SDL_RenderDebugText(renderer, track.x - 110, 14, "Camera speed"))
    {
        return false;
    }
    if (width >= minWidthForValue)
    {
        const double speed = camera.GetMoveSpeed();
        const float x = track.x + track.w + margin;
        if (speed >= 1000.0)
        {
            return SDL_RenderDebugTextFormat(renderer, x, 14, "%.2f km/s", speed / 1000.0);
        }
        return SDL_RenderDebugTextFormat(renderer, x, 14, speed < 10.0 ? "%.2f m/s" : "%.1f m/s", speed);
    }
    return true;
}

bool CameraToolbar::HandlePointer(const SDL_Event& event, int width, FreeCamera& camera, TransformSettings& settings)
{
    const SDL_FRect track = Track(width);
    const auto buttonAt = [&](const SDL_MouseButtonEvent& pointer)
    {
        const SDL_FPoint pos{pointer.x, pointer.y + height};
        const SDL_FRect button = SpaceButton(track);
        const SDL_FRect snap = SnapButton(button);
        if (SDL_PointInRectFloat(&pos, &snap))
        {
            return Button::Snap;
        }
        if (SDL_PointInRectFloat(&pos, &button))
        {
            return Button::Space;
        }
        return Button::None;
    };
    const auto setSpeed = [&](float x)
    {
        camera.SetSpeedFraction((x - grabOffset - track.x) / track.w);
    };

    switch (event.type)
    {
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        if (event.button.button == SDL_BUTTON_LEFT)
        {
            pressed = buttonAt(event.button);
            if (pressed != Button::None)
            {
                return true;
            }
        }
        if (event.button.y < 0)
        {
            if (event.button.button == SDL_BUTTON_LEFT &&
                event.button.x >= track.x - 5 && event.button.x <= track.x + track.w + 5)
            {
                dragging = true;
                const float thumb = track.x + track.w * static_cast<float>(camera.GetSpeedFraction());
                const float offset = event.button.x - thumb;
                grabOffset = std::abs(offset) <= 4 ? offset : 0.0f;
                setSpeed(event.button.x);
            }
            return true;
        }
        break;
    case SDL_EVENT_MOUSE_MOTION:
        if (dragging)
        {
            setSpeed(event.motion.x);
        }
        return dragging || pressed != Button::None || event.motion.y < 0;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (pressed != Button::None && event.button.button == SDL_BUTTON_LEFT)
        {
            if (buttonAt(event.button) == pressed)
            {
                if (pressed == Button::Snap)
                {
                    settings.snapEnabled = !settings.snapEnabled;
                }
                else
                {
                    settings.ToggleSpace();
                }
            }
            pressed = Button::None;
            return true;
        }
        if (dragging && event.button.button == SDL_BUTTON_LEFT)
        {
            setSpeed(event.button.x);
            dragging = false;
            return true;
        }
        return dragging || event.button.y < 0;
    }
    return dragging;
}
