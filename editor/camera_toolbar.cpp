#include "camera_toolbar.hpp"
#include "free_camera.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr float margin = 12.0f;
    constexpr float valueWidth = 88.0f;
    constexpr float buttonSize = 32.0f;
    constexpr int minWidthForValue = 162;

    SDL_FRect Track(int width)
    {
        const float right = width - margin - (width >= minWidthForValue ? valueWidth : 0.0f);
        const float left = std::max(margin + buttonSize + 8, right - 176.0f);
        return {left, CameraToolbar::height * 0.5f - 2.0f, std::max(1.0f, right - left), 4.0f};
    }

    bool ShowSpeedLabel(SDL_FRect track)
    {
        return track.x >= margin + buttonSize + 112.0f;
    }

    SDL_FRect SpaceButton(SDL_FRect track)
    {
        const float gap = ShowSpeedLabel(track) ? 112.0f : 8.0f;
        return {track.x - gap - buttonSize, (CameraToolbar::height - buttonSize) / 2, buttonSize, buttonSize};
    }

    bool Fill(SDL_Renderer* renderer, SDL_FRect rect, SDL_Color color)
    {
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a) && SDL_RenderFillRect(renderer, &rect);
    }
}

void CameraToolbar::LoadIcons(SDL_Renderer* renderer)
{
    for (GizmoSpace space : {GizmoSpace::World, GizmoSpace::Local})
    {
        SDL_Surface* surface = LoadTransformSpaceIcon(space);
        if (surface)
        {
            icons[static_cast<int>(space)] = SDL_CreateTextureFromSurface(renderer, surface);
            SDL_DestroySurface(surface);
        }
    }
}

bool CameraToolbar::Draw(SDL_Renderer* renderer, int width, const FreeCamera& camera, GizmoSpace space) const
{
    const float w = static_cast<float>(width);
    const SDL_FRect track = Track(width);
    const float fill = track.w * static_cast<float>(camera.GetSpeedFraction());
    const float thumb = track.x + fill;

    if (!Fill(renderer, {0, 0, w, height}, {31, 40, 47, 255}) ||
        !Fill(renderer, {0, height - 1, w, 1}, {60, 74, 80, 255}) ||
        !Fill(renderer, track, {76, 90, 97, 255}) ||
        !Fill(renderer, {track.x, track.y, fill, track.h}, {76, 158, 158, 255}) ||
        !Fill(renderer, {thumb - 4, 9, 8, 18}, dragging ? SDL_Color{164, 233, 227, 255} : SDL_Color{215, 229, 231, 255}))
    {
        return false;
    }

    if (!SDL_SetRenderDrawColor(renderer, 221, 228, 234, 255))
    {
        return false;
    }
    const SDL_FRect button = SpaceButton(track);
    SDL_Texture* icon = icons[static_cast<int>(space)];
    if (icon)
    {
        const SDL_FRect image = {button.x + spacePressed, button.y + spacePressed, button.w, button.h};
        if (!SDL_RenderTexture(renderer, icon, nullptr, &image))
        {
            return false;
        }
    }
    else if (!SDL_RenderRect(renderer, &button) ||
             !SDL_RenderDebugText(renderer, button.x + 12, button.y + 12, space == GizmoSpace::World ? "W" : "L"))
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

bool CameraToolbar::HandlePointer(const SDL_Event& event, int width, FreeCamera& camera, GizmoSpace& space)
{
    const SDL_FRect track = Track(width);
    const auto onSpaceButton = [&](const SDL_MouseButtonEvent& pointer)
    {
        const SDL_FPoint pos{pointer.x, pointer.y + height};
        const SDL_FRect button = SpaceButton(track);
        return SDL_PointInRectFloat(&pos, &button);
    };
    const auto setSpeed = [&](float x)
    {
        camera.SetSpeedFraction((x - grabOffset - track.x) / track.w);
    };

    switch (event.type)
    {
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        if (event.button.button == SDL_BUTTON_LEFT && onSpaceButton(event.button))
        {
            spacePressed = true;
            return true;
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
        return dragging || spacePressed || event.motion.y < 0;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if (spacePressed && event.button.button == SDL_BUTTON_LEFT)
        {
            spacePressed = false;
            if (onSpaceButton(event.button))
            {
                space = OtherSpace(space);
            }
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
