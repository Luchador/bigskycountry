#include "sdl_project_settings.hpp"
#include "editor_theme.hpp"
#include "project_settings_field.hpp"

namespace
{
    constexpr SDL_FRect distanceBox{20, 69, 125, 23}, applyBox{155, 69, 64, 23};
}

bool SdlProjectSettings::Open(SDL_Window* owner)
{
    if (window)
    {
        if ((SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) && !SDL_RestoreWindow(window)) return false;
        return SDL_RaiseWindow(window);
    }

    constexpr int width = 600, height = 380;
    const auto props = SDL_CreateProperties();
    if (!props) return false;
    const bool configured =
        SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_PARENT_POINTER, owner) &&
        SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_UTILITY_BOOLEAN, true) &&
        SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_RESIZABLE_BOOLEAN, true) &&
        SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_HIGH_PIXEL_DENSITY_BOOLEAN, true) &&
        SDL_SetStringProperty(props, SDL_PROP_WINDOW_CREATE_TITLE_STRING, "Project Settings") &&
        SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, width) &&
        SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, height);
    int x, y, w, h;
    if (configured && SDL_GetWindowPosition(owner, &x, &y) && SDL_GetWindowSize(owner, &w, &h))
    {
        SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_X_NUMBER, x + (w - width) / 2);
        SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_Y_NUMBER, y + (h - height) / 2);
    }
    if (configured) window = SDL_CreateWindowWithProperties(props);
    SDL_DestroyProperties(props);
    if (!window) return false;
    renderer = SDL_CreateRenderer(window, nullptr);
    if (renderer && SDL_SetWindowMinimumSize(window, 288, 132) && Paint() && SDL_RaiseWindow(window)) return true;
    Close();
    return false;
}

void SdlProjectSettings::Close()
{
    distanceField.Blur(window);
    if (renderer) SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    window = nullptr;
    renderer = nullptr;
}

bool SdlProjectSettings::Paint() const
{
    const float density = SDL_GetWindowPixelDensity(window);
    return SDL_SetRenderScale(renderer, density, density) && EditorTheme::SetColor(renderer, EditorTheme::panel) &&
           SDL_RenderClear(renderer) && EditorTheme::SetColor(renderer, EditorTheme::text) &&
           SDL_RenderDebugText(renderer, 20, 16, "Optimization") &&
           SDL_RenderDebugText(renderer, 20, 48, "Rail and tie render distance (m)") &&
           distanceField.DrawField(renderer, distanceBox, {}, 0, RailroadDistanceField::Text(), true) &&
           EditorTheme::SetColor(renderer, EditorTheme::button) && SDL_RenderFillRect(renderer, &applyBox) &&
           EditorTheme::SetColor(renderer, EditorTheme::text) && SDL_RenderDebugText(renderer, 167, 76, "Apply") &&
           SDL_RenderPresent(renderer);
}

void SdlProjectSettings::Apply()
{
    if (!distanceField.Focused()) return;
    const std::string text = distanceField.EditedText();
    if (RailroadDistanceField::Apply(text))
        distanceField.Blur(window);
    else
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Project Settings", SDL_GetError(), window);
        distanceField.StartEdit(window, {}, 0, text);
    }
}

bool SdlProjectSettings::Handle(const SDL_Event& event)
{
    if (!window || SDL_GetWindowFromEvent(&event) != window) return false;
    if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED ||
        (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE && !event.key.repeat))
    {
        Close();
        return true;
    }

    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT)
    {
        const SDL_FPoint point{event.button.x, event.button.y};
        if (SDL_PointInRectFloat(&point, &distanceBox) && !distanceField.Focused())
            distanceField.StartEdit(window, {}, 0, RailroadDistanceField::Text());
        else if (SDL_PointInRectFloat(&point, &applyBox)) Apply();
    }
    else if (event.type == SDL_EVENT_KEY_DOWN && (event.key.key == SDLK_RETURN || event.key.key == SDLK_KP_ENTER))
    {
        if (!event.key.repeat) Apply();
    }
    else
    {
        std::optional<InspectorEdit> unused;
        if (!distanceField.Key(window, event, unused) && event.type != SDL_EVENT_WINDOW_EXPOSED &&
            event.type != SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED && event.type != SDL_EVENT_WINDOW_RESTORED)
            return true;
    }
    if (!Paint()) SDL_Log("Project Settings: %s", SDL_GetError());
    return true;
}
