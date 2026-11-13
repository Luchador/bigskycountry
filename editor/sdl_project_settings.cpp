#include "sdl_project_settings.hpp"
#include "editor_theme.hpp"

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
    if (Paint() && SDL_RaiseWindow(window)) return true;
    SDL_DestroyWindow(window);
    window = nullptr;
    return false;
}

bool SdlProjectSettings::Paint() const
{
    SDL_Surface* surface = SDL_GetWindowSurface(window);
    const auto color = EditorTheme::panel;
    return surface && SDL_FillSurfaceRect(surface, nullptr, SDL_MapSurfaceRGB(surface, color.r, color.g, color.b)) &&
           SDL_UpdateWindowSurface(window);
}

bool SdlProjectSettings::Handle(const SDL_Event& event)
{
    if (!window || SDL_GetWindowFromEvent(&event) != window) return false;
    if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED ||
        (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE && !event.key.repeat))
    {
        SDL_DestroyWindow(window);
        window = nullptr;
    }
    else if (event.type == SDL_EVENT_WINDOW_EXPOSED || event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED ||
             event.type == SDL_EVENT_WINDOW_RESTORED)
    {
        if (!Paint()) SDL_Log("Project Settings: %s", SDL_GetError());
    }
    return true;
}
