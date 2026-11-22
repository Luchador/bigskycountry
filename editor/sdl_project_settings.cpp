#include "sdl_project_settings.hpp"
#include "editor_theme.hpp"
#include "project_settings_field.hpp"

namespace
{
    constexpr SDL_FRect distanceBoxes[]{{20, 69, 125, 23}, {20, 125, 125, 23}, {20, 235, 125, 23}, {20, 291, 125, 23}, {20, 347, 125, 23}, {20, 411, 125, 23}};
    constexpr SDL_FRect applyBox{20, 510, 80, 23};
    constexpr SDL_FRect presetBoxes[]{{155, 125, 64, 23}, {229, 125, 64, 23}, {303, 125, 64, 23}};
    constexpr const char* presetValues[]{"4", "8", "16"};
}

bool SdlProjectSettings::Open(SDL_Window* owner)
{
    if (window)
    {
        if ((SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) && !SDL_RestoreWindow(window)) return false;
        return SDL_RaiseWindow(window);
    }

    values = {ProjectDistanceFields::Text(0), ProjectDistanceFields::Text(1), ProjectDistanceFields::Text(2), ProjectDistanceFields::Text(3), ProjectDistanceFields::Text(4), ProjectDistanceFields::Text(5)};
    constexpr int width = 600, height = 570;
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
    if (renderer && SDL_SetWindowMinimumSize(window, 600, 560) && Paint() && SDL_RaiseWindow(window)) return true;
    Close();
    return false;
}

void SdlProjectSettings::Close()
{
    distanceField.Blur(window);
    activeField = -1;
    if (renderer) SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    window = nullptr;
    renderer = nullptr;
}

bool SdlProjectSettings::Paint() const
{
    const float density = SDL_GetWindowPixelDensity(window);
    if (!SDL_SetRenderScale(renderer, density, density) || !EditorTheme::SetColor(renderer, EditorTheme::panel) ||
        !SDL_RenderClear(renderer) || !EditorTheme::SetColor(renderer, EditorTheme::text) ||
        !SDL_RenderDebugText(renderer, 20, 16, "Optimization") ||
        !SDL_RenderDebugText(renderer, 20, 48, "Rail and tie render distance (m)") ||
        !SDL_RenderDebugText(renderer, 20, 104, "View distance (km)") ||
        !SDL_RenderDebugText(renderer, 20, 169, "3D visibility radius in game kilometers (0.1 - 128).") ||
        !SDL_RenderDebugText(renderer, 20, 189, "Editor and game. F3 toggles visibility statistics.") ||
        !SDL_RenderDebugText(renderer, 20, 214, "Geometry budget per CPU/GPU cache (MiB)") ||
        !SDL_RenderDebugText(renderer, 20, 270, "Shared model + texture budget (MiB)") ||
        !SDL_RenderDebugText(renderer, 20, 326, "Streamed collision budget (MiB)") ||
        !SDL_RenderDebugText(renderer, 20, 390, "Shadow distance (km, 0 - 10; 0 disables shadows)") ||
        !SDL_RenderDebugText(renderer, 20, 460, "Nearby collision and active GPU assets stay protected.") ||
        !SDL_RenderDebugText(renderer, 20, 480, "Editor source data and undo are always kept resident.")) return false;
    for (int i = 0; i < 6; ++i)
        if (!distanceField.DrawField(renderer, distanceBoxes[i], {}, i, values[i], true)) return false;
    for (int i = 0; i < 3; ++i)
        if (!EditorTheme::SetColor(renderer, EditorTheme::button) || !SDL_RenderFillRect(renderer, &presetBoxes[i]) ||
            !EditorTheme::SetColor(renderer, EditorTheme::text) ||
            !SDL_RenderDebugTextFormat(renderer, presetBoxes[i].x + 10, presetBoxes[i].y + 7, "%s km", presetValues[i])) return false;
    return EditorTheme::SetColor(renderer, EditorTheme::button) && SDL_RenderFillRect(renderer, &applyBox) &&
           EditorTheme::SetColor(renderer, EditorTheme::text) && SDL_RenderDebugText(renderer, 40, 517, "Apply") &&
           SDL_RenderPresent(renderer);
}

void SdlProjectSettings::StoreField()
{
    if (activeField >= 0 && distanceField.Focused()) values[activeField] = distanceField.EditedText();
    distanceField.Blur(window);
    activeField = -1;
}

void SdlProjectSettings::Apply()
{
    StoreField();
    if (ProjectDistanceFields::Apply(values[0], values[1], values[2], values[3], values[4], values[5]))
        values = {ProjectDistanceFields::Text(0), ProjectDistanceFields::Text(1), ProjectDistanceFields::Text(2), ProjectDistanceFields::Text(3), ProjectDistanceFields::Text(4), ProjectDistanceFields::Text(5)};
    else
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Project Settings", SDL_GetError(), window);
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
        StoreField();
        for (int i = 0; i < 6; ++i)
            if (SDL_PointInRectFloat(&point, &distanceBoxes[i]))
            {
                activeField = i;
                distanceField.StartEdit(window, {}, i, values[i]);
            }
        for (int i = 0; i < 3; ++i)
            if (SDL_PointInRectFloat(&point, &presetBoxes[i])) values[1] = presetValues[i];
        if (SDL_PointInRectFloat(&point, &applyBox)) Apply();
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
