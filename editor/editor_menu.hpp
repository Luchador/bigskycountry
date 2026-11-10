#pragma once

#include <SDL3/SDL.h>
#include <optional>

enum class EditorCommand { Save, Undo, Redo, ToggleWireframe, CreateRoad, CreateTestTerrain };

class EditorMenu
{
public:
#ifdef _WIN32
    static bool OpenMaterialMenu(SDL_Window* window, int screenX, int screenY);
#endif
    bool Attach(SDL_Window* window);
    std::optional<EditorCommand> OpenViewport(SDL_FPoint windowPoint, SDL_FPoint viewportPoint, SDL_Rect scene);
#ifndef _WIN32
    static constexpr int barHeight = 24;
    bool HandleBar(SDL_Renderer* renderer, const SDL_Event& event, std::optional<EditorCommand>& command);
    bool DrawBar(SDL_Renderer* renderer, int width) const;
    bool HandleViewportMenu(SDL_Renderer* renderer, SDL_Rect scene, const SDL_Event& event, std::optional<EditorCommand>& command);
    bool DrawViewportMenu(SDL_Renderer* renderer) const;
#endif
    bool WireframeEnabled() const { return wireframe; }
    void ToggleWireframe();
    void SetHistoryEnabled(bool canUndo, bool canRedo);
    std::optional<EditorCommand> Command(const SDL_Event& event) const;

private:
    bool wireframe = false, undoEnabled = false, redoEnabled = false;
    SDL_WindowID windowID = 0;
#ifdef _WIN32
    Uint32 commandEvent = 0;
    void* editMenu = nullptr;
    void* viewMenu = nullptr;
#else
    int openMenu = -1, barRow = 0;
    bool Enabled(EditorCommand command) const;
    enum class Popup { Closed, Create, Submenu };
    Popup popup = Popup::Closed;
    SDL_FRect rootItem{}, submenu{};
    int activeItem = 0;
#endif
};
