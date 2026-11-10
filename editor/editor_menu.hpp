#pragma once

#include <SDL3/SDL.h>
#include <optional>

enum class EditorCommand { Save, Undo, Redo, CreateRoad, CreateTestTerrain };

class EditorMenu
{
public:
    bool Attach(SDL_Window* window);
    std::optional<EditorCommand> OpenViewport(SDL_FPoint windowPoint, SDL_FPoint viewportPoint, SDL_Rect scene);
#ifndef _WIN32
    bool HandleViewportMenu(SDL_Renderer* renderer, SDL_Rect scene, const SDL_Event& event, std::optional<EditorCommand>& command);
    bool DrawViewportMenu(SDL_Renderer* renderer) const;
#endif
    void SetHistoryEnabled(bool canUndo, bool canRedo);
    std::optional<EditorCommand> Command(const SDL_Event& event) const;

private:
    SDL_WindowID windowID = 0;
#ifdef _WIN32
    Uint32 commandEvent = 0;
    void* editMenu = nullptr;
    bool undoEnabled = false;
    bool redoEnabled = false;
#else
    enum class Popup { Closed, Create, Submenu };
    Popup popup = Popup::Closed;
    SDL_FRect rootItem{}, submenu{};
    int activeItem = 0;
#endif
};
