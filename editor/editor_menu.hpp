#pragma once

#include <SDL3/SDL.h>
#include <optional>

enum class EditorCommand { Save, Undo, Redo };

class EditorMenu
{
public:
    bool Attach(SDL_Window* window);
    void SetHistoryEnabled(bool canUndo, bool canRedo);
    std::optional<EditorCommand> Command(const SDL_Event& event) const;

private:
    SDL_WindowID windowID = 0;
#ifdef _WIN32
    Uint32 commandEvent = 0;
    void* editMenu = nullptr;
    bool undoEnabled = false;
    bool redoEnabled = false;
#endif
};
