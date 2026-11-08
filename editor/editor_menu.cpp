#include "editor_menu.hpp"

#ifdef _WIN32
#include <windows.h>
#include <commctrl.h>
#include <initializer_list>
#include "menu_ids.h"

namespace
{
    struct MenuItem
    {
        UINT id;
        const wchar_t* label;
        UINT flags = MF_ENABLED;
    };

    bool AddSubmenu(HMENU bar, const wchar_t* title, std::initializer_list<MenuItem> items)
    {
        HMENU popup = CreatePopupMenu();
        if (!popup)
        {
            return SDL_SetError("Cannot create editor submenu (Windows error %lu)", GetLastError());
        }
        if (!AppendMenuW(bar, MF_POPUP, reinterpret_cast<UINT_PTR>(popup), title))
        {
            const DWORD error = GetLastError();
            DestroyMenu(popup);
            return SDL_SetError("Cannot attach editor submenu (Windows error %lu)", error);
        }
        // The bar now owns the popup, including on a later failure.
        for (const MenuItem& item : items)
        {
            if (!AppendMenuW(popup, MF_STRING | item.flags, item.id, item.label))
            {
                return SDL_SetError("Cannot add editor menu item (Windows error %lu)", GetLastError());
            }
        }
        return true;
    }

    LRESULT CALLBACK MenuProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam,
                             UINT_PTR subclass, DWORD_PTR eventType)
    {
        if (message == WM_COMMAND && lParam == 0)
        {
            const int id = LOWORD(wParam);
            if (id == BSC_FILE_SAVE || id == BSC_EDIT_UNDO || id == BSC_EDIT_REDO)
            {
                SDL_Event event{};
                event.type = static_cast<Uint32>(eventType);
                event.user.code = id;
                // Run edits inside the SDL loop, after Windows has dismissed the menu.
                if (!SDL_PushEvent(&event))
                {
                    SDL_Log("Editor menu command: %s", SDL_GetError());
                }
                return 0;
            }
        }
        if (message == WM_NCDESTROY)
        {
            RemoveWindowSubclass(window, MenuProc, subclass);
        }
        return DefSubclassProc(window, message, wParam, lParam);
    }
}
#endif

bool EditorMenu::Attach(SDL_Window* window)
{
    windowID = SDL_GetWindowID(window);
#ifdef _WIN32
    const HWND hwnd = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(window),
                                        SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
    if (!hwnd)
    {
        return SDL_SetError("Cannot find the native editor window");
    }
    commandEvent = SDL_RegisterEvents(1);
    if (!commandEvent)
    {
        return false;
    }
    SDL_SetHint(SDL_HINT_WINDOWS_ENABLE_MENU_MNEMONICS, "1");
    HMENU bar = CreateMenu();
    if (!bar)
    {
        return SDL_SetError("Cannot create the editor menu (Windows error %lu)", GetLastError());
    }
    if (!AddSubmenu(bar, L"&File", {{BSC_FILE_SAVE, L"&Save\tCtrl+S"}}) ||
        !AddSubmenu(bar, L"&Edit", {{BSC_EDIT_UNDO, L"&Undo\tCtrl+Z", MF_GRAYED},
                                    {BSC_EDIT_REDO, L"&Redo\tCtrl+Y", MF_GRAYED}}))
    {
        DestroyMenu(bar);
        return false;
    }
    if (!SetWindowSubclass(hwnd, MenuProc, 1, commandEvent) || !SetMenu(hwnd, bar))
    {
        const DWORD error = GetLastError();
        RemoveWindowSubclass(hwnd, MenuProc, 1);
        DestroyMenu(bar);
        return SDL_SetError("Cannot attach the editor menu (Windows error %lu)", error);
    }
    // The window now owns the menu and destroys it with the window.
    editMenu = GetSubMenu(bar, 1);
    DrawMenuBar(hwnd);
#endif
    return true;
}

void EditorMenu::SetHistoryEnabled(bool canUndo, bool canRedo)
{
#ifdef _WIN32
    if (canUndo != undoEnabled)
    {
        EnableMenuItem(static_cast<HMENU>(editMenu), BSC_EDIT_UNDO, MF_BYCOMMAND | (canUndo ? MF_ENABLED : MF_GRAYED));
        undoEnabled = canUndo;
    }
    if (canRedo != redoEnabled)
    {
        EnableMenuItem(static_cast<HMENU>(editMenu), BSC_EDIT_REDO, MF_BYCOMMAND | (canRedo ? MF_ENABLED : MF_GRAYED));
        redoEnabled = canRedo;
    }
#endif
}

std::optional<EditorCommand> EditorMenu::Command(const SDL_Event& event) const
{
#ifdef _WIN32
    if (event.type == commandEvent)
    {
        switch (event.user.code)
        {
        case BSC_FILE_SAVE: return EditorCommand::Save;
        case BSC_EDIT_UNDO: return EditorCommand::Undo;
        case BSC_EDIT_REDO: return EditorCommand::Redo;
        }
    }
#endif
    if (event.type == SDL_EVENT_KEY_DOWN && event.key.windowID == windowID && !event.key.repeat &&
        (event.key.mod & SDL_KMOD_CTRL) &&
        !(event.key.mod & (SDL_KMOD_ALT | SDL_KMOD_SHIFT | SDL_KMOD_GUI)))
    {
        switch (event.key.key)
        {
        case SDLK_S: return EditorCommand::Save;
        case SDLK_Z: return EditorCommand::Undo;
        case SDLK_Y: return EditorCommand::Redo;
        }
    }
    return std::nullopt;
}
