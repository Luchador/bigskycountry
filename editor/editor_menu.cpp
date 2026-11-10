#include "editor_menu.hpp"

#include <algorithm>
#include <iterator>

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

#ifndef _WIN32
namespace
{
    struct CreateItem
    {
        const char* label;
        EditorCommand command;
    };
    constexpr CreateItem createItems[] = {{"Create Road", EditorCommand::CreateRoad},
                                          {"Test Terrain", EditorCommand::CreateTestTerrain}};
    constexpr int createItemCount = static_cast<int>(std::size(createItems));
    constexpr float itemHeight = 26;
}
#endif

std::optional<EditorCommand> EditorMenu::OpenViewport(SDL_FPoint windowPoint, SDL_FPoint viewportPoint, SDL_Rect scene)
{
#ifdef _WIN32
    SDL_Window* window = SDL_GetWindowFromID(windowID);
    const HWND hwnd = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(window),
                                         SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
    RECT client{};
    int width, height;
    if (!GetClientRect(hwnd, &client) || !SDL_GetWindowSize(window, &width, &height) || width <= 0 || height <= 0)
    {
        return {};
    }
    POINT at{static_cast<LONG>(windowPoint.x * client.right / width),
             static_cast<LONG>(windowPoint.y * client.bottom / height)};
    HMENU root = CreatePopupMenu();
    if (!root)
    {
        return {};
    }
    UINT choice = 0;
    if (ClientToScreen(hwnd, &at) && AddSubmenu(root, L"Create", {{BSC_CREATE_ROAD, L"Create Road"},
                                                                {BSC_CREATE_TEST_TERRAIN, L"Test Terrain"}}))
    {
        choice = TrackPopupMenuEx(root, TPM_RETURNCMD | TPM_NONOTIFY | TPM_RIGHTBUTTON,
                                  at.x, at.y, hwnd, nullptr);
    }
    DestroyMenu(root);
    if (choice == BSC_CREATE_ROAD)
    {
        return EditorCommand::CreateRoad;
    }
    if (choice == BSC_CREATE_TEST_TERRAIN)
    {
        return EditorCommand::CreateTestTerrain;
    }
#else
    rootItem = {0, 0, 104, itemHeight};
    submenu = {0, 0, 120, itemHeight * createItemCount};
    const auto fit = [](float start, float extent, int limit)
    {
        return std::clamp(start, 0.0f, std::max(0.0f, limit - extent));
    };
    rootItem.x = fit(viewportPoint.x, rootItem.w, scene.w);
    rootItem.y = fit(viewportPoint.y, itemHeight, scene.h);
    submenu.x = rootItem.x + rootItem.w;
    submenu.y = fit(rootItem.y, submenu.h, scene.h);
    if (submenu.x + submenu.w > scene.w)
    {
        if (rootItem.x >= submenu.w)
        {
            submenu.x = rootItem.x - submenu.w;
        }
        else
        {
            submenu.x = fit(rootItem.x, submenu.w, scene.w);
            const float below = rootItem.y + itemHeight;
            const float top = below + submenu.h <= scene.h ? below : rootItem.y - submenu.h;
            submenu.y = fit(top, submenu.h, scene.h);
        }
    }
    activeItem = 0;
    popup = Popup::Create;
#endif
    return {};
}

#ifndef _WIN32
bool EditorMenu::HandleViewportMenu(SDL_Renderer* renderer, SDL_Rect scene, const SDL_Event& event,
                                    std::optional<EditorCommand>& command)
{
    if (popup == Popup::Closed)
    {
        return false;
    }
    switch (event.type)
    {
    case SDL_EVENT_WINDOW_FOCUS_LOST:
    case SDL_EVENT_WINDOW_RESIZED:
        popup = Popup::Closed;
        return false;
    case SDL_EVENT_KEY_DOWN:
        if (event.key.repeat)
        {
            return true;
        }
        switch (event.key.key)
        {
        case SDLK_ESCAPE: popup = Popup::Closed; break;
        case SDLK_LEFT:
            popup = Popup::Create;
            activeItem = 0;
            break;
        case SDLK_UP:
        case SDLK_DOWN:
            if (popup == Popup::Submenu)
            {
                const int step = event.key.key == SDLK_DOWN ? 1 : -1;
                activeItem = (activeItem + createItemCount + step) % createItemCount;
            }
            break;
        case SDLK_RIGHT: popup = Popup::Submenu; break;
        case SDLK_RETURN:
        case SDLK_KP_ENTER:
            if (popup == Popup::Submenu)
            {
                command = createItems[activeItem].command;
                popup = Popup::Closed;
            }
            else
            {
                popup = Popup::Submenu;
            }
            break;
        }
        return true;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
    case SDL_EVENT_MOUSE_MOTION:
    {
        const bool press = event.type == SDL_EVENT_MOUSE_BUTTON_DOWN;
        SDL_FPoint at{};
        if (SDL_SetRenderViewport(renderer, &scene) &&
            SDL_RenderCoordinatesFromWindow(renderer, press ? event.button.x : event.motion.x,
                                             press ? event.button.y : event.motion.y, &at.x, &at.y))
        {
            if ((!press || event.button.button == SDL_BUTTON_LEFT) && SDL_PointInRectFloat(&at, &rootItem))
            {
                popup = Popup::Submenu;
                return true;
            }
            if (popup == Popup::Submenu && SDL_PointInRectFloat(&at, &submenu))
            {
                activeItem = std::min(static_cast<int>((at.y - submenu.y) / itemHeight), createItemCount - 1);
                if (press && event.button.button == SDL_BUTTON_LEFT)
                {
                    command = createItems[activeItem].command;
                }
            }
        }
        if (press)
        {
            popup = Popup::Closed;
        }
        return true;
    }
    case SDL_EVENT_KEY_UP:
    case SDL_EVENT_MOUSE_BUTTON_UP:
    case SDL_EVENT_MOUSE_WHEEL:
        return true;
    default:
        return false;
    }
}

bool EditorMenu::DrawViewportMenu(SDL_Renderer* renderer) const
{
    const auto item = [&](SDL_FRect rect, const char* label, bool active)
    {
        return SDL_SetRenderDrawColor(renderer, active ? 58 : 38, active ? 75 : 43, active ? 88 : 49, 255) &&
               SDL_RenderFillRect(renderer, &rect) &&
               SDL_SetRenderDrawColor(renderer, 118, 127, 136, 255) && SDL_RenderRect(renderer, &rect) &&
               SDL_SetRenderDrawColor(renderer, 235, 239, 244, 255) &&
               SDL_RenderDebugText(renderer, rect.x + 8, rect.y + 9, label);
    };
    if (popup == Popup::Closed)
    {
        return true;
    }
    if (!item(rootItem, "Create    >", false))
    {
        return false;
    }
    if (popup == Popup::Submenu)
    {
        SDL_FRect row{submenu.x, submenu.y, submenu.w, itemHeight};
        for (int i = 0; i < createItemCount; ++i)
        {
            if (!item(row, createItems[i].label, i == activeItem))
            {
                return false;
            }
            row.y += itemHeight;
        }
    }
    return true;
}
#endif
