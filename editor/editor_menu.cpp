#include "editor_menu.hpp"

#include "editor_theme.hpp"
#include <algorithm>
#include <iterator>
#include <span>

#ifdef _WIN32
#include <windows.h>
#include <commctrl.h>
#include "windows_theme.hpp"
#include <string_view>
#include <cwctype>
#include "menu_ids.h"

namespace
{
    struct MenuItem
    {
        UINT id;
        const wchar_t* label;
        UINT flags = MF_ENABLED;
        bool bar = false;
    };

    // Owner-draw data must live as long as the menu, including while Windows tracks a popup.
    constexpr MenuItem file{0, L"&File", 0, true}, edit{0, L"&Edit", 0, true}, view{0, L"&View", 0, true}, create{0, L"&Create"};
    constexpr MenuItem fileItems[] = {{BSC_FILE_SAVE, L"&Save\tCtrl+S"}};
    constexpr MenuItem editItems[] = {{BSC_EDIT_UNDO, L"&Undo\tCtrl+Z", MF_GRAYED},
                                     {BSC_EDIT_REDO, L"&Redo\tCtrl+Y", MF_GRAYED}};
    constexpr MenuItem viewItems[] = {{BSC_VIEW_WIREFRAME, L"&Wireframe"}};
    constexpr MenuItem createItems[] = {{BSC_CREATE_ROAD, L"&Create Road"}, {BSC_CREATE_INTERSECTION, L"Create &Intersection"}, {BSC_CREATE_MARKER, L"&Marker"}};

    void MenuBackground(HMENU menu, SDL_Color color)
    {
        MENUINFO info{sizeof(info)};
        info.fMask = MIM_BACKGROUND;
        info.hbrBack = WindowsTheme::Brush(color);
        SetMenuInfo(menu, &info);
    }

    bool Insert(HMENU menu, const MenuItem& item, HMENU submenu = nullptr)
    {
        MENUITEMINFOW m{sizeof(m)};
        m.fMask = MIIM_ID | MIIM_FTYPE | MIIM_STATE | MIIM_DATA | MIIM_STRING;
        m.fType = MFT_OWNERDRAW;
        m.fState = item.flags;
        m.wID = item.id;
        m.dwItemData = reinterpret_cast<ULONG_PTR>(&item);
        m.dwTypeData = const_cast<wchar_t*>(item.label);
        if (submenu)
        {
            m.fMask |= MIIM_SUBMENU;
            m.hSubMenu = submenu;
        }
        return InsertMenuItemW(menu, GetMenuItemCount(menu), TRUE, &m) != FALSE;
    }

    bool AddSubmenu(HMENU parent, const MenuItem& title, std::span<const MenuItem> items)
    {
        HMENU popup = CreatePopupMenu();
        if (!popup) return SDL_SetError("Cannot create editor submenu (Windows error %lu)", GetLastError());
        MenuBackground(parent, title.bar ? EditorTheme::menuBar : EditorTheme::chrome);
        MenuBackground(popup, EditorTheme::chrome);
        if (!Insert(parent, title, popup))
        {
            DestroyMenu(popup);
            return SDL_SetError("Cannot attach editor submenu");
        }
        for (const auto& item : items)
        {
            if (!Insert(popup, item)) return SDL_SetError("Cannot add editor menu item");
        }
        return true;
    }

    void PaintMenuItem(HWND window, const MenuItem& item, MEASUREITEMSTRUCT* measure, DRAWITEMSTRUCT* draw)
    {
        using namespace EditorTheme;
        const UINT dpi = GetDpiForWindow(window);
        const int pad = MulDiv(8, dpi, 96);
        NONCLIENTMETRICSW metrics{sizeof(metrics)};
        SystemParametersInfoForDpi(SPI_GETNONCLIENTMETRICS, sizeof(metrics), &metrics, 0, dpi);
        HFONT font = CreateFontIndirectW(&metrics.lfMenuFont);
        HDC dc = draw ? draw->hDC : GetDC(window);
        const int saved = SaveDC(dc);
        if (font) SelectObject(dc, font);
        std::wstring_view label(item.label), shortcut;
        const auto tab = label.find(L'\t');
        if (tab != label.npos)
        {
            shortcut = label.substr(tab + 1);
            label = label.substr(0, tab);
        }
        if (measure)
        {
            RECT name{}, key{};
            DrawTextW(dc, label.data(), static_cast<int>(label.size()), &name, DT_SINGLELINE | DT_CALCRECT);
            if (!shortcut.empty()) DrawTextW(dc, shortcut.data(), static_cast<int>(shortcut.size()), &key, DT_SINGLELINE | DT_CALCRECT);
            measure->itemWidth = name.right + key.right + (item.bar ? pad : 5 * pad);
            measure->itemHeight = std::max<int>(GetSystemMetricsForDpi(SM_CYMENU, dpi), name.bottom + (item.bar ? 0 : pad));
        }
        else
        {
            RECT r = draw->rcItem;
            const bool active = draw->itemState & (ODS_SELECTED | ODS_HOTLIGHT);
            const auto background = item.bar ? menuBar : chrome;
            FillRect(dc, &r, WindowsTheme::Brush(active ? selected : background));
            const auto fg = (draw->itemState & (ODS_DISABLED | ODS_GRAYED)) ? disabled : text;
            SetBkMode(dc, TRANSPARENT);
            SetTextColor(dc, WindowsTheme::Color(fg));
            if (!item.bar) { r.left += pad; r.right -= pad; }
            if (item.id == BSC_VIEW_WIREFRAME)
            {
                if (draw->itemState & ODS_CHECKED)
                {
                    const LONG y = (r.top + r.bottom) / 2;
                    const POINT check[] = {{r.left, y}, {r.left + pad / 2, y + pad / 2}, {r.left + pad, y - pad / 2}};
                    SelectObject(dc, GetStockObject(DC_PEN));
                    SetDCPenColor(dc, WindowsTheme::Color(fg));
                    Polyline(dc, check, 3);
                }
                r.left += 2 * pad;
            }
            const UINT prefix = draw->itemState & ODS_NOACCEL ? DT_HIDEPREFIX : 0;
            DrawTextW(dc, label.data(), static_cast<int>(label.size()), &r,
                      DT_SINGLELINE | DT_VCENTER | prefix | (item.bar ? DT_CENTER : DT_LEFT));
            if (!shortcut.empty())
                DrawTextW(dc, shortcut.data(), static_cast<int>(shortcut.size()), &r, DT_SINGLELINE | DT_VCENTER | DT_RIGHT);
        }
        RestoreDC(dc, saved);
        if (!draw) ReleaseDC(window, dc);
        if (font) DeleteObject(font);
    }

    void PaintMenuBarFrame(HWND window)
    {
        HMENU menu = GetMenu(window);
        MENUBARINFO bar{sizeof(bar)};
        RECT bounds{}, client{};
        POINT origin{};
        if (!menu || IsIconic(window) || !GetMenuBarInfo(window, OBJID_MENU, 0, &bar) ||
            IsRectEmpty(&bar.rcBar) || !GetWindowRect(window, &bounds) || !GetClientRect(window, &client) ||
            IsRectEmpty(&client) || !ClientToScreen(window, &origin))
        {
            return;
        }
        HDC dc = GetWindowDC(window);
        if (!dc) return;
        HBRUSH brush = WindowsTheme::Brush(EditorTheme::menuBar);
        RECT last{};
        if (GetMenuItemRect(window, menu, GetMenuItemCount(menu) - 1, &last))
        {
            RECT unused{last.right, last.top, bar.rcBar.right, last.bottom};
            OffsetRect(&unused, -bounds.left, -bounds.top);
            FillRect(dc, &unused, brush);
        }

        // Windows paints a one-pixel separator outside the item rectangles.
        // Anchor it to the actual client edge so resizing and DPI changes keep it above the toolbar.
        OffsetRect(&client, origin.x - bounds.left, origin.y - bounds.top);
        RECT separator{client.left, client.top - 1, client.right, client.top};
        FillRect(dc, &separator, brush);
        ReleaseDC(window, dc);
    }

    LRESULT CALLBACK MenuProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam,
                             UINT_PTR subclass, DWORD_PTR eventType)
    {
        if (message == WM_NCPAINT || message == WM_NCACTIVATE || message == WM_EXITMENULOOP)
        {
            const LRESULT result = DefSubclassProc(window, message, wParam, lParam);
            PaintMenuBarFrame(window);
            return result;
        }
        if (message == WM_MEASUREITEM || message == WM_DRAWITEM)
        {
            auto* measure = message == WM_MEASUREITEM ? reinterpret_cast<MEASUREITEMSTRUCT*>(lParam) : nullptr;
            auto* draw = message == WM_DRAWITEM ? reinterpret_cast<DRAWITEMSTRUCT*>(lParam) : nullptr;
            if ((measure ? measure->CtlType : draw->CtlType) == ODT_MENU)
            {
                const auto* item = reinterpret_cast<const MenuItem*>(measure ? measure->itemData : draw->itemData);
                if (item)
                {
                    PaintMenuItem(window, *item, measure, draw);
                    return TRUE;
                }
            }
        }
        if (message == WM_MENUCHAR)
        {
            const HMENU menu = reinterpret_cast<HMENU>(lParam);
            for (int i = 0; i < GetMenuItemCount(menu); ++i)
            {
                wchar_t title[128]{};
                MENUITEMINFOW info{sizeof(info)};
                info.fMask = MIIM_STRING | MIIM_STATE;
                info.dwTypeData = title;
                info.cch = 128;
                if (!GetMenuItemInfoW(menu, i, TRUE, &info) || (info.fState & MFS_DISABLED)) continue;
                const wchar_t* key = wcschr(title, L'&');
                if (key && key[1] && towupper(key[1]) == towupper(LOWORD(wParam))) return MAKELRESULT(i, MNC_EXECUTE);
            }
        }
        if (message == WM_COMMAND && lParam == 0)
        {
            const int id = LOWORD(wParam);
            if (id == BSC_FILE_SAVE || id == BSC_EDIT_UNDO || id == BSC_EDIT_REDO || id == BSC_VIEW_WIREFRAME)
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
    WindowsTheme::TitleBar(hwnd);
    if (!AddSubmenu(bar, file, fileItems) || !AddSubmenu(bar, edit, editItems) || !AddSubmenu(bar, view, viewItems))
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
    viewMenu = GetSubMenu(bar, 2);
    DrawMenuBar(hwnd);
#endif
    return true;
}

void EditorMenu::ToggleWireframe()
{
    wireframe = !wireframe;
#ifdef _WIN32
    CheckMenuItem(static_cast<HMENU>(viewMenu), BSC_VIEW_WIREFRAME, MF_BYCOMMAND | (wireframe ? MF_CHECKED : MF_UNCHECKED));
#endif
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
#else
    undoEnabled = canUndo;
    redoEnabled = canRedo;
#endif
}

std::optional<EditorCommand> EditorMenu::Command(const SDL_Event& event) const
{
#ifdef _WIN32
    if (event.type == commandEvent)
    {
        switch (event.user.code)
        {
        case BSC_VIEW_WIREFRAME: return EditorCommand::ToggleWireframe;
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
    struct BarAction { const char* label; EditorCommand command; };
    constexpr BarAction save[] = {{"Save", EditorCommand::Save}};
    constexpr BarAction history[] = {{"Undo", EditorCommand::Undo}, {"Redo", EditorCommand::Redo}};
    constexpr BarAction display[] = {{"Wireframe", EditorCommand::ToggleWireframe}};
    struct BarMenu { const char* label; std::span<const BarAction> items; };
    constexpr BarMenu menus[] = {{"File", save}, {"Edit", history}, {"View", display}};
    constexpr int barTabWidth = 52, dropdownWidth = 136;

    struct CreateItem
    {
        const char* label;
        EditorCommand command;
    };
    constexpr CreateItem createItems[] = {{"Create Road", EditorCommand::CreateRoad},
                                          {"Create Intersection", EditorCommand::CreateIntersection},
                                          {"Marker", EditorCommand::CreateMarker}};
    constexpr int createItemCount = static_cast<int>(std::size(createItems));
    constexpr float itemHeight = 26;
}
#endif

#ifdef _WIN32
bool EditorMenu::OpenMaterialMenu(SDL_Window* window, int screenX, int screenY, bool onMaterial)
{
    constexpr static MenuItem add{BSC_ADD_MATERIAL, L"&Add New Material"};
    constexpr static MenuItem duplicate{BSC_DUPLICATE_MATERIAL, L"&Duplicate\tCtrl+D"};
    const auto& item = onMaterial ? duplicate : add;
    HWND owner = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(window),
                                   SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
    HMENU menu = CreatePopupMenu();
    if (!menu) return false;
    MenuBackground(menu, EditorTheme::chrome);
    const bool chosen = Insert(menu, item) && TrackPopupMenuEx(menu, TPM_RETURNCMD | TPM_NONOTIFY | TPM_RIGHTBUTTON,
                                                             screenX, screenY, owner, nullptr) == item.id;
    DestroyMenu(menu);
    return chosen;
}
#endif

std::optional<EditorCommand> EditorMenu::OpenViewport(SDL_FPoint windowPoint, SDL_FPoint viewportPoint, SDL_Rect scene, bool canCreateIntersection)
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
    if (ClientToScreen(hwnd, &at) && AddSubmenu(root, create, createItems))
    {
        if (!canCreateIntersection) EnableMenuItem(GetSubMenu(root, 0), BSC_CREATE_INTERSECTION, MF_BYCOMMAND | MF_GRAYED);
        choice = TrackPopupMenuEx(root, TPM_RETURNCMD | TPM_NONOTIFY | TPM_RIGHTBUTTON,
                                  at.x, at.y, hwnd, nullptr);
    }
    DestroyMenu(root);
    if (choice == BSC_CREATE_ROAD)
    {
        return EditorCommand::CreateRoad;
    }
    if (choice == BSC_CREATE_INTERSECTION) return EditorCommand::CreateIntersection;
    if (choice == BSC_CREATE_MARKER)
    {
        return EditorCommand::CreateMarker;
    }
#else
    intersectionEnabled = canCreateIntersection;
    rootItem = {0, 0, 104, itemHeight};
    submenu = {0, 0, 170, itemHeight * createItemCount};
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
bool EditorMenu::Enabled(EditorCommand command) const
{
    if (command == EditorCommand::CreateIntersection) return intersectionEnabled;
    if (command == EditorCommand::Undo) return undoEnabled;
    if (command == EditorCommand::Redo) return redoEnabled;
    return true;
}

bool EditorMenu::DrawBar(SDL_Renderer* renderer, int width) const
{
    const SDL_FRect bar{0, 0, static_cast<float>(width), barHeight};
    if (!EditorTheme::SetColor(renderer, EditorTheme::menuBar) || !SDL_RenderFillRect(renderer, &bar)) return false;
    for (int i = 0; i < static_cast<int>(std::size(menus)); ++i)
    {
        const SDL_FRect tab{static_cast<float>(i * barTabWidth), 0, barTabWidth, barHeight};
        if (i == openMenu && (!EditorTheme::SetColor(renderer, EditorTheme::selected) || !SDL_RenderFillRect(renderer, &tab))) return false;
        if (!EditorTheme::SetColor(renderer, EditorTheme::text) ||
            !SDL_RenderDebugText(renderer, tab.x + 8, 8, menus[i].label)) return false;
    }
    if (openMenu < 0) return true;
    const auto items = menus[openMenu].items;
    for (int i = 0; i < static_cast<int>(items.size()); ++i)
    {
        const SDL_FRect row{static_cast<float>(openMenu * barTabWidth), barHeight + i * itemHeight,
                            dropdownWidth, itemHeight};
        const bool enabled = Enabled(items[i].command);
        if (!EditorTheme::SetColor(renderer, i == barRow && enabled ? EditorTheme::selected : EditorTheme::chrome) ||
            !SDL_RenderFillRect(renderer, &row) ||
            !EditorTheme::SetColor(renderer, enabled ? EditorTheme::text : EditorTheme::disabled) ||
            !SDL_RenderDebugText(renderer, row.x + 24, row.y + 9, items[i].label)) return false;
        if (items[i].command == EditorCommand::ToggleWireframe && wireframe)
        {
            const SDL_FPoint check[] = {{row.x + 6, row.y + 13}, {row.x + 9, row.y + 16}, {row.x + 16, row.y + 8}};
            if (!SDL_RenderLines(renderer, check, 3)) return false;
        }
    }
    return true;
}

bool EditorMenu::HandleBar(SDL_Renderer* renderer, const SDL_Event& event, std::optional<EditorCommand>& command)
{
    constexpr int count = static_cast<int>(std::size(menus));
    if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST || event.type == SDL_EVENT_WINDOW_RESIZED)
    {
        openMenu = -1;
        return false;
    }
    if (openMenu >= 0 && event.type == SDL_EVENT_KEY_DOWN)
    {
        const auto items = menus[openMenu].items;
        const int rows = static_cast<int>(items.size());
        switch (event.key.key)
        {
        case SDLK_ESCAPE: openMenu = -1; break;
        case SDLK_LEFT: case SDLK_RIGHT:
            openMenu = (openMenu + count + (event.key.key == SDLK_RIGHT ? 1 : -1)) % count;
            barRow = 0;
            break;
        case SDLK_UP: case SDLK_DOWN:
            barRow = (barRow + rows + (event.key.key == SDLK_DOWN ? 1 : -1)) % rows;
            break;
        case SDLK_RETURN: case SDLK_KP_ENTER:
            if (Enabled(items[barRow].command)) command = items[barRow].command;
            openMenu = -1;
            break;
        }
        return true;
    }
    const bool press = event.type == SDL_EVENT_MOUSE_BUTTON_DOWN;
    if (!press && event.type != SDL_EVENT_MOUSE_MOTION)
        return openMenu >= 0 && (event.type == SDL_EVENT_MOUSE_BUTTON_UP || event.type == SDL_EVENT_MOUSE_WHEEL);
    if (!press && openMenu < 0) return false;
    SDL_Event e = event;
    if (!SDL_SetRenderViewport(renderer, nullptr) || !SDL_ConvertEventToRenderCoordinates(renderer, &e)) return false;
    const SDL_FPoint p = press ? SDL_FPoint{e.button.x, e.button.y} : SDL_FPoint{e.motion.x, e.motion.y};
    if (p.y >= 0 && p.y < barHeight)
    {
        const int tab = p.x >= 0 && p.x < count * barTabWidth ? static_cast<int>(p.x / barTabWidth) : -1;
        if (press || tab >= 0)
        {
            openMenu = press && (e.button.button != SDL_BUTTON_LEFT || openMenu == tab) ? -1 : tab;
            barRow = 0;
        }
        return true;
    }
    if (openMenu < 0) return false;
    const auto items = menus[openMenu].items;
    const SDL_FRect popupRect{static_cast<float>(openMenu * barTabWidth), barHeight,
                              dropdownWidth, items.size() * itemHeight};
    if (SDL_PointInRectFloat(&p, &popupRect))
    {
        barRow = static_cast<int>((p.y - barHeight) / itemHeight);
        if (press && e.button.button == SDL_BUTTON_LEFT && Enabled(items[barRow].command))
            command = items[barRow].command;
    }
    if (press) openMenu = -1;
    return true;
}

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
                do { activeItem = (activeItem + createItemCount + step) % createItemCount; }
                while (!Enabled(createItems[activeItem].command));
            }
            break;
        case SDLK_RIGHT: popup = Popup::Submenu; break;
        case SDLK_RETURN:
        case SDLK_KP_ENTER:
            if (popup == Popup::Submenu)
            {
                if (Enabled(createItems[activeItem].command)) command = createItems[activeItem].command;
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
                    if (Enabled(createItems[activeItem].command)) command = createItems[activeItem].command;
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
    const auto item = [&](SDL_FRect rect, const char* label, bool active, bool enabled = true)
    {
        return EditorTheme::SetColor(renderer, active && enabled ? EditorTheme::selected : EditorTheme::panel) &&
               SDL_RenderFillRect(renderer, &rect) &&
               EditorTheme::SetColor(renderer, EditorTheme::border) && SDL_RenderRect(renderer, &rect) &&
               EditorTheme::SetColor(renderer, enabled ? EditorTheme::text : EditorTheme::disabled) &&
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
            if (!item(row, createItems[i].label, i == activeItem, Enabled(createItems[i].command)))
            {
                return false;
            }
            row.y += itemHeight;
        }
    }
    return true;
}
#endif
