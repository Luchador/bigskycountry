#include "editor_menu.hpp"

#include "editor_theme.hpp"
#include <algorithm>
#include <array>
#include <iterator>
#include <span>
#include <string_view>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <commctrl.h>
#include "windows_theme.hpp"
#include "scene_outliner.hpp"
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
    constexpr MenuItem file{0, L"&File", 0, true}, edit{0, L"&Edit", 0, true}, view{0, L"&View", 0, true},
                       settings{0, L"&Settings", 0, true};
    constexpr MenuItem fileItems[] = {{BSC_FILE_SAVE, L"&Save\tCtrl+S"}, {BSC_FILE_COOK_WORLD, L"&Cook World"}};
    constexpr MenuItem editItems[] = {{BSC_EDIT_UNDO, L"&Undo\tCtrl+Z", MF_GRAYED},
                                     {BSC_EDIT_REDO, L"&Redo\tCtrl+Y", MF_GRAYED}};
    constexpr MenuItem viewItems[] = {{BSC_VIEW_WIREFRAME, L"&Wireframe"},
                                      {BSC_VIEW_ROAD_NAMES, L"Road &Names", MF_CHECKED},
                                      {BSC_VIEW_TRAFFIC_LANES, L"Traffic &Lanes"},
                                      {BSC_VIEW_FOG, L"&Fog", MF_CHECKED},
                                      {BSC_VIEW_SHADOWS, L"&Shadows", MF_CHECKED},
                                      {BSC_VIEW_VISIBILITY_STATS, L"Visibility &Statistics\tF3"}};
    constexpr MenuItem settingsItems[] = {{BSC_PROJECT_SETTINGS, L"&Project Settings"}};
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
            if (item.id == BSC_VIEW_WIREFRAME || item.id == BSC_VIEW_ROAD_NAMES || item.id == BSC_VIEW_FOG || item.id == BSC_VIEW_SHADOWS || item.id == BSC_VIEW_VISIBILITY_STATS)
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
            if (id == BSC_FILE_SAVE || id == BSC_FILE_COOK_WORLD || id == BSC_EDIT_UNDO || id == BSC_EDIT_REDO || id == BSC_VIEW_WIREFRAME ||
                id == BSC_VIEW_ROAD_NAMES || id == BSC_VIEW_FOG || id == BSC_VIEW_SHADOWS || id == BSC_VIEW_VISIBILITY_STATS || id == BSC_PROJECT_SETTINGS)
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
    if (!AddSubmenu(bar, file, fileItems) || !AddSubmenu(bar, edit, editItems) ||
        !AddSubmenu(bar, view, viewItems) || !AddSubmenu(bar, settings, settingsItems))
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

void EditorMenu::ToggleRoadNames()
{
    roadNames = !roadNames;
#ifdef _WIN32
    CheckMenuItem(static_cast<HMENU>(viewMenu), BSC_VIEW_ROAD_NAMES, MF_BYCOMMAND | (roadNames ? MF_CHECKED : MF_UNCHECKED));
#endif
}

void EditorMenu::ToggleTrafficLanes()
{
    trafficLanes = !trafficLanes;
#ifdef _WIN32
    CheckMenuItem(static_cast<HMENU>(viewMenu), BSC_VIEW_TRAFFIC_LANES, MF_BYCOMMAND | (trafficLanes ? MF_CHECKED : MF_UNCHECKED));
#endif
}

void EditorMenu::ToggleShadows()
{
    shadows = !shadows;
#ifdef _WIN32
    CheckMenuItem(static_cast<HMENU>(viewMenu), BSC_VIEW_SHADOWS, MF_BYCOMMAND | (shadows ? MF_CHECKED : MF_UNCHECKED));
#endif
}

void EditorMenu::ToggleFog()
{
    fog = !fog;
#ifdef _WIN32
    CheckMenuItem(static_cast<HMENU>(viewMenu), BSC_VIEW_FOG, MF_BYCOMMAND | (fog ? MF_CHECKED : MF_UNCHECKED));
#endif
}

void EditorMenu::ToggleVisibilityStatistics()
{
    visibilityStatistics = !visibilityStatistics;
#ifdef _WIN32
    CheckMenuItem(static_cast<HMENU>(viewMenu), BSC_VIEW_VISIBILITY_STATS,
                  MF_BYCOMMAND | (visibilityStatistics ? MF_CHECKED : MF_UNCHECKED));
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
        case BSC_VIEW_ROAD_NAMES: return EditorCommand::ToggleRoadNames;
        case BSC_VIEW_TRAFFIC_LANES: return EditorCommand::ToggleTrafficLanes;
        case BSC_VIEW_SHADOWS: return EditorCommand::ToggleShadows;
        case BSC_VIEW_FOG: return EditorCommand::ToggleFog;
        case BSC_VIEW_VISIBILITY_STATS: return EditorCommand::ToggleVisibilityStatistics;
        case BSC_PROJECT_SETTINGS: return EditorCommand::ProjectSettings;
        case BSC_FILE_COOK_WORLD: return EditorCommand::CookWorld;
        case BSC_FILE_SAVE: return EditorCommand::Save;
        case BSC_EDIT_UNDO: return EditorCommand::Undo;
        case BSC_EDIT_REDO: return EditorCommand::Redo;
        }
    }
#endif
    if (event.type == SDL_EVENT_KEY_DOWN && event.key.windowID == windowID && !event.key.repeat &&
        event.key.key == SDLK_F3 && !(event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_SHIFT | SDL_KMOD_GUI)))
        return EditorCommand::ToggleVisibilityStatistics;
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
    constexpr BarAction save[] = {{"Save", EditorCommand::Save}, {"Cook World", EditorCommand::CookWorld}};
    constexpr BarAction history[] = {{"Undo", EditorCommand::Undo}, {"Redo", EditorCommand::Redo}};
    constexpr BarAction display[] = {{"Wireframe", EditorCommand::ToggleWireframe},
                                     {"Road Names", EditorCommand::ToggleRoadNames},
                                     {"Traffic Lanes", EditorCommand::ToggleTrafficLanes},
                                     {"Fog", EditorCommand::ToggleFog},
                                     {"Shadows", EditorCommand::ToggleShadows},
                                     {"Visibility Statistics  F3", EditorCommand::ToggleVisibilityStatistics}};
    constexpr BarAction settings[] = {{"Project Settings", EditorCommand::ProjectSettings}};
    struct BarMenu { const char* label; std::span<const BarAction> items; };
    constexpr BarMenu menus[] = {{"File", save}, {"Edit", history}, {"View", display}, {"Settings", settings}};
    constexpr int dropdownWidth = 232;

    constexpr auto barTabs = []
    {
        std::array<SDL_FRect, std::size(menus)> tabs{};
        float x = 0;
        for (std::size_t i = 0; i < tabs.size(); ++i)
        {
            const auto characters = std::string_view(menus[i].label).size();
            const float width = std::max(52.0f, 16.0f + static_cast<float>(characters * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE));
            tabs[i] = {x, 0, width, EditorMenu::barHeight};
            x += width;
        }
        return tabs;
    }();

    constexpr const char* viewportCategories[] = {"Infrastructure", "Water", "Special"};
    constexpr float itemHeight = 26;
}
#endif

#ifdef _WIN32
namespace
{
    UINT TrackCommands(HWND owner, int x, int y, std::span<const MenuItem> items)
    {
        HMENU menu = CreatePopupMenu();
        if (!menu) return 0;
        MenuBackground(menu, EditorTheme::chrome);
        UINT chosen = 0;
        if (std::ranges::all_of(items, [&](const auto& item) { return Insert(menu, item); }))
            chosen = TrackPopupMenuEx(menu, TPM_RETURNCMD | TPM_NONOTIFY | TPM_RIGHTBUTTON, x, y, owner, nullptr);
        DestroyMenu(menu);
        return chosen;
    }
}

std::optional<OutlineOp> EditorMenu::OpenFolderMenu(void* frame, int screenX, int screenY)
{
    constexpr MenuItem items[] = {{BSC_HIDE_CHILDREN, L"&Hide Children"}, {BSC_UNHIDE_CHILDREN, L"&Unhide Children"}};
    switch (TrackCommands(static_cast<HWND>(frame), screenX, screenY, items))
    {
    case BSC_HIDE_CHILDREN: return OutlineOp::HideChildren;
    case BSC_UNHIDE_CHILDREN: return OutlineOp::UnhideChildren;
    default: return {};
    }
}

std::optional<MaterialAction> EditorMenu::OpenMaterialMenu(SDL_Window* window, int screenX, int screenY, bool onMaterial)
{
    constexpr static MenuItem add{BSC_ADD_MATERIAL, L"&Add New Material"};
    constexpr static MenuItem actions[] = {{BSC_DUPLICATE_MATERIAL, L"&Duplicate\tCtrl+D"},
                                          {BSC_RENAME_MATERIAL, L"&Rename\tF2"},
                                          {BSC_DELETE_MATERIAL, L"D&elete\tDelete"}};
    HWND owner = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(window),
                                   SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
    switch (TrackCommands(owner, screenX, screenY, onMaterial ? std::span<const MenuItem>(actions) : std::span<const MenuItem>(&add, 1)))
    {
    case BSC_ADD_MATERIAL: return MaterialAction::Create;
    case BSC_DUPLICATE_MATERIAL: return MaterialAction::Duplicate;
    case BSC_RENAME_MATERIAL: return MaterialAction::Rename;
    case BSC_DELETE_MATERIAL: return MaterialAction::Delete;
    default: return {};
    }
}
#endif

std::optional<EditorCommand> EditorMenu::OpenViewport(SDL_FPoint windowPoint, SDL_FPoint viewportPoint, SDL_Rect scene, bool canCreateIntersection,
                                                       std::optional<RampMenuOptions> ramps, std::optional<bool> canFlatten, bool crossingAvailable, bool crossingExists, std::optional<SplitMenuOptions> split, std::optional<bool> deadEnd, std::optional<bool> overpass, std::optional<std::array<bool,4>> switches, CurveMenuOptions curves, RiverMenuOptions rivers)
{
    // Both menu backends use the same categories and context-sensitive actions.
    auto& infrastructure = viewportItems[0];
    infrastructure = {{"Create Road", EditorCommand::CreateRoad},
        {"Create Railroad", EditorCommand::CreateRailroad},
        {"Create Intersection", EditorCommand::CreateIntersection, canCreateIntersection},
        {"Create Y-Intersection", EditorCommand::CreateYIntersection},
        {"Create Town Square", EditorCommand::CreateTownSquare},
        {"Marker", EditorCommand::CreateMarker},
        {crossingExists ? "Remove Railroad Crossing" : "Create Railroad Crossing", EditorCommand::ToggleRailroadCrossing, crossingAvailable}};
    if (ramps)
    {
        infrastructure.push_back({"Create Exit Ramp", EditorCommand::CreateExitRamp, ramps->exit});
        infrastructure.push_back({"Create Entrance Ramp", EditorCommand::CreateEntranceRamp, ramps->entrance});
    }
    if (switches)
    {
        constexpr const char* labels[] = {"Left Railroad Switch", "Right Railroad Switch",
            "Left Railroad Switch (Reverse)", "Right Railroad Switch (Reverse)"};
        for (int i = 0; i < 4; ++i)
            infrastructure.push_back({labels[i], static_cast<EditorCommand>(static_cast<int>(EditorCommand::CreateRailSwitchLeft) + i), (*switches)[i]});
    }
    if (overpass) infrastructure.push_back({*overpass ? "Remove Overpass" : "Overpass", EditorCommand::ToggleOverpass});
    if (deadEnd) infrastructure.push_back({"Dead End", EditorCommand::CreateDeadEnd, *deadEnd});
    if (canFlatten) infrastructure.push_back({"Flatten Segment", EditorCommand::FlattenSegment, *canFlatten});
    if (split) infrastructure.push_back({split->railroad ? "Split Railroad" : "Split Road", EditorCommand::SplitRoad, split->enabled});
    if (curves.start) infrastructure.push_back({"Start Curved Connector", EditorCommand::StartCurvedConnector});
    if (curves.finish) infrastructure.push_back({"Finish Curved Connector Here", EditorCommand::FinishCurvedConnector});
    if (curves.cancel) infrastructure.push_back({"Cancel Curved Connector", EditorCommand::CancelCurvedConnector});
    auto& water = viewportItems[1];
    water = {{"Create Lake", EditorCommand::CreateLake}, {"Create River", EditorCommand::CreateRiver}};
    if (rivers.start) water.push_back({"Connect River End", EditorCommand::StartRiverConnection});
    if (rivers.finish) water.push_back({"Connect River Here", EditorCommand::FinishRiverConnection});
    if (rivers.disconnect) water.push_back({"Disconnect River", EditorCommand::DisconnectRiver});
    if (rivers.cancel) water.push_back({"Cancel River Connection", EditorCommand::CancelRiverConnection});
    viewportItems[2] = {{"Spawn Point", EditorCommand::CreateSpawnPoint}};
#ifdef _WIN32
    SDL_Window* window = SDL_GetWindowFromID(windowID);
    const HWND hwnd = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(window),
                                         SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
    RECT client{};
    int width, height;
    if (!GetClientRect(hwnd, &client) || !SDL_GetWindowSize(window, &width, &height) || width <= 0 || height <= 0) return {};
    POINT at{static_cast<LONG>(windowPoint.x * client.right / width), static_cast<LONG>(windowPoint.y * client.bottom / height)};
    HMENU root = CreatePopupMenu();
    if (!root) return {};
    constexpr MenuItem categories[] = {{0, L"&Infrastructure"}, {0, L"&Water"}, {0, L"&Special"}};
    // Stable backing storage for the native owner-drawn item data and labels.
    std::array<std::vector<std::wstring>, 3> labels;
    std::array<std::vector<MenuItem>, 3> entries;
    constexpr UINT commandBase = 1000;
    bool ready = true;
    for (std::size_t group = 0; group < viewportItems.size() && ready; ++group)
    {
        labels[group].reserve(viewportItems[group].size());
        entries[group].reserve(viewportItems[group].size());
        for (const auto& action : viewportItems[group])
        {
            const std::string_view text(action.label);
            labels[group].emplace_back(text.begin(), text.end());
            entries[group].push_back({commandBase + static_cast<UINT>(action.command), labels[group].back().c_str(),
                                      static_cast<UINT>(action.enabled ? MF_ENABLED : MF_GRAYED)});
        }
        ready = AddSubmenu(root, categories[group], entries[group]);
    }
    UINT choice = 0;
    if (ready && ClientToScreen(hwnd, &at))
        choice = TrackPopupMenuEx(root, TPM_RETURNCMD | TPM_NONOTIFY | TPM_RIGHTBUTTON, at.x, at.y, hwnd, nullptr);
    DestroyMenu(root);
    if (choice >= commandBase && choice <= commandBase + static_cast<UINT>(EditorCommand::CreateTownSquare))
        return static_cast<EditorCommand>(choice - commandBase);
#else
    openMenu = -1;
    const float width = 176;
    const float x = std::clamp(viewportPoint.x, 0.0f, std::max(0.0f, scene.w - width));
    const float y = std::clamp(viewportPoint.y, 0.0f, std::max(0.0f, scene.h - 3 * itemHeight));
    for (int i = 0; i < 3; ++i) rootItems[i] = {x, y + i * itemHeight, width, itemHeight};
    activeCategory = activeItem = firstItem = 0;
    LayoutViewportSubmenu(scene);
    popup = Popup::Root;
#endif
    return {};
}

#ifndef _WIN32
bool EditorMenu::Enabled(EditorCommand command) const
{
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
        const SDL_FRect& tab = barTabs[i];
        if (i == openMenu && (!EditorTheme::SetColor(renderer, EditorTheme::selected) || !SDL_RenderFillRect(renderer, &tab))) return false;
        if (!EditorTheme::SetColor(renderer, EditorTheme::text) ||
            !SDL_RenderDebugText(renderer, tab.x + 8, 8, menus[i].label)) return false;
    }
    if (openMenu < 0) return true;
    const auto items = menus[openMenu].items;
    for (int i = 0; i < static_cast<int>(items.size()); ++i)
    {
        const SDL_FRect row{barTabs[openMenu].x, barHeight + i * itemHeight,
                            dropdownWidth, itemHeight};
        const bool enabled = Enabled(items[i].command);
        if (!EditorTheme::SetColor(renderer, i == barRow && enabled ? EditorTheme::selected : EditorTheme::chrome) ||
            !SDL_RenderFillRect(renderer, &row) ||
            !EditorTheme::SetColor(renderer, enabled ? EditorTheme::text : EditorTheme::disabled) ||
            !SDL_RenderDebugText(renderer, row.x + 24, row.y + 9, items[i].label)) return false;
        if ((items[i].command == EditorCommand::ToggleWireframe && wireframe) ||
            (items[i].command == EditorCommand::ToggleRoadNames && roadNames) ||
            (items[i].command == EditorCommand::ToggleTrafficLanes && trafficLanes) ||
            (items[i].command == EditorCommand::ToggleFog && fog) ||
            (items[i].command == EditorCommand::ToggleShadows && shadows) ||
            (items[i].command == EditorCommand::ToggleVisibilityStatistics && visibilityStatistics))
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
        int tab = -1;
        for (int i = 0; i < count; ++i)
        {
            if (SDL_PointInRectFloat(&p, &barTabs[i])) { tab = i; break; }
        }
        if (press || tab >= 0)
        {
            openMenu = press && (e.button.button != SDL_BUTTON_LEFT || openMenu == tab) ? -1 : tab;
            barRow = 0;
        }
        return true;
    }
    if (openMenu < 0) return false;
    const auto items = menus[openMenu].items;
    const SDL_FRect popupRect{barTabs[openMenu].x, barHeight,
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

void EditorMenu::LayoutViewportSubmenu(SDL_Rect scene)
{
    const auto& root = rootItems[activeCategory];
    const int count = static_cast<int>(viewportItems[activeCategory].size());
    const int rows = std::min(count, std::max(1, static_cast<int>(scene.h / itemHeight)));
    submenu = {root.x + root.w, root.y, 304, rows * itemHeight};
    if (submenu.x + submenu.w > scene.w) submenu.x = std::max(0.0f, root.x - submenu.w);
    submenu.y = std::clamp(submenu.y, 0.0f, std::max(0.0f, scene.h - submenu.h));
    firstItem = std::clamp(firstItem, std::max(0, activeItem - rows + 1), activeItem);
    firstItem = std::min(firstItem, count - rows);
}

bool EditorMenu::HandleViewportMenu(SDL_Renderer* renderer, SDL_Rect scene, const SDL_Event& event,
                                    std::optional<EditorCommand>& command)
{
    if (popup == Popup::Closed) return false;
    const auto& items = viewportItems[activeCategory];
    const int count = static_cast<int>(items.size());
    const auto stepItem = [&](int step) {
        for (int i = 0; i < count; ++i)
        {
            activeItem = (activeItem + count + step) % count;
            if (items[activeItem].enabled) break;
        }
        LayoutViewportSubmenu(scene);
    };
    switch (event.type)
    {
    case SDL_EVENT_WINDOW_FOCUS_LOST:
    case SDL_EVENT_WINDOW_RESIZED:
        popup = Popup::Closed;
        return false;
    case SDL_EVENT_KEY_DOWN:
        if (event.key.repeat) return true;
        switch (event.key.key)
        {
        case SDLK_ESCAPE: popup = Popup::Closed; break;
        case SDLK_LEFT: popup = Popup::Root; break;
        case SDLK_UP:
        case SDLK_DOWN:
            if (popup == Popup::Root)
            {
                activeCategory = (activeCategory + (event.key.key == SDLK_DOWN ? 1 : 2)) % 3;
                activeItem = firstItem = 0;
                LayoutViewportSubmenu(scene);
            }
            else stepItem(event.key.key == SDLK_DOWN ? 1 : -1);
            break;
        case SDLK_RIGHT: popup = Popup::Submenu; break;
        case SDLK_RETURN:
        case SDLK_KP_ENTER:
            if (popup == Popup::Root) popup = Popup::Submenu;
            else
            {
                if (items[activeItem].enabled) command = items[activeItem].command;
                popup = Popup::Closed;
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
            for (int group = 0; group < 3; ++group)
                if ((!press || event.button.button == SDL_BUTTON_LEFT) && SDL_PointInRectFloat(&at, &rootItems[group]))
                {
                    if (activeCategory != group) { activeCategory = group; activeItem = firstItem = 0; }
                    LayoutViewportSubmenu(scene);
                    popup = Popup::Submenu;
                    return true;
                }
            if (popup == Popup::Submenu && SDL_PointInRectFloat(&at, &submenu))
            {
                activeItem = std::min(firstItem + static_cast<int>((at.y - submenu.y) / itemHeight), count - 1);
                if (press && event.button.button == SDL_BUTTON_LEFT && items[activeItem].enabled)
                    command = items[activeItem].command;
            }
        }
        if (press) popup = Popup::Closed;
        return true;
    }
    case SDL_EVENT_MOUSE_WHEEL:
        if (popup == Popup::Submenu && event.wheel.y != 0) stepItem(event.wheel.y > 0 ? -1 : 1);
        return true;
    case SDL_EVENT_KEY_UP:
    case SDL_EVENT_MOUSE_BUTTON_UP:
        return true;
    default: return false;
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
    if (popup == Popup::Closed) return true;
    for (int group = 0; group < 3; ++group)
    {
        const auto& row = rootItems[group];
        if (!item(row, viewportCategories[group], activeCategory == group) ||
            !SDL_RenderDebugText(renderer, row.x + row.w - 16, row.y + 9, ">")) return false;
    }
    if (popup == Popup::Submenu)
    {
        const auto& items = viewportItems[activeCategory];
        SDL_FRect row{submenu.x, submenu.y, submenu.w, itemHeight};
        const int last = std::min(static_cast<int>(items.size()), firstItem + static_cast<int>(submenu.h / itemHeight));
        for (int i = firstItem; i < last; ++i)
        {
            if (!item(row, items[i].label, i == activeItem, items[i].enabled)) return false;
            row.y += itemHeight;
        }
    }
    return true;
}
#endif
