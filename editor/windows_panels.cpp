#include "windows_panels.hpp"
#include "marker.hpp"
#include "properties_inspector.hpp"
#include "windows_material_editor.hpp"
#include "windows_theme.hpp"
#include "windows_outliner.hpp"
#include "content_browser.hpp"
#include "editor_menu.hpp"
#include "windows_name_dialog.hpp"
#include "edit_history.hpp"
#include "free_camera.hpp"
#include "menu_ids.h"
#include "panel_ids.h"
#include "road_action.hpp"
#include "transform_settings.hpp"
#include <array>
#include <deque>

#include <windows.h>
#include <commctrl.h>
#include <windowsx.h>
#include <algorithm>
#include <cmath>
#include <cwchar>
#include <initializer_list>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace
{
    constexpr int speedSteps = 1000;

    using WindowsNameDialog::Wide;

    HTREEITEM AddTreeItem(HWND tree, HTREEITEM parent, const std::string& label, LPARAM data)
    {
        std::wstring wide = Wide(label);
        TVINSERTSTRUCTW node{};
        node.hParent = parent;
        node.hInsertAfter = TVI_LAST;
        node.item.mask = TVIF_TEXT | TVIF_PARAM;
        node.item.pszText = wide.data();
        node.item.lParam = data;
        return TreeView_InsertItem(tree, &node);
    }

    SIZE DialogSize(HWND dialog, int width, int height)
    {
        RECT rect{0, 0, width, height};
        MapDialogRect(dialog, &rect);
        return {rect.right, rect.bottom};
    }

    void Choices(HWND dialog, int id, std::initializer_list<const wchar_t*> choices)
    {
        for (const wchar_t* text : choices)
        {
            SendDlgItemMessageW(dialog, id, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(text));
        }
    }

    HBITMAP ToolbarBitmap(TransformIcon which, int size)
    {
        using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;
        Surface icon(LoadTransformIcon(which), SDL_DestroySurface);
        if (!icon)
        {
            return nullptr;
        }
        BITMAPINFO info{};
        info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        info.bmiHeader.biWidth = size;
        info.bmiHeader.biHeight = -size;
        info.bmiHeader.biPlanes = 1;
        info.bmiHeader.biBitCount = 32;
        info.bmiHeader.biCompression = BI_RGB;
        void* pixels = nullptr;
        HBITMAP bitmap = CreateDIBSection(nullptr, &info, DIB_RGB_COLORS, &pixels, nullptr, 0);
        if (!bitmap)
        {
            return nullptr;
        }
        Surface target(SDL_CreateSurfaceFrom(size, size, SDL_PIXELFORMAT_BGRA32, pixels, 4 * size), SDL_DestroySurface);
        const auto face = EditorTheme::button;
        if (!target || !SDL_FillSurfaceRect(target.get(), nullptr, SDL_MapSurfaceRGB(target.get(), face.r, face.g, face.b)) ||
            !SDL_BlitSurfaceScaled(icon.get(), nullptr, target.get(), nullptr, SDL_SCALEMODE_LINEAR))
        {
            DeleteObject(bitmap);
            return nullptr;
        }
        return bitmap;
    }

    // Keep the main HWND as the SDL input window, even over the swapchain.
    LRESULT CALLBACK ViewProc(HWND window, UINT message, WPARAM w, LPARAM l)
    {
        if (message == WM_NCHITTEST)
        {
            return HTTRANSPARENT;
        }
        return DefWindowProcW(window, message, w, l);
    }

    void WinError(const char* action)
    {
        SDL_SetError("%s (Windows error %lu)", action, GetLastError());
    }
}

struct WindowsPanels::Impl
{
    SDL_Window* frameWindow = nullptr;
    HWND frame = nullptr, view = nullptr, toolbar = nullptr, models = nullptr, road = nullptr;
    WindowsOutliner outline;
    HACCEL shortcuts = nullptr;
    Uint32 actions = 0, speedEvent = 0, wheelEvent = 0, pressEvent = 0, toolbarEvent = 0;
    Uint32 transformEvent = 0, modelDropEvent = 0;
    std::string draggedModel;
    std::deque<ModelDrop> modelDrops;
    Uint32 materialDropEvent = 0;
    std::deque<MaterialDrop> materialDrops;
    std::optional<TransformFields> inspected;
    std::deque<InspectorEdit> pendingEdits;
    std::optional<SDL_Keymod> pressModifiers;
    ContentBrowser* browser = nullptr;
    WindowsMaterialEditor materialEditor;
    std::string pressedTexture;
    POINT texturePress{};
    bool draggingTexture = false;
    bool populating = false, populated = false, texturesPopulated = false;
    std::set<std::string> collapsedTextureFolders;
    int scroll = 0;
    double lastSpeed = -1;
    std::array<HBITMAP, transformIcons.size()> icons{};
    int iconSize = 0;
    TransformSettings displayedSettings;

    ~Impl()
    {
        for (HBITMAP icon : icons)
        {
            DeleteObject(icon);
        }
    }

    void ShowButton(int id, TransformIcon which, const wchar_t* caption, const wchar_t* fallback)
    {
        HWND button = GetDlgItem(toolbar, id);
        HBITMAP icon = icons[static_cast<int>(which)];
        LONG_PTR style = GetWindowLongPtrW(button, GWL_STYLE);
        SetWindowLongPtrW(button, GWL_STYLE, icon ? style | BS_BITMAP : style & ~BS_BITMAP);
        SetWindowTextW(button, icon ? caption : fallback);
        SendMessageW(button, BM_SETIMAGE, IMAGE_BITMAP, reinterpret_cast<LPARAM>(icon));
        InvalidateRect(button, nullptr, TRUE);
    }

    void ShowSettings(const TransformSettings& settings)
    {
        displayedSettings = settings;
        const bool world = settings.space == GizmoSpace::World;
        ShowButton(IDC_TRANSFORM_SPACE, settings.SpaceIcon(), world ? L"World space" : L"Local space", world ? L"World" : L"Local");
        ShowButton(IDC_SNAP_TO_GRID, settings.SnapIcon(), settings.snapEnabled ? L"Grid snap on" : L"Grid snap off",
                   settings.snapEnabled ? L"On" : L"Off");
        SetDlgItemTextW(toolbar, IDC_SNAP_STEP, Wide(settings.StepLabel()).c_str());
    }

    void SizeTransformButtons(int side, int spaceX, int y)
    {
        const int gap = DialogSize(toolbar, 3, 0).cx;
        const int snapX = spaceX - side - gap;
        MoveWindow(GetDlgItem(toolbar, IDC_TRANSFORM_SPACE), spaceX, y, side, side, FALSE);
        MoveWindow(GetDlgItem(toolbar, IDC_SNAP_TO_GRID), snapX, y, side, side, FALSE);
        const SIZE label = DialogSize(toolbar, 34, 12);
        MoveWindow(GetDlgItem(toolbar, IDC_SNAP_STEP), snapX - gap - label.cx, DialogSize(toolbar, 0, 7).cy,
                   label.cx, label.cy, FALSE);
        const int size = std::max(1, side - 2 * y);
        if (size == iconSize)
        {
            return;
        }
        iconSize = size;
        for (int id : {IDC_TRANSFORM_SPACE, IDC_SNAP_TO_GRID})
        {
            SendDlgItemMessageW(toolbar, id, BM_SETIMAGE, IMAGE_BITMAP, 0);
        }
        for (TransformIcon which : transformIcons)
        {
            HBITMAP& icon = icons[static_cast<int>(which)];
            DeleteObject(icon);
            icon = ToolbarBitmap(which, size);
        }
        ShowSettings(displayedSettings);
    }

    struct Inspector
    {
        RoadStyle style;
        bool plan, drawing, draft, canFinish;
        int selected, point;
        double elevation;
        unsigned materialRevision;
        PropertiesInspector properties;
        bool operator==(const Inspector&) const = default;
    };
    std::optional<Inspector> lastInspector;

    static_assert(IDC_GEO_LOCATION - IDC_LOC_X == TransformFields::geographicLocation);

    void SetGeographicLayout(bool geographic)
    {
        const auto show = [&](int id, bool visible)
        {
            ShowWindow(GetDlgItem(road, id), visible ? SW_SHOWNA : SW_HIDE);
        };
        // Offset the existing rows in dialog units. Their client rectangles already
        // include the current inspector scroll offset.
        const auto shiftRow = [&](int id, int units)
        {
            const HWND control = GetDlgItem(road, id);
            RECT rect{};
            GetWindowRect(control, &rect);
            MapWindowPoints(nullptr, road, reinterpret_cast<POINT*>(&rect), 2);
            const int dy = DialogSize(road, 0, units).cy;
            SetWindowPos(control, nullptr, rect.left, rect.top + (geographic ? dy : -dy), 0, 0,
                         SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);
        };
        for (int id : {IDC_AXIS_X, IDC_AXIS_Y, IDC_AXIS_Z}) shiftRow(id, 25);
        for (int id : {IDC_ROTATION_LABEL, IDC_ROT_X, IDC_ROT_Y, IDC_ROT_Z}) shiftRow(id, 7);
        for (int id : {IDC_SCALE_LABEL, IDC_SCALE_X, IDC_SCALE_Y, IDC_SCALE_Z}) shiftRow(id, 7);

        // Elevation is the same world-Y field in either layout.
        RECT height = geographic ? RECT{138,24,193,38} : RECT{108,24,149,38};
        MapDialogRect(road, &height);
        MoveWindow(GetDlgItem(road, IDC_LOC_Y), height.left, height.top - scroll,
                   height.right - height.left, height.bottom - height.top, TRUE);
        for (int id : {IDC_POSITION_LABEL, IDC_LOC_X, IDC_LOC_Z}) show(id, !geographic);
        for (int id : {IDC_COORDINATES_LABEL, IDC_ELEVATION_LABEL, IDC_GEO_LOCATION}) show(id, geographic);
        RedrawWindow(road, nullptr, nullptr, RDW_INVALIDATE | RDW_ERASE | RDW_ALLCHILDREN);
    }

    static LRESULT CALLBACK GeographicEditProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR)
    {
        if (msg == WM_PASTE)
        {
            char* text = SDL_GetClipboardText();
            if (text && std::string_view(text).find(',') != std::string_view::npos)
                SendMessageW(hwnd, EM_SETSEL, 0, -1);
            SDL_free(text);
        }
        if (msg == WM_NCDESTROY) RemoveWindowSubclass(hwnd, GeographicEditProc, id);
        return DefSubclassProc(hwnd, msg, w, l);
    }

    void RestoreField(int id)
    {
        if (id == IDC_MARKER_TEXT && lastInspector && lastInspector->properties.page == PropertiesPage::Marker)
        {
            SetDlgItemTextW(road, id, Wide(lastInspector->properties.markerText).c_str());
            SendDlgItemMessageW(road, id, EM_SETMODIFY, FALSE, 0);
            return;
        }
        const int field = id - IDC_LOC_X;
        if (inspected && field >= 0 && field <= TransformFields::geographicLocation)
        {
            SetDlgItemTextW(road, id, Wide(inspected->Text(field)).c_str());
            SendDlgItemMessageW(road, id, EM_SETMODIFY, FALSE, 0);
        }
    }

    void SyncFields(const TransformFields& fields)
    {
        if (inspected && *inspected == fields) return;
        const bool changedTarget = !inspected || inspected->target != fields.target;
        inspected = fields;
        for (int field = 0; field <= TransformFields::geographicLocation; ++field)
        {
            HWND control = GetDlgItem(road, IDC_LOC_X + field);
            EnableWindow(control, fields.CanEdit(field));
            if (changedTarget || control != GetFocus()) RestoreField(IDC_LOC_X + field);
        }
    }

    void SubmitField(HWND control, int field)
    {
        const bool marker = field == InspectorEdit::markerText && lastInspector && lastInspector->properties.page == PropertiesPage::Marker;
        if (inspected && (marker || inspected->CanEdit(field)) && SendMessageW(control, EM_GETMODIFY, 0, 0))
        {
            std::wstring text(GetWindowTextLengthW(control) + 1, L'\0');
            const int length = GetWindowTextW(control, text.data(), static_cast<int>(text.size()));
            text.resize(length);
            auto input = WindowsNameDialog::Utf8(text);
            pendingEdits.push_back({inspected->target, field, std::move(input)});
            SDL_Event event{};
            event.type = transformEvent;
            event.user.windowID = SDL_GetWindowID(frameWindow);
            if (!SDL_PushEvent(&event)) pendingEdits.pop_back();
        }
        SetFocus(frame);
    }

    bool Post(Uint32 type, int code) const
    {
        SDL_Event event{};
        event.type = type;
        event.user.windowID = SDL_GetWindowID(frameWindow);
        event.user.code = code;
        if (!SDL_PushEvent(&event))
        {
            SDL_Log("Editor control: %s", SDL_GetError());
            return false;
        }
        return true;
    }

    void Layout()
    {
        RECT rect{};
        GetClientRect(frame, &rect);
        if (!rect.right || !rect.bottom)
        {
            return;
        }
        const int top = std::min<int>(DialogSize(toolbar, 0, TOOLBAR_HEIGHT).cy, rect.bottom);
        const int left = DialogSize(models, BROWSER_WIDTH, 0).cx, right = DialogSize(road, ROAD_PANEL_WIDTH, 0).cx;
        const int h = rect.bottom - top;
        SetWindowPos(toolbar, nullptr, 0, 0, rect.right, top, SWP_NOZORDER | SWP_NOACTIVATE | SWP_NOCOPYBITS);
        MoveWindow(models, 0, top, left, h, TRUE);
        const int outlineHeight = std::min<int>(h / 3, DialogSize(road, 0, 140).cy);
        outline.Layout(rect.right - right, top, right, outlineHeight);
        MoveWindow(road, rect.right - right, top + outlineHeight, right, h - outlineHeight, TRUE);
        MoveWindow(view, left, top, std::max<int>(1, rect.right - left - right), std::max(1, h), TRUE);

        auto moveTool = [&](int id, int rightOffset, int y, int w, int height)
        {
            const SIZE offset = DialogSize(toolbar, rightOffset, y), size = DialogSize(toolbar, w, height);
            MoveWindow(GetDlgItem(toolbar, id), rect.right - offset.cx, offset.cy, size.cx, size.cy, FALSE);
        };
        const int insetY = DialogSize(toolbar, 0, 2).cy;
        const int side = std::max(1, top - 2 * insetY);
        moveTool(IDC_SPEED_LABEL, 200, 7, 52, 12);
        moveTool(IDC_SPEED, 146, 3, 89, 19);
        moveTool(IDC_SPEED_VALUE, 54, 7, 50, 12);
        SizeTransformButtons(side, rect.right - DialogSize(toolbar, 204, 0).cx - side, insetY);
        // The controls have moved without repainting. Erase their old footprints and
        // paint all six children at their final positions, even when the icon size is unchanged.
        RedrawWindow(toolbar, nullptr, nullptr, RDW_INVALIDATE | RDW_ERASE | RDW_ALLCHILDREN | RDW_UPDATENOW);

        LayoutBrowserLists();

        LayoutInspectorScroll();
        // Repaint children after their final positions and scrollbar ranges are known.
        for (HWND panel : {models, outline.Window(), road})
        {
            RedrawWindow(panel, nullptr, nullptr, RDW_FRAME | RDW_INVALIDATE | RDW_ERASE | RDW_ALLCHILDREN | RDW_UPDATENOW);
        }
    }

    void LayoutInspectorScroll()
    {
        RECT client{};
        GetClientRect(road, &client);
        const auto page = lastInspector ? lastInspector->properties.page : PropertiesPage::Road;
        const int height = page == PropertiesPage::Road ? ROAD_PANEL_HEIGHT :
                           (page == PropertiesPage::Intersection || page == PropertiesPage::Terrain || page == PropertiesPage::Marker ? 192 : 150);
        SCROLLINFO si{sizeof(si), SIF_RANGE | SIF_PAGE | SIF_POS};
        si.nMax = DialogSize(road, 0, height).cy - 1;
        si.nPage = static_cast<UINT>(client.bottom);
        si.nPos = scroll;
        ScrollTo(SetScrollInfo(road, SB_VERT, &si, TRUE));
    }

    void ScrollTo(int pos)
    {
        SCROLLINFO si{sizeof(si), SIF_POS};
        si.nPos = pos;
        const int next = SetScrollInfo(road, SB_VERT, &si, TRUE);
        if (next != scroll)
        {
            ScrollWindowEx(road, 0, scroll - next, nullptr, nullptr, nullptr, nullptr,
                           SW_SCROLLCHILDREN | SW_INVALIDATE | SW_ERASE);
            scroll = next;
        }
    }

    void LayoutBrowserLists()
    {
        RECT panel{};
        GetClientRect(models, &panel);
        const SIZE inset = DialogSize(models, 7, 6);
        const int width = std::max<int>(1, panel.right - 2 * inset.cx);
        const int bottom = panel.bottom - inset.cy;
        const int modelTop = DialogSize(models, 0, BROWSER_MODEL_LIST_TOP).cy;
        MoveWindow(GetDlgItem(models, IDC_MODELS), inset.cx, modelTop, width, std::max(1, bottom - modelTop), TRUE);

        const int top = DialogSize(models, 0, BROWSER_PAGE_TOP).cy;
        const int treeHeight = std::clamp((bottom - top) / 3, 1, static_cast<int>(DialogSize(models, 0, 92).cy));
        MoveWindow(GetDlgItem(models, IDC_TEXTURE_FOLDERS), inset.cx, top, width, treeHeight, TRUE);
        int listTop = top + treeHeight + inset.cy;
        const int statusHeight = DialogSize(models, 0, 15).cy;
        MoveWindow(GetDlgItem(models, IDC_TEXTURE_STATUS), inset.cx, listTop, width, statusHeight, TRUE);
        if (!browser->Textures().Status().empty()) listTop += statusHeight;
        MoveWindow(GetDlgItem(models, IDC_TEXTURES), inset.cx, listTop, width, std::max(1, bottom - listTop), TRUE);
        SendDlgItemMessageW(models, IDC_TEXTURES, LB_SETITEMHEIGHT, 0, DialogSize(models, 0, 38).cy);
    }

    void PopulateTextureFolders()
    {
        const auto& catalog = browser->Textures();
        HWND tree = GetDlgItem(models, IDC_TEXTURE_FOLDERS);
        populating = true;
        SendMessageW(tree, WM_SETREDRAW, FALSE, 0);
        TreeView_DeleteAllItems(tree);
        std::map<std::string, HTREEITEM> nodes;
        for (std::size_t i = 0; i < catalog.Folders().size(); ++i)
        {
            const auto& folder = catalog.Folders()[i];
            const auto slash = folder.rfind('/');
            const auto parent = slash == std::string::npos ? std::string{} : folder.substr(0, slash);
            const auto label = folder.empty() ? "Textures" : folder.substr(slash == std::string::npos ? 0 : slash + 1);
            nodes[folder] = AddTreeItem(tree, folder.empty() ? TVI_ROOT : nodes.at(parent), label, i);
        }
        for (const auto& [path, node] : nodes)
            if (!collapsedTextureFolders.contains(path)) TreeView_Expand(tree, node, TVE_EXPAND);
        if (auto found = nodes.find(catalog.Folder()); found != nodes.end())
        {
            TreeView_SelectItem(tree, found->second);
            TreeView_EnsureVisible(tree, found->second);
        }
        SendMessageW(tree, WM_SETREDRAW, TRUE, 0);
        InvalidateRect(tree, nullptr, TRUE);
        populating = false;
    }

    void PopulateTextures(const std::string& selected = {}, bool keepScroll = false)
    {
        HWND list = GetDlgItem(models, IDC_TEXTURES);
        const LRESULT top = keepScroll ? SendMessageW(list, LB_GETTOPINDEX, 0, 0) : 0;
        SendMessageW(list, WM_SETREDRAW, FALSE, 0);
        SendMessageW(list, LB_RESETCONTENT, 0, 0);
        const auto& catalog = browser->Textures();
        for (std::size_t i = 0; i < catalog.Items().size(); ++i)
        {
            const auto& item = catalog.Items()[i];
            const auto path = Wide(item.path);
            const LRESULT row = SendMessageW(list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(path.c_str()));
            if (row >= 0)
            {
                SendMessageW(list, LB_SETITEMDATA, row, i);
                if (item.path == selected)
                {
                    SendMessageW(list, LB_SETCURSEL, row, 0);
                }
            }
        }
        if (top > 0)
        {
            SendMessageW(list, LB_SETTOPINDEX, top, 0);
        }
        SendMessageW(list, WM_SETREDRAW, TRUE, 0);
        InvalidateRect(list, nullptr, TRUE);
        SetDlgItemTextW(models, IDC_TEXTURE_STATUS, Wide(catalog.Status()).c_str());
        texturesPopulated = true;
    }

    void ShowBrowserTab()
    {
        const bool showModels = TabCtrl_GetCurSel(GetDlgItem(models, IDC_BROWSER_TABS)) == 0;
        browser->SetTab(showModels ? ContentBrowser::Tab::Models : ContentBrowser::Tab::Textures);
        for (int id : {IDC_PREVIEW, IDC_PREVIEW_STATUS, IDC_MODELS})
        {
            ShowWindow(GetDlgItem(models, id), showModels ? SW_SHOWNA : SW_HIDE);
        }
        ShowWindow(GetDlgItem(models, IDC_TEXTURE_FOLDERS), showModels ? SW_HIDE : SW_SHOWNA);
        if (!showModels && !texturesPopulated)
        {
            PopulateTextureFolders();
            PopulateTextures();
        }
        const bool hasStatus = !showModels && !browser->Textures().Status().empty();
        ShowWindow(GetDlgItem(models, IDC_TEXTURE_STATUS), hasStatus ? SW_SHOWNA : SW_HIDE);
        ShowWindow(GetDlgItem(models, IDC_TEXTURES), showModels ? SW_HIDE : SW_SHOWNA);
        LayoutBrowserLists();
    }

    void RefreshBrowser()
    {
        if (browser->ActiveTab() == ContentBrowser::Tab::Models)
        {
            browser->Refresh();
            Populate();
            return;
        }
        HWND list = GetDlgItem(models, IDC_TEXTURES);
        const LRESULT row = SendMessageW(list, LB_GETCURSEL, 0, 0);
        const LRESULT index = SendMessageW(list, LB_GETITEMDATA, row, 0);
        const auto& items = browser->Textures().Items();
        const std::string selected = static_cast<std::size_t>(index) < items.size() ? items[index].path : std::string{};
        browser->Refresh();
        PopulateTextureFolders();
        PopulateTextures(selected, true);
        materialEditor.RefreshImages();
        ShowBrowserTab();
    }

    bool TextureContextMenu(HWND source, POINT at)
    {
        if (browser->ActiveTab() != ContentBrowser::Tab::Textures) return false;
        HWND list = GetDlgItem(models, IDC_TEXTURES);
        HWND tree = GetDlgItem(models, IDC_TEXTURE_FOLDERS);
        POINT local = at;
        ScreenToClient(source, &local);
        if (source == list)
        {
            RECT client{}, row{};
            GetClientRect(list, &client);
            if (!PtInRect(&client, local)) return false;
            const auto hit = SendMessageW(list, LB_ITEMFROMPOINT, 0, MAKELPARAM(local.x, local.y));
            const int index = LOWORD(hit);
            if (index < SendMessageW(list, LB_GETCOUNT, 0, 0) &&
                SendMessageW(list, LB_GETITEMRECT, index, reinterpret_cast<LPARAM>(&row)) != LB_ERR &&
                PtInRect(&row, local)) return false;
        }
        else if (source == tree)
        {
            TVHITTESTINFO hit{};
            hit.pt = local;
            if (TreeView_HitTest(tree, &hit)) return false;
        }
        else if (source != models || local.y < DialogSize(models, 0, BROWSER_PAGE_TOP).cy) return false;

        if (EditorMenu::OpenMaterialMenu(frameWindow, at.x, at.y))
        {
            auto& catalog = browser->Textures();
            std::string name = catalog.NewMaterialName();
            while (auto input = WindowsNameDialog::Prompt(frame, L"Add New Material", name))
            {
                name = *input;
                std::string created, error;
                if (catalog.CreateMaterial(name, created, error))
                {
                    PopulateTextures(created);
                    ShowBrowserTab();
                    SetFocus(list);
                    break;
                }
                MessageBoxW(frame, Wide(error).c_str(), L"Add New Material", MB_OK | MB_ICONWARNING);
            }
        }
        return true;
    }

    void DrawTexture(const DRAWITEMSTRUCT& draw)
    {
        const bool selected = (draw.itemState & ODS_SELECTED) != 0;
        FillRect(draw.hDC, &draw.rcItem, WindowsTheme::Brush(selected ? EditorTheme::selected : EditorTheme::list));
        auto& catalog = browser->Textures();
        if (draw.itemID != UINT(-1) && draw.itemData < catalog.Items().size())
        {
            const auto& item = catalog.Items()[draw.itemData];
            const int pad = std::max<int>(2, DialogSize(models, 3, 0).cx);
            RECT icon = draw.rcItem;
            InflateRect(&icon, -pad, -pad);
            icon.right = icon.left + icon.bottom - icon.top;
            SDL_Surface* surface = item.material ? nullptr : catalog.Thumbnail(draw.itemData);
            WindowsTheme::DrawPreview(draw.hDC, icon, surface);
            SetBkMode(draw.hDC, TRANSPARENT);
            if (!surface)
            {
                SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::text));
                DrawTextW(draw.hDC, item.material ? L"M" : L"?", -1, &icon, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            }
            RECT label{icon.right + pad, icon.top, draw.rcItem.right - pad, icon.bottom};
            if (item.material)
            {
                label.bottom = (icon.top + icon.bottom) / 2;
            }
            constexpr UINT flags = DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX;
            SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::text));
            DrawTextW(draw.hDC, Wide(item.name).c_str(), -1, &label, flags);
            if (item.material)
            {
                label.top = label.bottom;
                label.bottom = icon.bottom;
                SetTextColor(draw.hDC, WindowsTheme::Color(selected ? EditorTheme::text : EditorTheme::secondary));
                DrawTextW(draw.hDC, L"Material", -1, &label, flags);
            }
        }
        if ((draw.itemState & (ODS_FOCUS | ODS_NOFOCUSRECT)) == ODS_FOCUS)
        {
            DrawFocusRect(draw.hDC, &draw.rcItem);
        }
    }

    void Populate()
    {
        populated = true;
        populating = true;
        HWND tree = GetDlgItem(models, IDC_MODELS);
        SendMessageW(tree, WM_SETREDRAW, FALSE, 0);
        TreeView_DeleteAllItems(tree);
        std::map<std::string, HTREEITEM> folders;
        HTREEITEM selected = nullptr;
        const auto& rows = browser->Rows();
        for (std::size_t i = 0; i < rows.size(); ++i)
        {
            const auto& row = rows[i];
            if (row.modelPath.empty())
            {
                continue;
            }
            HTREEITEM parent = TVI_ROOT;
            std::size_t start = 0, slash;
            while ((slash = row.modelPath.find('/', start)) != std::string::npos)
            {
                auto [it, created] = folders.try_emplace(row.modelPath.substr(0, slash));
                if (created)
                {
                    it->second = AddTreeItem(tree, parent, row.modelPath.substr(start, slash - start), 0);
                }
                parent = it->second;
                start = slash + 1;
            }
            const HTREEITEM item = AddTreeItem(tree, parent, row.label, static_cast<LPARAM>(i + 1));
            if (row.modelPath == browser->Selected())
            {
                selected = item;
            }
        }
        for (const auto& [path, item] : folders)
        {
            TreeView_Expand(tree, item, TVE_EXPAND);
        }
        if (selected)
        {
            TreeView_SelectItem(tree, selected);
            TreeView_EnsureVisible(tree, selected);
        }
        SendMessageW(tree, WM_SETREDRAW, TRUE, 0);
        InvalidateRect(tree, nullptr, TRUE);
        populating = false;
    }

    void OpenSelectedMaterial()
    {
        HWND list = GetDlgItem(models, IDC_TEXTURES);
        const auto row = SendMessageW(list, LB_GETCURSEL, 0, 0);
        const auto index = static_cast<std::size_t>(SendMessageW(list, LB_GETITEMDATA, row, 0));
        auto& catalog = browser->Textures();
        if (index < catalog.Items().size() && catalog.Items()[index].material &&
            !materialEditor.Open(frame, catalog, catalog.Items()[index].path))
            MessageBoxW(frame, Wide(SDL_GetError()).c_str(), L"Material Editor", MB_OK | MB_ICONWARNING);
    }

    void DrawRoadMaterial(const DRAWITEMSTRUCT& draw)
    {
        const RECT r = draw.rcItem;
        FillRect(draw.hDC, &r, WindowsTheme::Brush(EditorTheme::list));
        const int pad = DialogSize(road, 3, 0).cx;
        const RECT thumb{r.left + pad, r.top + pad, r.left + r.bottom - r.top - pad, r.bottom - pad};
        const bool active = lastInspector && lastInspector->properties.materialSlot;
        const auto path = active ? lastInspector->properties.material : std::string{};
        WindowsTheme::DrawPreview(draw.hDC, thumb, active ? browser->RoadMaterialThumbnail(path) : nullptr);
        SetBkMode(draw.hDC, TRANSPARENT);
        RECT line{thumb.right + pad, r.top, r.right - pad, (r.top + r.bottom) / 2};
        constexpr UINT flags = DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX;
        SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::secondary));
        const bool terrain = lastInspector && lastInspector->properties.page == PropertiesPage::Terrain;
        DrawTextW(draw.hDC, terrain ? L"Terrain material" : L"Pavement material", -1, &line, flags);
        line.top = line.bottom;
        line.bottom = r.bottom;
        SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::text));
        const auto label = !active ? L"Select a road" : path.empty() ? L"Mesh colors" : Wide(path.substr(path.rfind('/') + 1));
        DrawTextW(draw.hDC, label.c_str(), -1, &line, flags);
    }

    void CancelTextureDrag(HWND list)
    {
        pressedTexture.clear();
        draggingTexture = false;
        materialEditor.Hover({});
        if (GetCapture() == list) ReleaseCapture();
    }

    static LRESULT CALLBACK TextureListProc(HWND list, UINT msg, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR data)
    {
        auto& self = *reinterpret_cast<Impl*>(data);
        if (msg == WM_NCDESTROY)
            RemoveWindowSubclass(list, TextureListProc, id);
        if (msg == WM_KEYDOWN && w == VK_RETURN)
        {
            self.OpenSelectedMaterial();
            return 0;
        }
        if (msg == WM_LBUTTONDOWN)
        {
            const auto hit = SendMessageW(list, LB_ITEMFROMPOINT, 0, l);
            const auto row = LOWORD(hit);
            RECT r{};
            POINT p{GET_X_LPARAM(l), GET_Y_LPARAM(l)};
            const auto index = static_cast<std::size_t>(SendMessageW(list, LB_GETITEMDATA, row, 0));
            const auto& items = self.browser->Textures().Items();
            if (!HIWORD(hit) && index < items.size() && (items[index].material || self.materialEditor.Window()) &&
                SendMessageW(list, LB_GETITEMRECT, row, reinterpret_cast<LPARAM>(&r)) != LB_ERR && PtInRect(&r, p))
            {
                SetFocus(list);
                SendMessageW(list, LB_SETCURSEL, row, 0);
                self.pressedTexture = items[index].path;
                self.texturePress = p;
                SetCapture(list);
                return 0;
            }
        }
        if (!self.pressedTexture.empty())
        {
            if (msg == WM_MOUSEMOVE || msg == WM_LBUTTONUP)
            {
                POINT point{GET_X_LPARAM(l), GET_Y_LPARAM(l)};
                if (std::abs(point.x - self.texturePress.x) >= GetSystemMetrics(SM_CXDRAG) ||
                    std::abs(point.y - self.texturePress.y) >= GetSystemMetrics(SM_CYDRAG)) self.draggingTexture = true;
                ClientToScreen(list, &point);
                const bool material = Material::ValidAssetPath(self.pressedTexture);
                const auto slot = material ? std::nullopt : self.materialEditor.SlotAt(point);
                const bool inspector = material && self.lastInspector &&
                    self.lastInspector->properties.materialSlot &&
                    WindowFromPoint(point) == GetDlgItem(self.road, IDC_ROAD_MATERIAL);
                const auto viewport = material ? self.ViewportDropPoint(point) : std::nullopt;
                if (msg == WM_LBUTTONUP)
                {
                    const auto path = std::move(self.pressedTexture);
                    const bool drop = self.draggingTexture && (slot || inspector || viewport);
                    self.CancelTextureDrag(list);
                    if (drop && material)
                    {
                        const auto target = inspector && self.inspected ? std::optional{self.inspected->target} : std::nullopt;
                        self.materialDrops.push_back({path, viewport, target});
                        if (!self.Post(self.materialDropEvent, 0)) self.materialDrops.pop_back();
                    }
                    else if (drop) self.materialEditor.Drop(*slot, path);
                }
                else if (self.draggingTexture)
                {
                    self.materialEditor.Hover(slot);
                    SetCursor(LoadCursorW(nullptr, (slot || inspector || viewport) ? IDC_CROSS : IDC_NO));
                }
                return 0;
            }
            if (msg == WM_CAPTURECHANGED || msg == WM_CANCELMODE || msg == WM_RBUTTONDOWN ||
                (msg == WM_KEYDOWN && w == VK_ESCAPE))
            {
                self.CancelTextureDrag(list);
                return 0;
            }
        }
        return DefSubclassProc(list, msg, w, l);
    }

    void Control(int id, int notification)
    {
        if (id == IDC_TEXTURES && notification == LBN_DBLCLK)
        {
            OpenSelectedMaterial();
            return;
        }
        if (notification == BN_CLICKED)
        {
            switch (id)
            {
            case IDC_USE_GEOGRAPHIC:
                SetGeographicLayout(IsDlgButtonChecked(road, id) == BST_CHECKED);
                return;
            case IDC_TRANSFORM_SPACE:
            case IDC_SNAP_TO_GRID: Post(toolbarEvent, id); SetFocus(frame); return;
            case IDC_REFRESH: RefreshBrowser(); return;
            case IDC_VIEW: Post(actions, int(RoadAction::ChangeView)); SetFocus(frame); return;
            case IDC_NEW: Post(actions, int(RoadAction::New)); SetFocus(frame); return;
            case IDC_FINISH: Post(actions, int(RoadAction::Finish)); SetFocus(frame); return;
            case IDC_CANCEL_DRAFT: Post(actions, int(RoadAction::Cancel)); SetFocus(frame); return;
            case IDC_RADIUS_LESS: Post(actions, int(RoadAction::SmallerRadius)); return;
            case IDC_RADIUS_MORE: Post(actions, int(RoadAction::LargerRadius)); return;
            case IDC_STOP_LINE: Post(actions, int(RoadAction::ToggleStopLine)); return;
            case IDC_INVINCIBLE: Post(actions, int(RoadAction::ToggleInvincible)); return;
            case IDC_FEWER: Post(actions, int(RoadAction::Fewer)); return;
            case IDC_MORE: Post(actions, int(RoadAction::More)); return;
            case IDC_DIVIDED: Post(actions, int(RoadAction::Divide)); return;
            case IDC_GAP_LESS: Post(actions, int(RoadAction::NarrowerGap)); return;
            case IDC_GAP_MORE: Post(actions, int(RoadAction::WiderGap)); return;
            case IDC_LEFT: Post(actions, int(RoadAction::Left)); return;
            case IDC_RIGHT: Post(actions, int(RoadAction::Right)); return;
            }
        }
        if (notification == CBN_SELCHANGE)
        {
            const LRESULT i = SendDlgItemMessageW(road, id, CB_GETCURSEL, 0, 0);
            static constexpr RoadAction alignment[] = {RoadAction::Straight, RoadAction::Curve};
            static constexpr RoadAction layout[] = {RoadAction::TwoNorthbound, RoadAction::TwoSouthbound, RoadAction::CenterTurn};
            static constexpr RoadAction edge[] = {RoadAction::NoEdge, RoadAction::Transition, RoadAction::Curb};
            if (id == IDC_ALIGNMENT && i >= 0 && i < 2)
            {
                Post(actions, int(alignment[i]));
            }
            if (id == IDC_LAYOUT && i >= 0 && i < 3)
            {
                Post(actions, int(layout[i]));
            }
            if (id == IDC_MEDIAN && (i == 0 || i == 1))
            {
                Post(actions, int(i == 0 ? RoadAction::ConcreteMedian : RoadAction::GrassMedian));
            }
            if (id == IDC_EDGE && i >= 0 && i < 3)
            {
                Post(actions, int(edge[i]));
            }
        }
    }

    void UpdateInspector(const Inspector& data)
    {
        if (lastInspector && *lastInspector == data)
        {
            return;
        }

        const auto& properties = data.properties;
        const bool pageChanged = !lastInspector || lastInspector->properties.page != properties.page;
        const bool marker = properties.page == PropertiesPage::Marker;
        const bool refreshText = marker && (pageChanged || lastInspector->properties.title != properties.title ||
                                             lastInspector->properties.markerText != properties.markerText);
        lastInspector = data;
        const bool roadPage = properties.page == PropertiesPage::Road;
        const bool junction = properties.page == PropertiesPage::Intersection;
        if (pageChanged)
        {
            ScrollTo(0);
            LayoutInspectorScroll();
            for (int id : {IDC_VIEW, IDC_NEW, IDC_ALIGNMENT, IDC_FEWER, IDC_LANE_COUNT, IDC_MORE, IDC_LAYOUT,
                           IDC_LEFT, IDC_RIGHT, IDC_EDGE, IDC_WIDTH, IDC_POINT, IDC_FINISH, IDC_CANCEL_DRAFT,
                           IDC_LAYOUT_LABEL, IDC_LANE_DETAIL, IDC_DIVIDED, IDC_GAP_LESS, IDC_GAP_VALUE,
                           IDC_GAP_MORE, IDC_MEDIAN, IDC_CAPTION_ALIGN, IDC_CAPTION_LANES, IDC_CAPTION_SHOULDERS,
                           IDC_CAPTION_EDGES, IDC_CAPTION_GAP, IDC_CAPTION_MEDIAN})
                ShowWindow(GetDlgItem(road, id), roadPage ? SW_SHOWNA : SW_HIDE);
            for (int id : {IDC_RADIUS_LESS, IDC_RADIUS_VALUE, IDC_RADIUS_MORE})
                ShowWindow(GetDlgItem(road, id), junction ? SW_SHOWNA : SW_HIDE);
            ShowWindow(GetDlgItem(road, IDC_STOP_LINE), properties.page == PropertiesPage::Anchor ? SW_SHOWNA : SW_HIDE);
            ShowWindow(GetDlgItem(road, IDC_INVINCIBLE), (properties.page == PropertiesPage::Object || marker) ? SW_SHOWNA : SW_HIDE);
            ShowWindow(GetDlgItem(road, IDC_MARKER_TEXT_LABEL), marker ? SW_SHOWNA : SW_HIDE);
            ShowWindow(GetDlgItem(road, IDC_MARKER_TEXT), marker ? SW_SHOWNA : SW_HIDE);
            ShowWindow(GetDlgItem(road, IDC_PROPERTY_DETAIL), !roadPage && properties.page != PropertiesPage::Other ? SW_SHOWNA : SW_HIDE);
            ShowWindow(GetDlgItem(road, IDC_ROAD_MATERIAL), roadPage || properties.materialSlot ? SW_SHOWNA : SW_HIDE);
        }
        if (refreshText) RestoreField(IDC_MARKER_TEXT);
        SetDlgItemTextW(road, IDC_ROAD_TITLE, Wide(properties.title).c_str());
        InvalidateRect(GetDlgItem(road, IDC_ROAD_MATERIAL), nullptr, FALSE);
        const int lanes = data.style.GetLaneCount();
        if (!roadPage)
        {
            SetDlgItemTextW(road, IDC_PROPERTY_DETAIL, junction ? L"Corner radius (m)" : Wide(properties.detail).c_str());
            if (junction)
            {
                wchar_t radius[48];
                swprintf(radius, 48, L"%.2f", properties.radius);
                SetDlgItemTextW(road, IDC_RADIUS_VALUE, radius);
                EnableWindow(GetDlgItem(road, IDC_RADIUS_LESS), properties.radius > 0);
            }
            CheckDlgButton(road, IDC_STOP_LINE, properties.stopLine ? BST_CHECKED : BST_UNCHECKED);
            CheckDlgButton(road, IDC_INVINCIBLE, properties.invincible ? BST_CHECKED : BST_UNCHECKED);
            return;
        }
        SetDlgItemTextW(road, IDC_VIEW, data.plan ? L"Back to 3D  [R]" : L"Top-down view  [R]");
        SetDlgItemTextW(road, IDC_NEW, data.draft && !data.drawing ? L"Resume draft" : L"New road");
        EnableWindow(GetDlgItem(road, IDC_NEW), !data.drawing);
        SendDlgItemMessageW(road, IDC_ALIGNMENT, CB_SETCURSEL, data.style.curved, 0);
        SetDlgItemInt(road, IDC_LANE_COUNT, lanes, FALSE);
        EnableWindow(GetDlgItem(road, IDC_FEWER), lanes > (data.style.IsDivided() ? 2 : 1));
        EnableWindow(GetDlgItem(road, IDC_MORE), lanes < RoadStyle::maxLanes);

        for (int id : {IDC_LAYOUT_LABEL, IDC_LAYOUT})
        {
            ShowWindow(GetDlgItem(road, id), lanes == 3 ? SW_SHOWNA : SW_HIDE);
        }

        const bool centerTurn = lanes > 3 && data.style.HasCenterTurnLane();

        if (centerTurn)
        {
            const std::wstring detail = std::to_wstring(lanes / 2) + L" each way + center turn";
            SetDlgItemTextW(road, IDC_LANE_DETAIL, detail.c_str());
        }

        ShowWindow(GetDlgItem(road, IDC_LANE_DETAIL), centerTurn ? SW_SHOWNA : SW_HIDE);
        SendDlgItemMessageW(road, IDC_LAYOUT, CB_SETCURSEL, lanes == 3 ? int(data.style.threeLaneLayout) : -1, 0);
        CheckDlgButton(road, IDC_LEFT, data.style.leftShoulder ? BST_CHECKED : BST_UNCHECKED);
        CheckDlgButton(road, IDC_RIGHT, data.style.rightShoulder ? BST_CHECKED : BST_UNCHECKED);
        SendDlgItemMessageW(road, IDC_EDGE, CB_SETCURSEL, int(data.style.edge), 0);
        wchar_t text[100];
        const bool divided = data.style.IsDivided();
        const bool hasMedian = data.style.HalfMedianWidth() > 0;
        CheckDlgButton(road, IDC_DIVIDED, divided ? BST_CHECKED : BST_UNCHECKED);
        EnableWindow(GetDlgItem(road, IDC_DIVIDED), data.style.CanDivide());
        EnableWindow(GetDlgItem(road, IDC_GAP_LESS), divided && data.style.DividedGap() > RoadStyle::minSeparation);
        EnableWindow(GetDlgItem(road, IDC_GAP_MORE), divided);
        swprintf(text, 100, L"%.2f", data.style.DividedGap());
        SetDlgItemTextW(road, IDC_GAP_VALUE, divided ? text : L"");
        EnableWindow(GetDlgItem(road, IDC_MEDIAN), hasMedian);
        SendDlgItemMessageW(road, IDC_MEDIAN, CB_SETCURSEL, hasMedian ? int(data.style.median) : -1, 0);
        swprintf(text, 100, L"Width: %.2f m", data.style.LeftWidth() + data.style.RightWidth() + 2 * data.style.EdgeWidth());
        SetDlgItemTextW(road, IDC_WIDTH, text);

        if (data.point >= 0)
        {
            swprintf(text, 100, L"Point %d   Y: %.2f m", data.point + 1, data.elevation);
            SetDlgItemTextW(road, IDC_POINT, text);
        }

        ShowWindow(GetDlgItem(road, IDC_POINT), data.point >= 0 ? SW_SHOWNA : SW_HIDE);

        for (int id : {IDC_FINISH, IDC_CANCEL_DRAFT})
        {
            ShowWindow(GetDlgItem(road, id), data.plan ? SW_SHOWNA : SW_HIDE);
        }

        EnableWindow(GetDlgItem(road, IDC_FINISH), data.canFinish);
        EnableWindow(GetDlgItem(road, IDC_CANCEL_DRAFT), data.drawing);
    }

    std::optional<SDL_FPoint> ViewportDropPoint(POINT p) const
    {
        if (!lastInspector || lastInspector->plan) return {};
        const HWND under = WindowFromPoint(p);
        if (under != frame && under != view) return {};
        ScreenToClient(view, &p);
        RECT rect{};
        GetClientRect(view, &rect);
        if (!PtInRect(&rect, p)) return {};
        return SDL_FPoint{float(p.x) / rect.right, float(p.y) / rect.bottom};
    }

    void CancelModelDrag()
    {
        draggedModel.clear();
        if (GetCapture() == models) ReleaseCapture();
        SetCursor(LoadCursorW(nullptr, IDC_ARROW));
    }

    static INT_PTR CALLBACK DialogProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l)
    {
        if (msg == WM_INITDIALOG)
        {
            SetWindowLongPtrW(hwnd, DWLP_USER, l);
            WindowsTheme::ApplyDialog(hwnd, GetDlgItem(hwnd, IDC_SPEED) ? EditorTheme::chrome : EditorTheme::panel);
            return FALSE;
        }

        auto* self = reinterpret_cast<Impl*>(GetWindowLongPtrW(hwnd, DWLP_USER));

        if (!self)
        {
            return FALSE;
        }

        switch (msg)
        {
        case WM_COMMAND:
            if (HIWORD(w) == EN_KILLFOCUS)
            {
                self->RestoreField(LOWORD(w));
                return TRUE;
            }
            if (l)
            {
                self->Control(LOWORD(w), HIWORD(w));
            }
            return TRUE;
        case WM_HSCROLL:
            if (reinterpret_cast<HWND>(l) == GetDlgItem(hwnd, IDC_SPEED))
            {
                self->Post(self->speedEvent, static_cast<int>(SendDlgItemMessageW(hwnd, IDC_SPEED, TBM_GETPOS, 0, 0)));
            }
            return TRUE;
        case WM_MOUSEMOVE:
        case WM_LBUTTONUP:
            if (hwnd == self->models && !self->draggedModel.empty())
            {
                POINT screen{GET_X_LPARAM(l), GET_Y_LPARAM(l)};
                ClientToScreen(hwnd, &screen);
                const auto point = self->ViewportDropPoint(screen);
                if (msg == WM_MOUSEMOVE)
                {
                    SetCursor(LoadCursorW(nullptr, point ? IDC_CROSS : IDC_NO));
                }
                else
                {
                    if (point)
                    {
                        self->modelDrops.push_back({std::move(self->draggedModel), *point});
                        if (!self->Post(self->modelDropEvent, 0)) self->modelDrops.pop_back();
                    }
                    self->CancelModelDrag();
                    if (point) SetFocus(self->frame);
                }
                return TRUE;
            }
            return FALSE;
        case WM_RBUTTONDOWN:
        case WM_CANCELMODE:
        case WM_CAPTURECHANGED:
            if (hwnd == self->models && !self->draggedModel.empty()) self->CancelModelDrag();
            return FALSE;
        case WM_CONTEXTMENU:
            return hwnd == self->models && self->TextureContextMenu(reinterpret_cast<HWND>(w),
                                                                    {GET_X_LPARAM(l), GET_Y_LPARAM(l)});
        case WM_NOTIFY:
        {
            auto* header = reinterpret_cast<NMHDR*>(l);
            if (header->idFrom == IDC_BROWSER_TABS && header->code == TCN_SELCHANGE)
            {
                self->ShowBrowserTab();
                return TRUE;
            }
            if (header->idFrom == IDC_TEXTURE_FOLDERS && header->code == TVN_ITEMEXPANDEDW && !self->populating)
            {
                const auto& change = *reinterpret_cast<NMTREEVIEWW*>(l);
                const auto index = static_cast<std::size_t>(change.itemNew.lParam);
                const auto& folders = self->browser->Textures().Folders();
                if (index < folders.size())
                {
                    if (change.action == TVE_COLLAPSE) self->collapsedTextureFolders.insert(folders[index]);
                    else self->collapsedTextureFolders.erase(folders[index]);
                }
                return TRUE;
            }
            if (header->idFrom == IDC_TEXTURE_FOLDERS && header->code == TVN_SELCHANGEDW && !self->populating)
            {
                auto& catalog = self->browser->Textures();
                const auto index = static_cast<std::size_t>(reinterpret_cast<NMTREEVIEWW*>(l)->itemNew.lParam);
                if (index < catalog.Folders().size())
                {
                    catalog.SelectFolder(catalog.Folders()[index]);
                    self->PopulateTextures();
                    self->ShowBrowserTab();
                }
                return TRUE;
            }
            if (header->idFrom == IDC_MODELS && header->code == TVN_BEGINDRAGW &&
                self->lastInspector && !self->lastInspector->plan)
            {
                const LPARAM item = reinterpret_cast<NMTREEVIEWW*>(l)->itemNew.lParam;
                const auto& rows = self->browser->Rows();
                if (item > 0 && static_cast<std::size_t>(item) <= rows.size())
                {
                    self->draggedModel = rows[item - 1].modelPath;
                    SetCapture(hwnd);
                    SetCursor(LoadCursorW(nullptr, IDC_NO));
                }
                return TRUE;
            }
            if (header->idFrom == IDC_MODELS && header->code == TVN_SELCHANGEDW && !self->populating)
            {
                const LPARAM item = reinterpret_cast<NMTREEVIEWW*>(l)->itemNew.lParam;
                self->browser->Select(item > 0 ? self->browser->Rows()[item - 1].modelPath : std::string{});
            }
            return FALSE;
        }
        case WM_DRAWITEM:
            if (w == IDC_ROAD_MATERIAL)
            {
                self->DrawRoadMaterial(*reinterpret_cast<DRAWITEMSTRUCT*>(l));
                return TRUE;
            }
            if (w == IDC_PREVIEW)
            {
                const auto& draw = *reinterpret_cast<DRAWITEMSTRUCT*>(l);
                WindowsTheme::DrawPreview(draw.hDC, draw.rcItem, self->browser->Preview());
                return TRUE;
            }
            if (w == IDC_TEXTURES)
            {
                self->DrawTexture(*reinterpret_cast<DRAWITEMSTRUCT*>(l));
                return TRUE;
            }
            return FALSE;
        case WM_VSCROLL:
            if (hwnd == self->road && !l)
            {
                SCROLLINFO si{sizeof(si), SIF_ALL};
                GetScrollInfo(hwnd, SB_VERT, &si);
                const int line = DialogSize(hwnd, 0, 12).cy;
                switch (LOWORD(w))
                {
                case SB_LINEUP: self->ScrollTo(si.nPos - line); break;
                case SB_LINEDOWN: self->ScrollTo(si.nPos + line); break;
                case SB_PAGEUP: self->ScrollTo(si.nPos - static_cast<int>(si.nPage)); break;
                case SB_PAGEDOWN: self->ScrollTo(si.nPos + static_cast<int>(si.nPage)); break;
                case SB_THUMBTRACK: case SB_THUMBPOSITION: self->ScrollTo(si.nTrackPos); break;
                case SB_TOP: self->ScrollTo(0); break;
                case SB_BOTTOM: self->ScrollTo(si.nMax); break;
                }
                return TRUE;
            }
            return FALSE;
        case WM_MOUSEWHEEL:
            if (hwnd == self->road)
            {
                self->ScrollTo(self->scroll - GET_WHEEL_DELTA_WPARAM(w) * DialogSize(hwnd, 0, 24).cy / WHEEL_DELTA);
                return TRUE;
            }
            return FALSE;
        }
        return FALSE;
    }

    static LRESULT CALLBACK FrameProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR data)
    {
        auto& self = *reinterpret_cast<Impl*>(data);
        if (msg == WM_SIZE || msg == WM_DPICHANGED)
        {
            const LRESULT result = DefSubclassProc(hwnd, msg, w, l);
            self.Layout();
            return result;
        }
        if (msg == WM_ACTIVATEAPP && !w)
        {
            if (!self.draggedModel.empty()) self.CancelModelDrag();
            if (!self.pressedTexture.empty()) self.CancelTextureDrag(GetDlgItem(self.models, IDC_TEXTURES));
        }
        // A modifier may already be held in a native control when the user clicks
        // the viewport. Queue the modifier immediately before SDL queues the click.
        if (msg == WM_LBUTTONDOWN || msg == WM_LBUTTONDBLCLK)
        {
            self.Post(self.pressEvent, ((w & MK_SHIFT) ? SDL_KMOD_SHIFT : 0) |
                                       ((w & MK_CONTROL) ? SDL_KMOD_CTRL : 0));
        }
        if (msg == WM_LBUTTONDOWN || msg == WM_LBUTTONDBLCLK || msg == WM_RBUTTONDOWN ||
            msg == WM_MBUTTONDOWN || msg == WM_MBUTTONDBLCLK)
        {
            SetFocus(hwnd);
        }
        if (msg == WM_NCDESTROY)
        {
            SDL_SetWindowsMessageHook(nullptr, nullptr);
            if (self.shortcuts)
            {
                DestroyAcceleratorTable(self.shortcuts);
            }
            RemoveWindowSubclass(hwnd, FrameProc, id);
        }
        return DefSubclassProc(hwnd, msg, w, l);
    }

    static bool SDLCALL Filter(void* data, MSG* msg)
    {
        auto& self = *static_cast<Impl*>(data);
        HWND focus = GetFocus();
        HWND material = self.materialEditor.Window();
        if (material && msg->message >= WM_KEYFIRST && msg->message <= WM_KEYLAST &&
            (msg->hwnd == material || IsChild(material, msg->hwnd)))
            return !IsDialogMessageW(material, msg);

        if (focus != self.frame && !IsChild(self.frame, focus))
        {
            return true;
        }

        if (!self.pressedTexture.empty() && msg->message == WM_KEYDOWN && msg->wParam == VK_ESCAPE)
        {
            self.CancelTextureDrag(GetDlgItem(self.models, IDC_TEXTURES));
            return false;
        }
        if (!self.draggedModel.empty() && msg->message == WM_KEYDOWN && msg->wParam == VK_ESCAPE)
        {
            self.CancelModelDrag();
            return false;
        }
        if (msg->message == WM_MOUSEWHEEL)
        {
            const POINT at{GET_X_LPARAM(msg->lParam), GET_Y_LPARAM(msg->lParam)};
            HWND hover = WindowFromPoint(at);

            if (hover == self.toolbar || IsChild(self.toolbar, hover))
            {
                self.Post(self.wheelEvent, GET_WHEEL_DELTA_WPARAM(msg->wParam));
                return false;
            }

            // Wheel input follows the hovered viewport or tree even when a panel has keyboard focus.
            if ((hover == self.frame || hover == self.view) && msg->hwnd != self.frame)
            {
                SendMessageW(self.frame, WM_MOUSEWHEEL, msg->wParam, msg->lParam);
                return false;
            }

            if (IsChild(self.road, hover) && msg->hwnd != hover && msg->hwnd != self.road)
            {
                SendMessageW(self.road, WM_MOUSEWHEEL, msg->wParam, msg->lParam);
                return false;
            }

            HWND outlineList = GetDlgItem(self.outline.Window(), IDC_OUTLINE_LIST);
            if (hover == outlineList && msg->hwnd != outlineList)
            {
                SendMessageW(outlineList, WM_MOUSEWHEEL, msg->wParam, msg->lParam);
                return false;
            }

            HWND assets = GetDlgItem(self.models, self.browser->ActiveTab() == ContentBrowser::Tab::Models ? IDC_MODELS : IDC_TEXTURES);
            if ((hover == assets || hover == GetDlgItem(self.models, IDC_TEXTURE_FOLDERS)) && msg->hwnd != hover)
            {
                SendMessageW(hover, WM_MOUSEWHEEL, msg->wParam, msg->lParam);
                return false;
            }
        }
        if (msg->message == WM_KEYDOWN && msg->wParam == VK_TAB && !(GetKeyState(VK_CONTROL) & 0x8000))
        {
            const bool back = (GetKeyState(VK_SHIFT) & 0x8000) != 0;
            HWND next = GetNextDlgTabItem(self.frame, focus == self.frame ? nullptr : focus, back);
            HWND boundary = GetNextDlgTabItem(self.frame, nullptr, back);
            HWND target = focus != self.frame && next == boundary ? self.frame : next;
            if (IsChild(self.road, target))
            {
                RECT child{}, panel{};
                GetWindowRect(target, &child);
                MapWindowPoints(nullptr, self.road, reinterpret_cast<POINT*>(&child), 2);
                GetClientRect(self.road, &panel);
                if (child.top < 0 || child.bottom > panel.bottom)
                {
                    self.ScrollTo(self.scroll + (child.top < 0 ? child.top : child.bottom - panel.bottom));
                }
            }
            SetFocus(target);
            return false;
        }

        if (msg->message == WM_KEYDOWN && msg->wParam == 'F' && GetDlgCtrlID(focus) == IDC_OUTLINE_LIST &&
            !(GetKeyState(VK_CONTROL) & 0x8000) && !(GetKeyState(VK_MENU) & 0x8000) &&
            self.outline.QueueTerrainFill((msg->lParam & (1L << 30)) != 0)) return false;

        const int id = GetDlgCtrlID(focus);
        const int field = id == IDC_MARKER_TEXT ? InspectorEdit::markerText : id - IDC_LOC_X;
        if (IsChild(self.road, focus) && (id == IDC_MARKER_TEXT || (id >= IDC_LOC_X && id <= IDC_GEO_LOCATION)) &&
            msg->message >= WM_KEYFIRST && msg->message <= WM_KEYLAST)
        {
            if (msg->message == WM_KEYDOWN)
            {
                if (msg->wParam == VK_RETURN)
                {
                    self.SubmitField(focus, field);
                    return false;
                }
                if (msg->wParam == VK_ESCAPE)
                {
                    SetFocus(self.frame);
                    return false;
                }
                if (msg->wParam == 'A' && (GetKeyState(VK_CONTROL) & 0x8000))
                {
                    SendMessageW(focus, EM_SETSEL, 0, -1);
                    return false;
                }
            }
            TranslateMessage(msg);
            DispatchMessageW(msg);
            return false;
        }
        if (focus != self.frame && msg->message >= WM_KEYFIRST && msg->message <= WM_KEYLAST)
        {
            if (TranslateAcceleratorW(self.frame, self.shortcuts, msg))
            {
                return false;
            }
            for (HWND panel : {self.toolbar, self.models, self.outline.Window(), self.road})
            {
                if ((panel == focus || IsChild(panel, focus)) && IsDialogMessageW(panel, msg))
                {
                    return false;
                }
            }
        }
        return true;
    }
};

WindowsPanels::WindowsPanels() : impl(std::make_unique<Impl>()) {}
WindowsPanels::~WindowsPanels() = default;

SDL_Window* WindowsPanels::Create(SDL_Window* frame, ContentBrowser& browser)
{
    auto& p = *impl;
    p.browser = &browser;
    p.frameWindow = frame;
    p.frame = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(frame), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
    const HINSTANCE module = GetModuleHandleW(nullptr);
    INITCOMMONCONTROLSEX common{sizeof(common), ICC_LISTVIEW_CLASSES | ICC_TREEVIEW_CLASSES | ICC_BAR_CLASSES | ICC_TAB_CLASSES};

    if (!p.frame || !InitCommonControlsEx(&common))
    {
        WinError("Cannot initialize editor controls");
        return nullptr;
    }

    p.actions = SDL_RegisterEvents(8);

    if (!p.actions)
    {
        return nullptr;
    }

    p.speedEvent = p.actions + 1;
    p.wheelEvent = p.actions + 2;
    p.pressEvent = p.actions + 3;
    p.toolbarEvent = p.actions + 4;
    p.transformEvent = p.actions + 5;
    p.modelDropEvent = p.actions + 6;
    p.materialDropEvent = p.actions + 7;
    WNDCLASSW wc{};
    wc.hInstance = module;
    wc.lpfnWndProc = ViewProc;
    wc.lpszClassName = L"BigSkyEditorViewport";
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);

    if (!RegisterClassW(&wc) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS)
    {
        WinError("Cannot register viewport");
        return nullptr;
    }

    SetWindowLongPtrW(p.frame, GWL_STYLE, GetWindowLongPtrW(p.frame, GWL_STYLE) | WS_CLIPCHILDREN);
    p.view = CreateWindowExW(WS_EX_NOPARENTNOTIFY, wc.lpszClassName, L"Viewport", WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS,
                             0, 0, 1, 1, p.frame, nullptr, module, nullptr);
    auto panel = [&](int resource)
    {
        return CreateDialogParamW(module, MAKEINTRESOURCEW(resource), p.frame, Impl::DialogProc, reinterpret_cast<LPARAM>(&p));
    };

    p.toolbar = panel(IDD_TOOLBAR);
    p.models = panel(IDD_BROWSER);
    p.road = panel(IDD_ROAD);

    if (!p.view || !p.toolbar || !p.models || !p.road || !p.outline.Create(p.frame, module) || !SetWindowSubclass(p.frame, Impl::FrameProc, 2, reinterpret_cast<DWORD_PTR>(&p)))
    {
        WinError("Cannot create editor panels");
        return nullptr;
    }

    if (!SetWindowSubclass(GetDlgItem(p.models, IDC_TEXTURES), Impl::TextureListProc, 2, reinterpret_cast<DWORD_PTR>(&p)))
    {
        WinError("Cannot initialize texture dragging");
        return nullptr;
    }
    HWND tabs = GetDlgItem(p.models, IDC_BROWSER_TABS);
    for (const wchar_t* label : {L"Models", L"Textures"})
    {
        TCITEMW item{};
        item.mask = TCIF_TEXT;
        item.pszText = const_cast<wchar_t*>(label);
        if (TabCtrl_InsertItem(tabs, TabCtrl_GetItemCount(tabs), &item) == -1)
        {
            WinError("Cannot create content browser tabs");
            return nullptr;
        }
    }
    ACCEL keys[] = {{FCONTROL | FVIRTKEY, 'S', BSC_FILE_SAVE}, {FCONTROL | FVIRTKEY, 'Z', BSC_EDIT_UNDO},
                    {FCONTROL | FVIRTKEY, 'Y', BSC_EDIT_REDO}};

    p.shortcuts = CreateAcceleratorTableW(keys, 3);

    if (!p.shortcuts)
    {
        WinError("Cannot create editor shortcuts");
        return nullptr;
    }

    if (!SetWindowSubclass(GetDlgItem(p.road, IDC_GEO_LOCATION), Impl::GeographicEditProc, 1, 0))
    {
        WinError("Cannot initialize geographic coordinate input");
        return nullptr;
    }
    SendDlgItemMessageW(p.road, IDC_MARKER_TEXT, EM_SETLIMITTEXT, 2 * Marker::maxCharacters, 0);
    for (int id = IDC_LOC_X; id <= IDC_GEO_LOCATION; ++id)
    {
        SendDlgItemMessageW(p.road, id, EM_SETLIMITTEXT, 120, 0);
    }
    Choices(p.road, IDC_ALIGNMENT, {L"Straight", L"B\u00e9zier"});
    Choices(p.road, IDC_LAYOUT, {L"2 north / 1 south", L"1 north / 2 south", L"Center turn"});
    Choices(p.road, IDC_MEDIAN, {L"Concrete", L"Grass"});
    Choices(p.road, IDC_EDGE, {L"None", L"Transition strip", L"Curb"});
    SendDlgItemMessageW(p.toolbar, IDC_SPEED, TBM_SETRANGE, FALSE, MAKELONG(0, speedSteps));
    SendDlgItemMessageW(p.toolbar, IDC_SPEED, TBM_SETPAGESIZE, 0, speedSteps / 10);
    const SIZE minimum = DialogSize(p.road, BROWSER_WIDTH + ROAD_PANEL_WIDTH + 72, 90);
    SDL_SetWindowMinimumSize(frame, minimum.cx, minimum.cy);
    p.Layout();
    const SDL_PropertiesID props = SDL_CreateProperties();
    SDL_Window* result = nullptr;

    if (props && SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_WIN32_HWND_POINTER, p.view))
    {
        result = SDL_CreateWindowWithProperties(props);
    }

    SDL_DestroyProperties(props);

    if (result)
    {
        SDL_SetWindowsMessageHook(Impl::Filter, &p);
        SetFocus(p.frame);
    }

    return result;
}

void WindowsPanels::SyncOutliner(const SceneOutliner& model) { impl->outline.Sync(model); }

std::optional<OutlineCommand> WindowsPanels::TakeOutlineCommand(const SDL_Event& event)
{
    return impl->outline.Take(event);
}

std::optional<WindowsPanels::ModelDrop> WindowsPanels::TakeModelDrop(const SDL_Event& event)
{
    if (event.type != impl->modelDropEvent || impl->modelDrops.empty()) return {};
    auto drop = std::move(impl->modelDrops.front());
    impl->modelDrops.pop_front();
    return drop;
}

std::optional<WindowsPanels::MaterialDrop> WindowsPanels::TakeMaterialDrop(const SDL_Event& event)
{
    if (event.type != impl->materialDropEvent || impl->materialDrops.empty()) return {};
    auto drop = std::move(impl->materialDrops.front());
    impl->materialDrops.pop_front();
    return drop;
}

bool WindowsPanels::ViewportFocused() const
{
    return GetFocus() == impl->frame;
}

void WindowsPanels::ToViewport(SDL_Window* frame, SDL_Event& event, const SDL_Rect& scene) const
{
    RECT area{}, client{};
    GetClientRect(impl->frame, &client);
    GetClientRect(impl->view, &area);
    POINT origin{};
    MapWindowPoints(impl->view, impl->frame, &origin, 1);
    int width = 0, height = 0;
    SDL_GetWindowSize(frame, &width, &height);
    const float sx = float(scene.w) / std::max<LONG>(1, area.right), sy = float(scene.h) / std::max<LONG>(1, area.bottom);
    const float wx = float(client.right) / std::max(1, width), wy = float(client.bottom) / std::max(1, height);

    auto convert = [&](float& x, float& y)
    {
        x = (x * wx - origin.x) * sx;
        y = (y * wy - origin.y) * sy;
    };

    switch (event.type)
    {
        case SDL_EVENT_MOUSE_MOTION:
            convert(event.motion.x, event.motion.y);
            event.motion.xrel *= sx * wx;
            event.motion.yrel *= sy * wy;
            break;
        case SDL_EVENT_MOUSE_WHEEL: convert(event.wheel.mouse_x, event.wheel.mouse_y); break;
        default: convert(event.button.x, event.button.y); break;
    }
}

bool WindowsPanels::Command(const SDL_Event& event, RoadAction& action) const
{
    if (event.type != impl->actions)
    {
        return false;
    }

    action = static_cast<RoadAction>(event.user.code);

    return true;
}

bool WindowsPanels::HandleInput(const SDL_Event& event, FreeCamera& camera, TransformSettings& settings)
{
    if (event.type == impl->pressEvent)
    {
        impl->pressModifiers = static_cast<SDL_Keymod>(event.user.code);
    }
    else if (event.type == impl->toolbarEvent)
    {
        if (event.user.code == IDC_SNAP_TO_GRID)
        {
            settings.snapEnabled = !settings.snapEnabled;
        }
        else
        {
            settings.ToggleSpace();
        }
    }
    else if (event.type == impl->speedEvent)
    {
        camera.SetSpeedFraction(double(event.user.code) / speedSteps);
    }
    else if (event.type == impl->wheelEvent)
    {
        camera.ScrollSpeed(double(event.user.code) / WHEEL_DELTA);
    }
    else
    {
        return false;
    }
    return true;
}

std::optional<InspectorEdit> WindowsPanels::TakeInspectorEdit(const SDL_Event& event)
{
    if (event.type != impl->transformEvent || impl->pendingEdits.empty()) return {};
    InspectorEdit edit = std::move(impl->pendingEdits.front());
    impl->pendingEdits.pop_front();
    return edit;
}

SDL_Keymod WindowsPanels::TakePressModifiers()
{
    return std::exchange(impl->pressModifiers, std::nullopt).value_or(SDL_GetModState());
}

void WindowsPanels::Sync(const RoadEditorState& state, const Road* current, bool plan, const FreeCamera& camera, const TransformSettings& settings, const TransformFields& fields, const PropertiesInspector& properties)
{
    auto& p = *impl;
    p.SyncFields(fields);

    if (settings != p.displayedSettings)
    {
        p.ShowSettings(settings);
    }

    if (!p.populated)
    {
        p.Populate();
    }

    if (p.browser->ActiveTab() == ContentBrowser::Tab::Models && p.browser->UpdatePreview())
    {
        const auto& status = p.browser->ScanStatus().empty() ? p.browser->PreviewStatus() : p.browser->ScanStatus();
        SetDlgItemTextW(p.models, IDC_PREVIEW_STATUS, Wide(status).c_str());
        InvalidateRect(GetDlgItem(p.models, IDC_PREVIEW), nullptr, FALSE);
    }

    const int point = !plan && current ? state.point : -1;

    p.UpdateInspector({current ? current->style : state.defaults, plan, plan && state.drawing,
                       state.draft.has_value(), plan && state.drawing && state.draft->points.size() >= 2,
                       state.selected, point, point >= 0 ? current->points[point].position.y : 0.0, Material::Revision(), properties});

    const double speed = camera.GetMoveSpeed();

    if (p.lastSpeed != speed)
    {
        p.lastSpeed = speed;
        const int pos = static_cast<int>(std::lround(camera.GetSpeedFraction() * speedSteps));
        SendDlgItemMessageW(p.toolbar, IDC_SPEED, TBM_SETPOS, TRUE, pos);
        wchar_t text[48];

        if (speed >= 1000.0)
        {
            swprintf(text, 48, L"%.2f km/s", speed / 1000.0);
        }
        else
        {
            swprintf(text, 48, speed < 10.0 ? L"%.2f m/s" : L"%.1f m/s", speed);
        }

        SetDlgItemTextW(p.toolbar, IDC_SPEED_VALUE, text);
    }
}
