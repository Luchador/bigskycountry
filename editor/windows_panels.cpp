#include "windows_panels.hpp"
#include "content_browser.hpp"
#include "edit_history.hpp"
#include "free_camera.hpp"
#include "menu_ids.h"
#include "panel_ids.h"
#include "road_action.hpp"

#include <windows.h>
#include <commctrl.h>
#include <windowsx.h>
#include <algorithm>
#include <cmath>
#include <cwchar>
#include <initializer_list>
#include <map>
#include <string>
#include <utility>

namespace
{
    constexpr int speedSteps = 1000;

    std::wstring Wide(const std::string& text)
    {
        const int count = MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
        std::wstring wide(count, L'\0');
        if (count)
        {
            MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), wide.data(), count);
        }
        return wide;
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

    void DrawPreview(const DRAWITEMSTRUCT& draw, SDL_Surface* surface)
    {
        SetDCBrushColor(draw.hDC, RGB(17, 23, 28));
        FillRect(draw.hDC, &draw.rcItem, static_cast<HBRUSH>(GetStockObject(DC_BRUSH)));
        if (!surface)
        {
            return;
        }
        BITMAPINFO info{};
        info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        info.bmiHeader.biWidth = surface->pitch / 4;
        info.bmiHeader.biHeight = -surface->h;
        info.bmiHeader.biPlanes = 1;
        info.bmiHeader.biBitCount = 32;
        info.bmiHeader.biCompression = BI_RGB;
        const int w = draw.rcItem.right - draw.rcItem.left;
        const int h = draw.rcItem.bottom - draw.rcItem.top;
        const double scale = std::min(double(w) / surface->w, double(h) / surface->h);
        const int iw = static_cast<int>(surface->w * scale), ih = static_cast<int>(surface->h * scale);
        StretchDIBits(draw.hDC, draw.rcItem.left + (w - iw) / 2, draw.rcItem.top + (h - ih) / 2, iw, ih,
                      0, 0, surface->w, surface->h, surface->pixels, &info, DIB_RGB_COLORS, SRCCOPY);
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
    HACCEL shortcuts = nullptr;
    Uint32 actions = 0, speedEvent = 0, wheelEvent = 0, pressEvent = 0;
    std::optional<SDL_Keymod> pressModifiers;
    ContentBrowser* browser = nullptr;
    bool populating = false, populated = false;
    int scroll = 0;
    double lastSpeed = -1;

    struct Inspector
    {
        RoadStyle style;
        bool plan, drawing, draft, canFinish;
        int selected, point;
        double elevation;
        bool operator==(const Inspector&) const = default;
    };
    std::optional<Inspector> lastInspector;

    void Post(Uint32 type, int code) const
    {
        SDL_Event event{};
        event.type = type;
        event.user.windowID = SDL_GetWindowID(frameWindow);
        event.user.code = code;
        if (!SDL_PushEvent(&event))
        {
            SDL_Log("Editor control: %s", SDL_GetError());
        }
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
        MoveWindow(toolbar, 0, 0, rect.right, top, TRUE);
        MoveWindow(models, 0, top, left, h, TRUE);
        MoveWindow(road, rect.right - right, top, right, h, TRUE);
        MoveWindow(view, left, top, std::max<int>(1, rect.right - left - right), std::max(1, h), TRUE);

        auto moveTool = [&](int id, int rightOffset, int y, int w, int height)
        {
            const SIZE offset = DialogSize(toolbar, rightOffset, y), size = DialogSize(toolbar, w, height);
            MoveWindow(GetDlgItem(toolbar, id), rect.right - offset.cx, offset.cy, size.cx, size.cy, TRUE);
        };
        moveTool(IDC_SPEED_LABEL, 200, 7, 52, 12);
        moveTool(IDC_SPEED, 146, 3, 89, 19);
        moveTool(IDC_SPEED_VALUE, 54, 7, 50, 12);
        const SIZE inset = DialogSize(models, 7, 124);
        MoveWindow(GetDlgItem(models, IDC_MODELS), inset.cx, inset.cy, left - 2 * inset.cx,
                   std::max<int>(1, h - inset.cy - DialogSize(models, 0, 6).cy), TRUE);

        SCROLLINFO si{sizeof(si), SIF_RANGE | SIF_PAGE | SIF_POS};
        si.nMax = DialogSize(road, 0, ROAD_PANEL_HEIGHT).cy - 1;
        si.nPage = static_cast<UINT>(h);
        si.nPos = scroll;
        const int next = SetScrollInfo(road, SB_VERT, &si, TRUE);
        ScrollTo(next);
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

    void Populate()
    {
        populated = true;
        populating = true;
        HWND tree = GetDlgItem(models, IDC_MODELS);
        SendMessageW(tree, WM_SETREDRAW, FALSE, 0);
        TreeView_DeleteAllItems(tree);
        std::map<std::string, HTREEITEM> folders;
        auto add = [&](HTREEITEM parent, const std::string& label, LPARAM data)
        {
            std::wstring wide = Wide(label);
            TVINSERTSTRUCTW node{};
            node.hParent = parent;
            node.hInsertAfter = TVI_LAST;
            node.item.mask = TVIF_TEXT | TVIF_PARAM;
            node.item.pszText = wide.data();
            node.item.lParam = data;
            return TreeView_InsertItem(tree, &node);
        };

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
                    it->second = add(parent, row.modelPath.substr(start, slash - start), 0);
                }
                parent = it->second;
                start = slash + 1;
            }
            const HTREEITEM item = add(parent, row.label, static_cast<LPARAM>(i + 1));
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
        SetDlgItemTextW(models, IDC_MODEL_ROOT, browser->ScanStatus().empty() ? L"assets/models" : Wide(browser->ScanStatus()).c_str());
        SendMessageW(tree, WM_SETREDRAW, TRUE, 0);
        InvalidateRect(tree, nullptr, TRUE);
        populating = false;
    }

    void Control(int id, int notification)
    {
        if (notification == BN_CLICKED)
        {
            switch (id)
            {
            case IDC_REFRESH: browser->Refresh(); Populate(); return;
            case IDC_VIEW: Post(actions, int(RoadAction::ChangeView)); SetFocus(frame); return;
            case IDC_NEW: Post(actions, int(RoadAction::New)); SetFocus(frame); return;
            case IDC_FINISH: Post(actions, int(RoadAction::Finish)); SetFocus(frame); return;
            case IDC_CANCEL_DRAFT: Post(actions, int(RoadAction::Cancel)); SetFocus(frame); return;
            case IDC_REMOVE: Post(actions, int(RoadAction::Remove)); return;
            case IDC_FEWER: Post(actions, int(RoadAction::Fewer)); return;
            case IDC_MORE: Post(actions, int(RoadAction::More)); return;
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
        lastInspector = data;
        const int lanes = data.style.GetLaneCount();
        const std::wstring title = data.drawing ? L"Road editor | Draft" : data.selected >= 0 ?
            L"Road editor | Road " + std::to_wstring(data.selected + 1) : L"Road editor";
        SetDlgItemTextW(road, IDC_ROAD_TITLE, title.c_str());
        SetDlgItemTextW(road, IDC_VIEW, data.plan ? L"Back to 3D  [R]" : L"Top-down view  [R]");
        SetDlgItemTextW(road, IDC_NEW, data.draft && !data.drawing ? L"Resume draft" : L"New road");
        EnableWindow(GetDlgItem(road, IDC_NEW), !data.drawing);
        SendDlgItemMessageW(road, IDC_ALIGNMENT, CB_SETCURSEL, data.style.curved, 0);
        SetDlgItemInt(road, IDC_LANE_COUNT, lanes, FALSE);
        EnableWindow(GetDlgItem(road, IDC_FEWER), lanes > 1);
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
        EnableWindow(GetDlgItem(road, IDC_REMOVE), data.selected >= 0 && !data.drawing);
    }

    static INT_PTR CALLBACK DialogProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l)
    {
        if (msg == WM_INITDIALOG)
        {
            SetWindowLongPtrW(hwnd, DWLP_USER, l);
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
        case WM_NOTIFY:
        {
            auto* header = reinterpret_cast<NMHDR*>(l);
            if (header->idFrom == IDC_MODELS && header->code == TVN_SELCHANGEDW && !self->populating)
            {
                const LPARAM item = reinterpret_cast<NMTREEVIEWW*>(l)->itemNew.lParam;
                self->browser->Select(item > 0 ? self->browser->Rows()[item - 1].modelPath : std::string{});
            }
            return FALSE;
        }
        case WM_DRAWITEM:
            if (w == IDC_PREVIEW)
            {
                DrawPreview(*reinterpret_cast<DRAWITEMSTRUCT*>(l), self->browser->Preview());
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
        // A modifier may already be held in a native control when the user clicks
        // the viewport. Queue the modifier immediately before SDL queues the click.
        if (msg == WM_LBUTTONDOWN || msg == WM_LBUTTONDBLCLK)
        {
            self.Post(self.pressEvent, ((w & MK_SHIFT) ? SDL_KMOD_SHIFT : 0) |
                                       ((w & MK_CONTROL) ? SDL_KMOD_CTRL : 0));
        }
        if (msg == WM_LBUTTONDOWN || msg == WM_LBUTTONDBLCLK || msg == WM_RBUTTONDOWN)
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
        if (focus != self.frame && !IsChild(self.frame, focus))
        {
            return true;
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
            HWND tree = GetDlgItem(self.models, IDC_MODELS);
            if (hover == tree && msg->hwnd != tree)
            {
                SendMessageW(tree, WM_MOUSEWHEEL, msg->wParam, msg->lParam);
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
        if (focus != self.frame && msg->message >= WM_KEYFIRST && msg->message <= WM_KEYLAST)
        {
            if (TranslateAcceleratorW(self.frame, self.shortcuts, msg))
            {
                return false;
            }
            for (HWND panel : {self.toolbar, self.models, self.road})
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
    INITCOMMONCONTROLSEX common{sizeof(common), ICC_TREEVIEW_CLASSES | ICC_BAR_CLASSES};
    if (!p.frame || !InitCommonControlsEx(&common))
    {
        WinError("Cannot initialize editor controls");
        return nullptr;
    }

    p.actions = SDL_RegisterEvents(4);
    if (!p.actions)
    {
        return nullptr;
    }
    p.speedEvent = p.actions + 1;
    p.wheelEvent = p.actions + 2;
    p.pressEvent = p.actions + 3;
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
    if (!p.view || !p.toolbar || !p.models || !p.road || !SetWindowSubclass(p.frame, Impl::FrameProc, 2, reinterpret_cast<DWORD_PTR>(&p)))
    {
        WinError("Cannot create editor panels");
        return nullptr;
    }
    ACCEL keys[] = {{FCONTROL | FVIRTKEY, 'S', BSC_FILE_SAVE}, {FCONTROL | FVIRTKEY, 'Z', BSC_EDIT_UNDO},
                    {FCONTROL | FVIRTKEY, 'Y', BSC_EDIT_REDO}};
    p.shortcuts = CreateAcceleratorTableW(keys, 3);
    if (!p.shortcuts)
    {
        WinError("Cannot create editor shortcuts");
        return nullptr;
    }
    Choices(p.road, IDC_ALIGNMENT, {L"Straight", L"B\u00e9zier"});
    Choices(p.road, IDC_LAYOUT, {L"2 north / 1 south", L"1 north / 2 south", L"Center turn"});
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

bool WindowsPanels::HandleInput(const SDL_Event& event, FreeCamera& camera)
{
    if (event.type == impl->pressEvent)
    {
        impl->pressModifiers = static_cast<SDL_Keymod>(event.user.code);
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

SDL_Keymod WindowsPanels::TakePressModifiers()
{
    return std::exchange(impl->pressModifiers, std::nullopt).value_or(SDL_GetModState());
}

void WindowsPanels::Sync(const RoadEditorState& state, const Road* current, bool plan, const FreeCamera& camera)
{
    auto& p = *impl;
    if (!p.populated)
    {
        p.Populate();
    }
    if (p.browser->UpdatePreview())
    {
        SetDlgItemTextW(p.models, IDC_PREVIEW_STATUS, Wide(p.browser->PreviewStatus()).c_str());
        InvalidateRect(GetDlgItem(p.models, IDC_PREVIEW), nullptr, FALSE);
    }
    const int point = !plan && current ? state.point : -1;
    p.UpdateInspector({current ? current->style : state.defaults, plan, plan && state.drawing,
                       state.draft.has_value(), plan && state.drawing && state.draft->points.size() >= 2,
                       state.selected, point, point >= 0 ? current->points[point].position.y : 0.0});

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
