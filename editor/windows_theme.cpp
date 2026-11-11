#include "windows_theme.hpp"
#include "panel_ids.h"
#include <commctrl.h>
#include <dwmapi.h>
#include <uxtheme.h>
#include <algorithm>
#include <map>
#include <string>

namespace WindowsTheme
{
namespace
{
    using namespace EditorTheme;
    enum class Face { Button, Combo, Tabs, Slider, Header, Divider };

    HBRUSH Solid(COLORREF color)
    {
        struct Brushes : std::map<COLORREF, HBRUSH>
        {
            ~Brushes() { for (const auto& entry : *this) DeleteObject(entry.second); }
        };
        static Brushes brushes;
        auto [it, added] = brushes.try_emplace(color);
        if (added) it->second = CreateSolidBrush(color);
        return it->second;
    }

    bool IsClass(HWND window, const wchar_t* name)
    {
        wchar_t cls[64]{};
        GetClassNameW(window, cls, 64);
        return lstrcmpiW(cls, name) == 0;
    }

    int Scale(HWND window, int value)
    {
        return MulDiv(value, GetDpiForWindow(window), 96);
    }

    std::wstring Caption(HWND window)
    {
        std::wstring value(GetWindowTextLengthW(window) + 1, L'\0');
        value.resize(GetWindowTextW(window, value.data(), static_cast<int>(value.size())));
        return value;
    }

    void Box(HDC dc, RECT r, SDL_Color fill, SDL_Color edge)
    {
        FillRect(dc, &r, Brush(fill));
        FrameRect(dc, &r, Brush(edge));
    }

    void Label(HDC dc, RECT r, const wchar_t* caption, SDL_Color color, UINT flags)
    {
        SetBkMode(dc, TRANSPARENT);
        SetTextColor(dc, Color(color));
        DrawTextW(dc, caption, -1, &r, flags | DT_SINGLELINE | DT_VCENTER);
    }

    void Focus(HWND w, HDC dc, RECT r)
    {
        if (GetFocus() == w && !(SendMessageW(w, WM_QUERYUISTATE, 0, 0) & UISF_HIDEFOCUS))
        {
            InflateRect(&r, -2, -2);
            FrameRect(dc, &r, Brush(accent));
        }
    }

    void Stroke(HDC dc, COLORREF color, POINT a, POINT b, POINT c)
    {
        SelectObject(dc, GetStockObject(DC_PEN));
        SetDCPenColor(dc, color);
        const POINT points[]{a, b, c};
        Polyline(dc, points, 3);
    }

    LRESULT CALLBACK DialogColors(HWND, UINT, WPARAM, LPARAM, UINT_PTR, DWORD_PTR);

    void DrawButton(HWND w, HDC dc, RECT r, bool hot)
    {
        const auto state = SendMessageW(w, BM_GETSTATE, 0, 0);
        const auto style = GetWindowLongPtrW(w, GWL_STYLE);
        const auto type = style & BS_TYPEMASK;
        const bool enabled = IsWindowEnabled(w);
        const SDL_Color fg = enabled ? text : disabled;
        const SDL_Color face = !enabled ? header : (state & BST_PUSHED) ? pressed : hot ? hover : button;
        const UINT prefix = SendMessageW(w, WM_QUERYUISTATE, 0, 0) & UISF_HIDEACCEL ? DT_HIDEPREFIX : 0;
        if (type == BS_AUTOCHECKBOX || type == BS_CHECKBOX)
        {
            DWORD_PTR bg = Color(panel);
            GetWindowSubclass(GetParent(w), DialogColors, 1, &bg);
            FillRect(dc, &r, Solid(static_cast<COLORREF>(bg)));
            const int side = std::min<int>(r.bottom - r.top, Scale(w, 13));
            const LONG y = (r.bottom - side) / 2;
            RECT mark{r.left, y, r.left + side, y + side};
            const bool checked = SendMessageW(w, BM_GETCHECK, 0, 0) == BST_CHECKED;
            Box(dc, mark, checked ? selected : hot ? button : field, enabled ? disabled : button);
            if (checked)
            {
                Stroke(dc, Color(fg), {mark.left + side/5, y + side/2},
                       {mark.left + side*2/5, y + side*3/4}, {mark.right - side/6, y + side/4});
            }
            r.left = mark.right + Scale(w, 4);
            Label(dc, r, Caption(w).c_str(), fg, DT_LEFT | prefix);
        }
        else
        {
            Box(dc, r, face, type == BS_DEFPUSHBUTTON ? accent : border);
            auto bitmap = reinterpret_cast<HBITMAP>(SendMessageW(w, BM_GETIMAGE, IMAGE_BITMAP, 0));
            if ((style & BS_BITMAP) && bitmap)
            {
                BITMAP b{};
                GetObjectW(bitmap, sizeof(b), &b);
                HDC source = CreateCompatibleDC(dc);
                auto old = SelectObject(source, bitmap);
                BitBlt(dc, (r.right - b.bmWidth)/2, (r.bottom - b.bmHeight)/2,
                       b.bmWidth, b.bmHeight, source, 0, 0, SRCCOPY);
                SelectObject(source, old);
                DeleteDC(source);
            }
            else
            {
                Label(dc, r, Caption(w).c_str(), fg, DT_CENTER | prefix);
            }
        }
        Focus(w, dc, r);
    }

    void DrawCombo(HWND w, HDC dc, RECT r, bool hot)
    {
        const bool open = SendMessageW(w, CB_GETDROPPEDSTATE, 0, 0) != 0;
        Box(dc, r, open ? pressed : hot ? hover : button, GetFocus() == w ? accent : border);
        COMBOBOXINFO info{sizeof(info)};
        GetComboBoxInfo(w, &info);
        const LONG x = (info.rcButton.left + info.rcButton.right) / 2;
        const LONG y = (r.top + r.bottom) / 2;
        const int d = Scale(w, 3);
        const auto fg = IsWindowEnabled(w) ? text : disabled;
        Stroke(dc, Color(fg), {x-d, y-1}, {x, y+d-1}, {x+d+1, y-2});
        r.left += Scale(w, 5);
        r.right = info.rcButton.left;
        Label(dc, r, Caption(w).c_str(), fg, DT_LEFT | DT_NOPREFIX | DT_END_ELLIPSIS);
    }

    void DrawTabs(HWND w, HDC dc, RECT area, POINT mouse, bool hot)
    {
        FillRect(dc, &area, Brush(panel));
        const int current = TabCtrl_GetCurSel(w);
        for (int i = 0; i < TabCtrl_GetItemCount(w); ++i)
        {
            RECT r{};
            wchar_t title[128]{};
            TCITEMW item{};
            item.mask = TCIF_TEXT;
            item.pszText = title;
            item.cchTextMax = 128;
            if (!TabCtrl_GetItemRect(w, i, &r) || !TabCtrl_GetItem(w, i, &item)) continue;
            FillRect(dc, &r, Brush(i == current ? pressed : hot && PtInRect(&r, mouse) ? button : list));
            Label(dc, r, title, i == current ? text : secondary, DT_CENTER | DT_NOPREFIX);
            if (i == current)
            {
                Focus(w, dc, r);
                r.top = r.bottom - Scale(w, 2);
                FillRect(dc, &r, Brush(accent));
            }
        }
    }

    void DrawSlider(HWND w, HDC dc, RECT area, bool hot)
    {
        FillRect(dc, &area, Brush(chrome));
        RECT thumb{}, track{};
        SendMessageW(w, TBM_GETTHUMBRECT, 0, reinterpret_cast<LPARAM>(&thumb));
        SendMessageW(w, TBM_GETCHANNELRECT, 0, reinterpret_cast<LPARAM>(&track));
        const LONG y = (thumb.top + thumb.bottom) / 2;
        track.top = y - Scale(w, 2);
        track.bottom = y + Scale(w, 2);
        FillRect(dc, &track, Brush(button));
        track.right = (thumb.left + thumb.right) / 2;
        FillRect(dc, &track, Brush(accent));
        Box(dc, thumb, hot || GetCapture() == w ? text : secondary, border);
        Focus(w, dc, area);
    }

    void DrawHeader(HWND w, HDC dc, RECT area)
    {
        FillRect(dc, &area, Brush(header));
        for (int i = 0; i < Header_GetItemCount(w); ++i)
        {
            RECT r{};
            wchar_t value[256]{};
            HDITEMW item{};
            item.mask = HDI_TEXT;
            item.pszText = value;
            item.cchTextMax = 256;
            if (!Header_GetItemRect(w, i, &r) || !Header_GetItem(w, i, &item)) continue;
            RECT rule{r.right - 1, r.top, r.right, r.bottom};
            FillRect(dc, &rule, Brush(border));
            r.left += Scale(w, 5);
            Label(dc, r, value, secondary, DT_LEFT | DT_NOPREFIX | DT_END_ELLIPSIS);
        }
    }

    LRESULT CALLBACK FaceProc(HWND w, UINT msg, WPARAM wp, LPARAM lp, UINT_PTR id, DWORD_PTR data)
    {
        if (msg == WM_PAINT || msg == WM_PRINTCLIENT)
        {
            PAINTSTRUCT paint{};
            HDC dc = wp ? reinterpret_cast<HDC>(wp) : BeginPaint(w, &paint);
            const int saved = SaveDC(dc);
            auto font = reinterpret_cast<HFONT>(SendMessageW(w, WM_GETFONT, 0, 0));
            if (font) SelectObject(dc, font);
            RECT r{};
            GetClientRect(w, &r);
            POINT mouse{};
            GetCursorPos(&mouse);
            const bool hot = WindowFromPoint(mouse) == w && IsWindowEnabled(w);
            ScreenToClient(w, &mouse);
            switch (static_cast<Face>(data))
            {
            case Face::Button: DrawButton(w, dc, r, hot); break;
            case Face::Combo: DrawCombo(w, dc, r, hot); break;
            case Face::Tabs: DrawTabs(w, dc, r, mouse, hot); break;
            case Face::Slider: DrawSlider(w, dc, r, hot); break;
            case Face::Header: DrawHeader(w, dc, r); break;
            case Face::Divider: FillRect(dc, &r, Brush(border)); break;
            }
            RestoreDC(dc, saved);
            if (!wp) EndPaint(w, &paint);
            return 0;
        }
        if (msg == WM_ERASEBKGND) return 1;
        if (msg == WM_CTLCOLORLISTBOX)
        {
            HDC dc = reinterpret_cast<HDC>(wp);
            SetTextColor(dc, Color(text));
            SetBkColor(dc, Color(list));
            return reinterpret_cast<LRESULT>(Brush(list));
        }
        if (msg == WM_NCDESTROY) RemoveWindowSubclass(w, FaceProc, id);
        const LRESULT result = DefSubclassProc(w, msg, wp, lp);
        switch (msg)
        {
        case WM_MOUSEMOVE:
        {
            TRACKMOUSEEVENT track{sizeof(track), TME_LEAVE, w, 0};
            TrackMouseEvent(&track);
            break;
        }
        case WM_MOUSELEAVE: case WM_SETFOCUS: case WM_KILLFOCUS: case WM_ENABLE:
        case WM_LBUTTONDOWN: case WM_LBUTTONUP: case WM_CAPTURECHANGED: case WM_UPDATEUISTATE:
        case BM_SETSTATE: case BM_SETCHECK: case BM_SETIMAGE: case CB_SETCURSEL: case TCM_SETCURSEL:
            break;
        default: return result;
        }
        if (IsWindow(w)) InvalidateRect(w, nullptr, FALSE);
        return result;
    }

    void DrawScrollBar(HWND w, HDC dc, RECT window, bool vertical)
    {
        SCROLLBARINFO info{sizeof(info)};
        if (!GetScrollBarInfo(w, vertical ? OBJID_VSCROLL : OBJID_HSCROLL, &info) ||
            (info.rgstate[0] & STATE_SYSTEM_INVISIBLE)) return;
        RECT track = info.rcScrollBar;
        OffsetRect(&track, -window.left, -window.top);
        FillRect(dc, &track, Brush(field));
        const LONG cap = info.dxyLineButton;
        if (cap > 0)
        {
            const LONG x = (track.left + track.right)/2, y = (track.top + track.bottom)/2;
            const LONG d = std::max(2, Scale(w, 3));
            const COLORREF ink = Color(secondary);
            if (vertical)
            {
                const LONG first = track.top + cap/2, last = track.bottom - cap/2;
                Stroke(dc, ink, {x-d, first+1}, {x, first+1-d}, {x+d+1, first+2});
                Stroke(dc, ink, {x-d, last-1}, {x, last-1+d}, {x+d+1, last-2});
            }
            else
            {
                const LONG first = track.left + cap/2, last = track.right - cap/2;
                Stroke(dc, ink, {first+1,y-d}, {first+1-d,y}, {first+2,y+d+1});
                Stroke(dc, ink, {last-1,y-d}, {last-1+d,y}, {last-2,y+d+1});
            }
        }
        if (info.xyThumbBottom > info.xyThumbTop)
        {
            RECT thumb = track;
            if (vertical)
            {
                thumb.top = track.top + info.xyThumbTop;
                thumb.bottom = track.top + info.xyThumbBottom;
                InflateRect(&thumb, -Scale(w, 3), 0);
            }
            else
            {
                thumb.left = track.left + info.xyThumbTop;
                thumb.right = track.left + info.xyThumbBottom;
                InflateRect(&thumb, 0, -Scale(w, 3));
            }
            FillRect(dc, &thumb, Brush(info.rgstate[3] & STATE_SYSTEM_PRESSED ? hover : button));
        }
    }

    // Paint only the nonclient border and standard scrollbar. Windows still
    // determines thumb geometry, tracks the pointer and sends scroll messages.
    LRESULT CALLBACK Edges(HWND w, UINT msg, WPARAM wp, LPARAM lp, UINT_PTR id, DWORD_PTR)
    {
        if (msg == WM_NCDESTROY) RemoveWindowSubclass(w, Edges, id);
        const LRESULT result = DefSubclassProc(w, msg, wp, lp);
        switch (msg)
        {
        case WM_NCPAINT: case WM_PAINT: case WM_WINDOWPOSCHANGED:
        case WM_SETFOCUS: case WM_KILLFOCUS: case WM_HSCROLL: case WM_VSCROLL:
        case WM_NCMOUSEMOVE: case WM_NCLBUTTONUP: case WM_MOUSEWHEEL:
            break;
        default: return result;
        }
        RECT window{}, client{};
        if (!IsWindowVisible(w) || !GetWindowRect(w, &window) || !GetClientRect(w, &client)) return result;
        MapWindowPoints(w, nullptr, reinterpret_cast<POINT*>(&client), 2);
        if (EqualRect(&window, &client)) return result;
        OffsetRect(&client, -window.left, -window.top);
        HDC dc = GetWindowDC(w);
        const int saved = SaveDC(dc);
        ExcludeClipRect(dc, client.left, client.top, client.right, client.bottom);
        RECT area{0, 0, window.right - window.left, window.bottom - window.top};
        FillRect(dc, &area, Brush(GetFocus() == w ? accent : border));
        const auto style = GetWindowLongPtrW(w, GWL_STYLE);
        if (style & WS_VSCROLL) DrawScrollBar(w, dc, window, true);
        if (style & WS_HSCROLL) DrawScrollBar(w, dc, window, false);
        RestoreDC(dc, saved);
        ReleaseDC(w, dc);
        return result;
    }

    BOOL CALLBACK StyleControl(HWND w, LPARAM)
    {
        Face face;
        if (IsClass(w, WC_BUTTONW))
        {
            const auto type = GetWindowLongPtrW(w, GWL_STYLE) & BS_TYPEMASK;
            if (type != BS_PUSHBUTTON && type != BS_DEFPUSHBUTTON && type != BS_AUTOCHECKBOX && type != BS_CHECKBOX)
            {
                SetWindowTheme(w, L"DarkMode_Explorer", nullptr);
                return TRUE;
            }
            face = Face::Button;
        }
        else if (IsClass(w, WC_TABCONTROLW)) face = Face::Tabs;
        else if (IsClass(w, TRACKBAR_CLASSW)) face = Face::Slider;
        else if (IsClass(w, WC_HEADERW)) face = Face::Header;
        else if (IsClass(w, WC_COMBOBOXW) && (GetWindowLongPtrW(w, GWL_STYLE) & 3) == CBS_DROPDOWNLIST)
        {
            COMBOBOXINFO info{sizeof(info)};
            if (GetComboBoxInfo(w, &info) && info.hwndList)
            {
                SetWindowTheme(info.hwndList, L"DarkMode_Explorer", nullptr);
                SetWindowSubclass(info.hwndList, Edges, 1, 0);
            }
            face = Face::Combo;
        }
        else if (IsClass(w, WC_STATICW) && (GetWindowLongPtrW(w, GWL_STYLE) & SS_TYPEMASK) == SS_ETCHEDHORZ)
        {
            SetWindowSubclass(w, Edges, 1, 0);
            face = Face::Divider;
        }
        else
        {
            if (IsClass(w, WC_TREEVIEWW))
            {
                SetWindowTheme(w, L"DarkMode_Explorer", nullptr);
                TreeView_SetBkColor(w, Color(list));
                TreeView_SetTextColor(w, Color(text));
                TreeView_SetLineColor(w, Color(disabled));
            }
            else if (IsClass(w, WC_LISTVIEWW))
            {
                SetWindowTheme(w, L"DarkMode_Explorer", nullptr);
                ListView_SetBkColor(w, Color(list));
                ListView_SetTextBkColor(w, Color(list));
                ListView_SetTextColor(w, Color(text));
            }
            else if (IsClass(w, WC_EDITW) || IsClass(w, WC_LISTBOXW))
            {
                SetWindowTheme(w, L"DarkMode_Explorer", nullptr);
            }
            if (IsClass(w, WC_TREEVIEWW) || IsClass(w, WC_LISTVIEWW) || IsClass(w, WC_EDITW) || IsClass(w, WC_LISTBOXW))
            {
                SetWindowSubclass(w, Edges, 1, 0);
            }
            return TRUE;
        }
        SetWindowSubclass(w, FaceProc, 1, static_cast<DWORD_PTR>(face));
        return TRUE;
    }

    LRESULT CALLBACK DialogColors(HWND dlg, UINT msg, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR data)
    {
        COLORREF bg = static_cast<COLORREF>(data);
        auto fg = text;
        switch (msg)
        {
        case WM_ERASEBKGND:
        {
            RECT r{};
            GetClientRect(dlg, &r);
            FillRect(reinterpret_cast<HDC>(w), &r, Solid(bg));
            return 1;
        }
        case WM_CTLCOLORDLG: return reinterpret_cast<LRESULT>(Solid(bg));
        case WM_CTLCOLOREDIT: bg = Color(field); break;
        case WM_CTLCOLORLISTBOX: bg = Color(list); break;
        case WM_CTLCOLORBTN: break;
        case WM_CTLCOLORSTATIC:
        {
            HWND control = reinterpret_cast<HWND>(l);
            const int controlId = GetDlgCtrlID(control);
            if (controlId >= IDC_AXIS_X && controlId <= IDC_AXIS_Z) fg = axes[controlId - IDC_AXIS_X];
            if (IsClass(control, WC_EDITW)) bg = Color(field);
            if (!IsWindowEnabled(control)) fg = disabled;
            break;
        }
        case WM_NOTIFY:
        {
            const auto* notification = reinterpret_cast<NMHDR*>(l);
            if (notification->code == NM_CUSTOMDRAW && IsClass(notification->hwndFrom, WC_LISTVIEWW))
            {
                auto* draw = reinterpret_cast<NMLVCUSTOMDRAW*>(l);
                if (draw->nmcd.dwDrawStage == CDDS_PREPAINT) return CDRF_NOTIFYITEMDRAW;
                if (draw->nmcd.dwDrawStage == CDDS_ITEMPREPAINT)
                {
                    const auto row = static_cast<int>(draw->nmcd.dwItemSpec);
                    const UINT state = ListView_GetItemState(notification->hwndFrom, row, LVIS_SELECTED | LVIS_DROPHILITED);
                    draw->clrText = Color(text);
                    draw->clrTextBk = Color(state ? selected : (row % 2 ? alternateRow : list));
                    draw->nmcd.uItemState &= ~(CDIS_SELECTED | CDIS_DROPHILITED);
                    return CDRF_NEWFONT;
                }
            }
            if (notification->code == NM_CUSTOMDRAW && IsClass(notification->hwndFrom, WC_TREEVIEWW))
            {
                auto* draw = reinterpret_cast<NMTVCUSTOMDRAW*>(l);
                if (draw->nmcd.dwDrawStage == CDDS_PREPAINT) return CDRF_NOTIFYITEMDRAW;
                if (draw->nmcd.dwDrawStage == CDDS_ITEMPREPAINT)
                {
                    const auto item = reinterpret_cast<HTREEITEM>(draw->nmcd.dwItemSpec);
                    const bool active = TreeView_GetSelection(notification->hwndFrom) == item;
                    draw->clrText = Color(text);
                    draw->clrTextBk = Color(active ? selected : list);
                    draw->nmcd.uItemState &= ~CDIS_SELECTED;
                    return CDRF_NEWFONT;
                }
            }
            return DefSubclassProc(dlg, msg, w, l);
        }
        case WM_NCDESTROY:
            RemoveWindowSubclass(dlg, DialogColors, id);
            [[fallthrough]];
        default: return DefSubclassProc(dlg, msg, w, l);
        }
        auto dc = reinterpret_cast<HDC>(w);
        SetBkColor(dc, bg);
        SetTextColor(dc, Color(fg));
        return reinterpret_cast<LRESULT>(Solid(bg));
    }
}

void DrawPreview(HDC dc, const RECT& area, SDL_Surface* surface, SDL_Color background)
{
    FillRect(dc, &area, WindowsTheme::Brush(background));
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
    const int w = area.right - area.left;
    const int h = area.bottom - area.top;
    const double scale = std::min(double(w) / surface->w, double(h) / surface->h);
    const int iw = static_cast<int>(surface->w * scale), ih = static_cast<int>(surface->h * scale);
    StretchDIBits(dc, area.left + (w - iw) / 2, area.top + (h - ih) / 2, iw, ih,
                  0, 0, surface->w, surface->h, surface->pixels, &info, DIB_RGB_COLORS, SRCCOPY);
}

HBRUSH Brush(SDL_Color color) { return Solid(Color(color)); }

void TitleBar(HWND window)
{
    const BOOL dark = TRUE;
    const COLORREF bg = Color(EditorTheme::chrome), fg = Color(EditorTheme::text);
    DwmSetWindowAttribute(window, DWMWA_USE_IMMERSIVE_DARK_MODE, &dark, sizeof(dark));
    DwmSetWindowAttribute(window, DWMWA_CAPTION_COLOR, &bg, sizeof(bg));
    DwmSetWindowAttribute(window, DWMWA_TEXT_COLOR, &fg, sizeof(fg));
}

void ApplyDialog(HWND dialog, SDL_Color background)
{
    SetWindowTheme(dialog, L"DarkMode_Explorer", nullptr);
    SetWindowSubclass(dialog, DialogColors, 1, Color(background));
    EnumChildWindows(dialog, StyleControl, 0);
    if (GetWindowLongPtrW(dialog, GWL_STYLE) & (WS_VSCROLL | WS_HSCROLL)) SetWindowSubclass(dialog, Edges, 1, 0);
    if (!(GetWindowLongPtrW(dialog, GWL_STYLE) & WS_CHILD)) TitleBar(dialog);
}
}
