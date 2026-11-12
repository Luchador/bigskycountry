#include "windows_material_editor.hpp"
#include "windows_name_dialog.hpp"
#include "windows_theme.hpp"
#include "panel_ids.h"
#include <algorithm>
#include <commctrl.h>
#include <commdlg.h>
#include <cmath>
#include <windowsx.h>

namespace
{
    int Units(HWND window, int units, bool horizontal)
    {
        RECT r{0, 0, units, units};
        MapDialogRect(window, &r);
        return horizontal ? r.right : r.bottom;
    }
}

bool WindowsMaterialEditor::Open(HWND owner, TextureCatalog& assets, const std::string& path)
{
    if (!editor.Open(assets, path))
        return false;
    if (!window)
    {
        if (!CreateDialogParamW(GetModuleHandleW(nullptr), MAKEINTRESOURCEW(IDD_MATERIAL), owner, Dialog,
                                reinterpret_cast<LPARAM>(this)))
            return SDL_SetError("Cannot open the material editor");
        if (!SetWindowSubclass(GetDlgItem(window, IDC_MAT_PREVIEW), PreviewInput, 1, reinterpret_cast<DWORD_PTR>(this)))
        {
            DestroyWindow(window);
            return SDL_SetError("Cannot initialize material preview input");
        }
        RECT frame{}, bounds{};
        GetWindowRect(owner, &frame);
        GetWindowRect(window, &bounds);
        SetWindowPos(window, nullptr,
                     frame.left + std::max(0L, (frame.right - frame.left - (bounds.right - bounds.left)) / 2),
                     frame.top + std::max(0L, (frame.bottom - frame.top - (bounds.bottom - bounds.top)) / 2), 0, 0,
                     SWP_NOSIZE | SWP_NOZORDER);
    }
    hover.reset();
    SetWindowTextW(window, WindowsNameDialog::Wide("Material Editor - " + path).c_str());
    Sync();
    ShowWindow(window, SW_SHOWNORMAL);
    SetForegroundWindow(window);
    return true;
}

void WindowsMaterialEditor::Layout()
{
    RECT area{};
    GetClientRect(window, &area);
    const int pad = Units(window, 6, true), side = Units(window, 144, true);
    const int split = std::max<int>(1, area.right - side);
    const int x = split + pad, width = side - 2 * pad;
    const int top = Units(window, 42, false), clear = Units(window, 28, true);
    const int step = std::min(Units(window, 46, false), std::max(1, (int(area.bottom) - top - pad) / 5));
    MoveWindow(GetDlgItem(window, IDC_MAT_PREVIEW), 0, 0, split, area.bottom, FALSE);
    MoveWindow(GetDlgItem(window, IDC_MAT_LABEL), x, pad, width, Units(window, 11, false), FALSE);
    MoveWindow(GetDlgItem(window, IDC_MAT_TYPE), x, Units(window, 19, false), width, Units(window, 60, false), FALSE);
    int row = 0;
    const bool unlit = editor.Current().type == MaterialType::Unlit;
    for (int id : {IDC_MAT_TINT, IDC_MAT_FACTOR_LABEL, IDC_MAT_FACTOR})
        ShowWindow(GetDlgItem(window, id), unlit ? SW_SHOWNA : SW_HIDE);
    for (int i = 0; i < int(materialSlotCount); ++i)
    {
        const bool visible = editor.Current().Uses(static_cast<MaterialSlot>(i));
        ShowWindow(GetDlgItem(window, IDC_MAT_SLOT_FIRST + i), visible ? SW_SHOWNA : SW_HIDE);
        ShowWindow(GetDlgItem(window, IDC_MAT_CLEAR_FIRST + i), visible ? SW_SHOWNA : SW_HIDE);
        if (!visible) continue;
        const int y = top + step * row++;
        MoveWindow(GetDlgItem(window, IDC_MAT_SLOT_FIRST + i), x, y, width - clear - pad, std::max(1, step - pad),
                   FALSE);
        const int h = Units(window, 15, false);
        MoveWindow(GetDlgItem(window, IDC_MAT_CLEAR_FIRST + i), x + width - clear, y + (step - pad - h) / 2, clear, h,
                   FALSE);
        if (unlit && i == int(MaterialSlot::Emissive))
            MoveWindow(GetDlgItem(window, IDC_MAT_TINT), x, top + step * row++, width, Units(window, 20, false), FALSE);
    }
    if (unlit)
    {
        const int y = top + step * row;
        MoveWindow(GetDlgItem(window, IDC_MAT_FACTOR_LABEL), x, y, width, Units(window, 12, false), FALSE);
        MoveWindow(GetDlgItem(window, IDC_MAT_FACTOR), x, y + Units(window, 14, false), width, Units(window, 18, false), FALSE);
    }
    // Erase the old control positions and paint the whole layout during the native resize loop.
    RedrawWindow(window, nullptr, nullptr, RDW_INVALIDATE | RDW_ERASE | RDW_ALLCHILDREN | RDW_UPDATENOW);
}

void WindowsMaterialEditor::Sync()
{
    SendDlgItemMessageW(window, IDC_MAT_TYPE, CB_SETCURSEL, int(editor.Current().type), 0);
    for (int i = 0; i < int(materialSlotCount); ++i)
    {
        const auto& path = editor.Current().textures[i];
        const auto name =
            WindowsNameDialog::Wide(std::string(materialSlotLabels[i]) + " - " + (path.empty() ? "Unassigned" : path));
        SetDlgItemTextW(window, IDC_MAT_SLOT_FIRST + i, name.c_str());
        EnableWindow(GetDlgItem(window, IDC_MAT_CLEAR_FIRST + i), !path.empty());
    }
    SyncFactor();
    Layout();
}

void WindowsMaterialEditor::SyncFactor()
{
    wchar_t label[40];
    swprintf(label, 40, L"Translucency %.2f", editor.Translucency());
    SetDlgItemTextW(window, IDC_MAT_FACTOR_LABEL, label);
    SendDlgItemMessageW(window, IDC_MAT_FACTOR, TBM_SETPOS, TRUE, std::lround(editor.Translucency() * 100));
}

void WindowsMaterialEditor::RefreshImages()
{
    if (window && editor.ImagesChanged())
        Sync();
}

std::optional<MaterialSlot> WindowsMaterialEditor::SlotAt(POINT screen) const
{
    if (!window)
        return {};
    HWND hit = WindowFromPoint(screen);
    if (GetParent(hit) != window)
        return {};
    const int i = GetDlgCtrlID(hit) - IDC_MAT_SLOT_FIRST;
    if (i < 0 || i >= int(materialSlotCount))
        return {};
    return static_cast<MaterialSlot>(i);
}

void WindowsMaterialEditor::Hover(std::optional<MaterialSlot> slot)
{
    if (window && hover != slot)
    {
        if (hover)
            InvalidateRect(GetDlgItem(window, IDC_MAT_SLOT_FIRST + int(*hover)), nullptr, FALSE);
        if (slot)
            InvalidateRect(GetDlgItem(window, IDC_MAT_SLOT_FIRST + int(*slot)), nullptr, FALSE);
    }
    hover = slot;
}

void WindowsMaterialEditor::Drop(MaterialSlot slot, const std::string& path)
{
    if (!editor.SetTexture(slot, path))
        MessageBoxW(window, WindowsNameDialog::Wide(SDL_GetError()).c_str(), L"Material Editor",
                    MB_OK | MB_ICONWARNING);
    Sync();
}

LRESULT CALLBACK WindowsMaterialEditor::PreviewInput(HWND view, UINT message, WPARAM w, LPARAM l,
                                                      UINT_PTR id, DWORD_PTR data)
{
    auto& self = *reinterpret_cast<WindowsMaterialEditor*>(data);
    switch (message)
    {
    case WM_ERASEBKGND:
        // The owner-drawn frame covers the entire viewport.
        return 1;
    case WM_LBUTTONDOWN:
        SetFocus(view);
        self.lastOrbitPoint = {GET_X_LPARAM(l), GET_Y_LPARAM(l)};
        SetCapture(view);
        return 0;
    case WM_MOUSEMOVE:
    case WM_LBUTTONUP:
        if (GetCapture() == view)
        {
            if ((w & MK_LBUTTON) || message == WM_LBUTTONUP)
            {
                const POINT point{GET_X_LPARAM(l), GET_Y_LPARAM(l)};
                RECT bounds{};
                GetClientRect(view, &bounds);
                if (self.editor.Orbit(point.x - self.lastOrbitPoint.x, point.y - self.lastOrbitPoint.y,
                                      std::min(bounds.right, bounds.bottom)))
                    InvalidateRect(view, nullptr, FALSE);
                self.lastOrbitPoint = point;
            }
            if (message == WM_LBUTTONUP || !(w & MK_LBUTTON)) ReleaseCapture();
            return 0;
        }
        break;
    case WM_CANCELMODE:
    case WM_KILLFOCUS:
        if (GetCapture() == view) ReleaseCapture();
        break;
    case WM_NCDESTROY:
        RemoveWindowSubclass(view, PreviewInput, id);
        break;
    }
    return DefSubclassProc(view, message, w, l);
}

void WindowsMaterialEditor::Paint(const DRAWITEMSTRUCT& draw)
{
    if (draw.CtlID == IDC_MAT_TINT)
    {
        RECT area = draw.rcItem;
        FillRect(draw.hDC, &area, WindowsTheme::Brush(EditorTheme::button));
        InflateRect(&area, -4, -3);
        SetBkMode(draw.hDC, TRANSPARENT);
        SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::text));
        DrawTextW(draw.hDC, L"Emissive tint", -1, &area, DT_SINGLELINE | DT_VCENTER);
        RECT swatch = area;
        swatch.left = swatch.right - Units(window, 32, true);
        const auto& c = editor.Current().emissiveTint;
        SetDCBrushColor(draw.hDC, RGB(std::lround(c[0]*255), std::lround(c[1]*255), std::lround(c[2]*255)));
        FillRect(draw.hDC, &swatch, static_cast<HBRUSH>(GetStockObject(DC_BRUSH)));
        if (draw.itemState & ODS_FOCUS) DrawFocusRect(draw.hDC, &draw.rcItem);
        return;
    }
    if (draw.CtlID == IDC_MAT_PREVIEW)
    {
        const RECT& view = draw.rcItem;
        auto* pixels = editor.Preview(view.right - view.left, view.bottom - view.top);
        WindowsTheme::DrawPreview(draw.hDC, view, pixels, EditorTheme::field);
        return;
    }
    const unsigned slot = draw.CtlID - IDC_MAT_SLOT_FIRST;
    if (slot >= materialSlotCount)
        return;
    const bool active = hover && unsigned(*hover) == slot;
    const RECT r = draw.rcItem;
    FillRect(draw.hDC, &r, WindowsTheme::Brush(active ? EditorTheme::selected : EditorTheme::list));
    const int pad = Units(window, 3, true);
    RECT icon{r.left + pad, r.top + pad, r.left + r.bottom - r.top - pad, r.bottom - pad};
    WindowsTheme::DrawPreview(draw.hDC, icon, editor.Thumbnail(slot));
    SetBkMode(draw.hDC, TRANSPARENT);
    SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::text));
    const auto& path = editor.Current().textures[slot];
    RECT line{icon.right + pad, r.top, r.right - pad, (r.top + r.bottom) / 2};
    constexpr UINT flags = DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX;
    DrawTextW(draw.hDC, WindowsNameDialog::Wide(materialSlotLabels[slot]).c_str(), -1, &line, flags);
    line.top = line.bottom;
    line.bottom = r.bottom;
    SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::secondary));
    const auto name = path.empty() ? std::string("Drop texture") : path.substr(path.rfind('/') + 1);
    DrawTextW(draw.hDC, WindowsNameDialog::Wide(name).c_str(), -1, &line, flags);
}

INT_PTR CALLBACK WindowsMaterialEditor::Dialog(HWND hwnd, UINT message, WPARAM w, LPARAM l)
{
    if (message == WM_INITDIALOG)
    {
        auto& self = *reinterpret_cast<WindowsMaterialEditor*>(l);
        SetWindowLongPtrW(hwnd, DWLP_USER, l);
        self.window = hwnd;
        WindowsTheme::ApplyDialog(hwnd);
        SendDlgItemMessageW(hwnd, IDC_MAT_FACTOR, TBM_SETRANGE, FALSE, MAKELPARAM(0, 100));
        for (const auto& type : materialTypes)
        {
            const auto label = WindowsNameDialog::Wide(type.label);
            SendDlgItemMessageW(hwnd, IDC_MAT_TYPE, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
        }
        self.Layout();
        return TRUE;
    }
    auto* self = reinterpret_cast<WindowsMaterialEditor*>(GetWindowLongPtrW(hwnd, DWLP_USER));
    if (!self)
        return FALSE;
    switch (message)
    {
    case WM_SIZE:
        if (w != SIZE_MINIMIZED) self->Layout();
        return TRUE;
    case WM_GETMINMAXINFO: {
        auto* limits = reinterpret_cast<MINMAXINFO*>(l);
        limits->ptMinTrackSize = {Units(hwnd, 270, true), Units(hwnd, 225, false)};
        return TRUE;
    }
    case WM_DRAWITEM:
        self->Paint(*reinterpret_cast<DRAWITEMSTRUCT*>(l));
        return TRUE;
    case WM_HSCROLL:
        if (reinterpret_cast<HWND>(l) != GetDlgItem(hwnd, IDC_MAT_FACTOR)) return FALSE;
        {
            const float value = SendDlgItemMessageW(hwnd, IDC_MAT_FACTOR, TBM_GETPOS, 0, 0) / 100.f;
            if (LOWORD(w) == TB_THUMBTRACK) self->editor.PreviewTranslucency(value);
            else if (!self->editor.SetTranslucency(value))
                MessageBoxW(hwnd, WindowsNameDialog::Wide(SDL_GetError()).c_str(), L"Material Editor", MB_OK | MB_ICONWARNING);
            self->SyncFactor();
            InvalidateRect(GetDlgItem(hwnd, IDC_MAT_PREVIEW), nullptr, FALSE);
        }
        return TRUE;
    case WM_COMMAND:
        if (LOWORD(w) == IDC_MAT_TINT && HIWORD(w) == BN_CLICKED)
        {
            static COLORREF custom[16]{};
            const auto& c = self->editor.Current().emissiveTint;
            CHOOSECOLORW picker{sizeof(picker)};
            picker.hwndOwner = hwnd;
            picker.lpCustColors = custom;
            picker.rgbResult = RGB(std::lround(c[0]*255), std::lround(c[1]*255), std::lround(c[2]*255));
            picker.Flags = CC_FULLOPEN | CC_RGBINIT;
            if (ChooseColorW(&picker))
            {
                const auto rgb = picker.rgbResult;
                if (!self->editor.SetTint({GetRValue(rgb)/255.f, GetGValue(rgb)/255.f, GetBValue(rgb)/255.f}))
                    MessageBoxW(hwnd, WindowsNameDialog::Wide(SDL_GetError()).c_str(), L"Material Editor", MB_OK | MB_ICONWARNING);
                self->Sync();
            }
            return TRUE;
        }
        if (LOWORD(w) == IDC_MAT_TYPE && HIWORD(w) == CBN_SELCHANGE)
        {
            const auto i = SendDlgItemMessageW(hwnd, IDC_MAT_TYPE, CB_GETCURSEL, 0, 0);
            if (i >= 0 && !self->editor.SetType(static_cast<MaterialType>(i)))
                MessageBoxW(hwnd, WindowsNameDialog::Wide(SDL_GetError()).c_str(), L"Material Editor",
                            MB_OK | MB_ICONWARNING);
            self->Sync();
            return TRUE;
        }
        if (LOWORD(w) >= IDC_MAT_CLEAR_FIRST && LOWORD(w) < IDC_MAT_CLEAR_FIRST + materialSlotCount &&
            HIWORD(w) == BN_CLICKED)
        {
            self->Drop(static_cast<MaterialSlot>(LOWORD(w) - IDC_MAT_CLEAR_FIRST), {});
            return TRUE;
        }
        if (LOWORD(w) != IDCANCEL)
            return FALSE;
        [[fallthrough]];
    case WM_CLOSE:
        DestroyWindow(hwnd);
        return TRUE;
    case WM_NCDESTROY:
        self->window = nullptr;
        self->hover.reset();
        return FALSE;
    default:
        return FALSE;
    }
}
