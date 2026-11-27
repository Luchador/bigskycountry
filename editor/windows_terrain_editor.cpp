#include "windows_terrain_editor.hpp"
#include "windows_theme.hpp"
#include "panel_ids.h"
#include <algorithm>
#include <utility>

namespace
{
    SIZE Units(HWND dialog, int width, int height)
    {
        RECT rect{0, 0, width, height};
        MapDialogRect(dialog, &rect);
        return {rect.right, rect.bottom};
    }
}

WindowsTerrainEditor::~WindowsTerrainEditor() { Close(); }

bool WindowsTerrainEditor::Open(HWND owner, std::function<void()> onStateChanged)
{
    if (!window)
    {
        stateChanged = std::move(onStateChanged);
        if (!CreateDialogParamW(GetModuleHandleW(nullptr), MAKEINTRESOURCEW(IDD_TERRAIN_EDITOR), owner,
                                Dialog, reinterpret_cast<LPARAM>(this)))
            return SDL_SetError("Cannot open Terrain Editor (Windows error %lu)", GetLastError());
    }
    ShowWindow(window, IsIconic(window) ? SW_RESTORE : SW_SHOW);
    SetForegroundWindow(window);
    SetFocus(GetDlgItem(window, IDCANCEL));
    if (stateChanged) stateChanged();
    return true;
}

void WindowsTerrainEditor::Close()
{
    if (window) DestroyWindow(window);
}

void WindowsTerrainEditor::Layout()
{
    RECT area{};
    GetClientRect(window, &area);
    const auto pad = Units(window, 8, 8), button = Units(window, 54, 16);
    const int width = std::max(1L, area.right - 2 * pad.cx);
    const int groupHeight = std::max(1L, area.bottom - 3 * pad.cy - button.cy);
    MoveWindow(GetDlgItem(window, IDC_TERRAIN_TOOLS), pad.cx, pad.cy, width, groupHeight, TRUE);
    MoveWindow(GetDlgItem(window, IDC_TERRAIN_EMPTY), 2 * pad.cx, 3 * pad.cy,
               std::max(1L, width - 2 * pad.cx), Units(window, 0, 24).cy, TRUE);
    MoveWindow(GetDlgItem(window, IDCANCEL), area.right - pad.cx - button.cx,
               area.bottom - pad.cy - button.cy, button.cx, button.cy, TRUE);
}

INT_PTR CALLBACK WindowsTerrainEditor::Dialog(HWND hwnd, UINT message, WPARAM w, LPARAM l)
{
    if (message == WM_INITDIALOG)
    {
        auto& self = *reinterpret_cast<WindowsTerrainEditor*>(l);
        self.window = hwnd;
        SetWindowLongPtrW(hwnd, DWLP_USER, l);
        WindowsTheme::ApplyDialog(hwnd);
        self.Layout();
        return TRUE;
    }
    auto* self = reinterpret_cast<WindowsTerrainEditor*>(GetWindowLongPtrW(hwnd, DWLP_USER));
    if (!self) return FALSE;
    switch (message)
    {
    case WM_SIZE:
        if (w != SIZE_MINIMIZED) self->Layout();
        return TRUE;
    case WM_GETMINMAXINFO:
    {
        const auto minimum = Units(hwnd, 220, 160);
        reinterpret_cast<MINMAXINFO*>(l)->ptMinTrackSize = {minimum.cx, minimum.cy};
        return TRUE;
    }
    case WM_COMMAND:
        if (LOWORD(w) != IDCANCEL) return FALSE;
        [[fallthrough]];
    case WM_CLOSE:
    {
        const HWND owner = GetWindow(hwnd, GW_OWNER);
        self->Close();
        if (IsWindow(owner)) SetFocus(owner);
        return TRUE;
    }
    case WM_NCDESTROY:
        self->window = nullptr;
        SetWindowLongPtrW(hwnd, DWLP_USER, 0);
        if (self->stateChanged) self->stateChanged();
        break;
    }
    return FALSE;
}
