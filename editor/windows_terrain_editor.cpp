#include "windows_terrain_editor.hpp"
#include "windows_theme.hpp"
#include "panel_ids.h"
#include "asset_paths.hpp"
#include <cmath>
#include <cwchar>
#include <memory>
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

WindowsTerrainEditor::~WindowsTerrainEditor()
{
    Close();
    for (auto icon : icons) DeleteObject(icon);
}

bool WindowsTerrainEditor::Open(HWND owner, std::function<void()> onStateChanged)
{
    if (!window)
    {
        stateChanged = std::move(onStateChanged);
        if (!CreateDialogParamW(GetModuleHandleW(nullptr), MAKEINTRESOURCEW(IDD_TERRAIN_EDITOR), owner,
                                Dialog, reinterpret_cast<LPARAM>(this)))
            return SDL_SetError("Cannot open Terrain Editor (Windows error %lu)", GetLastError());
    }
    settings.enabled = true;
    Sync();
    ShowWindow(window, IsIconic(window) ? SW_RESTORE : SW_SHOW);
    SetForegroundWindow(window);
    SetFocus(GetDlgItem(window, IDC_TERRAIN_RAISE));
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
    MoveWindow(GetDlgItem(window, IDCANCEL), area.right - pad.cx - button.cx,
               area.bottom - pad.cy - button.cy, button.cx, button.cy, TRUE);
    // Let the themed dialog paint all exposed space, including the old button
    // position after a resize. No overlapping group box should clip its fill.
    InvalidateRect(window, nullptr, TRUE);
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
        const int size = Units(hwnd, 26, 0).cx;
        constexpr std::array names{"btn_terrain_off.png", "btn_terrain_on.png", "btn_terrain_lower_off.png",
                                   "btn_terrain_lower_on.png", "btn_terrain_flatten_off.png", "btn_terrain_flatten_on.png"};
        for (std::size_t i = 0; i < names.size(); ++i)
        {
            DeleteObject(self.icons[i]);
            const auto path = AssetPaths::Root(AssetRoot::Project) + "textures/editor/" + names[i];
            self.icons[i] = WindowsTheme::ButtonBitmap(SDL_LoadPNG(path.c_str()), size);
        }
        self.Sync();
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
        const auto minimum = Units(hwnd, 260, 180);
        RECT frame{0, 0, minimum.cx, minimum.cy};
        AdjustWindowRectEx(&frame, GetWindowLongPtrW(hwnd, GWL_STYLE), FALSE,
                          GetWindowLongPtrW(hwnd, GWL_EXSTYLE));
        reinterpret_cast<MINMAXINFO*>(l)->ptMinTrackSize = {frame.right - frame.left, frame.bottom - frame.top};
        return TRUE;
    }
    case WM_COMMAND:
        if (LOWORD(w) >= IDC_TERRAIN_RAISE && LOWORD(w) <= IDC_TERRAIN_FLATTEN && HIWORD(w) == BN_CLICKED)
        {
            self->ReadSettings();
            self->settings.tool = static_cast<TerrainTool>(LOWORD(w) - IDC_TERRAIN_RAISE);
            self->settings.enabled = true;
            self->Sync();
            return TRUE;
        }
        if (LOWORD(w) == IDC_TERRAIN_ENABLED && HIWORD(w) == BN_CLICKED)
        {
            self->settings.enabled = IsDlgButtonChecked(hwnd, IDC_TERRAIN_ENABLED) == BST_CHECKED;
            self->Sync();
            return TRUE;
        }
        if (LOWORD(w) == IDOK || ((LOWORD(w) >= IDC_TERRAIN_RADIUS && LOWORD(w) <= IDC_TERRAIN_SEAM) && HIWORD(w) == EN_KILLFOCUS))
        {
            self->ReadSettings();
            self->Sync();
            return TRUE;
        }
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
        self->settings.enabled = false;
        self->window = nullptr;
        SetWindowLongPtrW(hwnd, DWLP_USER, 0);
        if (self->stateChanged) self->stateChanged();
        break;
    }
    return FALSE;
}

void WindowsTerrainEditor::Sync()
{
    if (!window) return;
    constexpr std::array names{L"Raise", L"Lower", L"Flatten"};
    for (int i = 0; i < 3; ++i)
    {
        HWND button = GetDlgItem(window, IDC_TERRAIN_RAISE + i);
        const auto icon = icons[2 * i + (settings.enabled && static_cast<int>(settings.tool) == i)];
        const auto style = GetWindowLongPtrW(button, GWL_STYLE);
        SetWindowLongPtrW(button, GWL_STYLE, icon ? style | BS_BITMAP : style & ~BS_BITMAP);
        SetWindowTextW(button, names[i]);
        SendMessageW(button, BM_SETIMAGE, IMAGE_BITMAP, reinterpret_cast<LPARAM>(icon));
        InvalidateRect(button, nullptr, TRUE);
    }
    CheckDlgButton(window, IDC_TERRAIN_ENABLED, settings.enabled ? BST_CHECKED : BST_UNCHECKED);
    for (const auto& [id, value] : {std::pair{IDC_TERRAIN_RADIUS, settings.radius},
                                  std::pair{IDC_TERRAIN_STRENGTH, settings.strength}, std::pair{IDC_TERRAIN_SEAM, settings.seamFalloff}})
    {
        wchar_t text[64]; std::swprintf(text, 64, L"%.4g", value);
        SetDlgItemTextW(window, id, text);
    }
}
void WindowsTerrainEditor::ReadSettings()
{
    const auto read = [&](int id, double& value, double low, double high)
    {
        wchar_t text[64]; GetDlgItemTextW(window, id, text, 64);
        wchar_t* end = nullptr;
        const double next = std::wcstod(text, &end);
        if (end != text && *end == 0 && std::isfinite(next)) value = std::clamp(next, low, high);
    };
    read(IDC_TERRAIN_RADIUS, settings.radius, 0.1, 1000);
    read(IDC_TERRAIN_STRENGTH, settings.strength, 0.01, 100);
    read(IDC_TERRAIN_SEAM, settings.seamFalloff, 0.1, 1000);
}
void WindowsTerrainEditor::SetRadius(double radius)
{
    settings.radius = std::clamp(radius, 0.1, 1000.0);
    Sync();
}
void WindowsTerrainEditor::Disable()
{
    settings.enabled = false;
    Sync();
}
