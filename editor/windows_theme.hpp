#pragma once

#include "editor_theme.hpp"
#include <windows.h>
#include <optional>

namespace WindowsTheme
{
    constexpr COLORREF Color(SDL_Color c) { return RGB(c.r, c.g, c.b); }
    HBRUSH Brush(SDL_Color color);
    void DrawColorButton(const DRAWITEMSTRUCT& draw, const wchar_t* label, COLORREF color);
    std::optional<COLORREF> PickColor(HWND owner, COLORREF current);
    void DrawPreview(HDC dc, const RECT& area, SDL_Surface* surface, SDL_Color background = EditorTheme::preview);
    void TitleBar(HWND window);
    // Call during WM_INITDIALOG. Control input, focus, notifications and accessibility stay native.
    void ApplyDialog(HWND dialog, SDL_Color background = EditorTheme::panel);
}
