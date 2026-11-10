#pragma once

#include "editor_theme.hpp"
#include <windows.h>

namespace WindowsTheme
{
    constexpr COLORREF Color(SDL_Color c) { return RGB(c.r, c.g, c.b); }
    HBRUSH Brush(SDL_Color color);
    void TitleBar(HWND window);
    // Call during WM_INITDIALOG. Control input, focus, notifications and accessibility stay native.
    void ApplyDialog(HWND dialog, SDL_Color background = EditorTheme::panel);
}
