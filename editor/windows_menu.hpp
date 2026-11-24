#pragma once
#include <windows.h>
#include <SDL3/SDL.h>
#include <span>

// Native menus shared by the main editor and its utility windows.
namespace WindowsMenu
{
    struct MenuItem
    {
        UINT id;
        const wchar_t* label;
        UINT flags = MF_ENABLED;
        bool bar = false;
        bool checkable = false;
    };
    // Item labels/data must outlive the menu, including popup tracking.
    void MenuBackground(HMENU menu, SDL_Color color);
    bool Insert(HMENU menu, const MenuItem& item, HMENU submenu = nullptr);
    bool AddSubmenu(HMENU parent, const MenuItem& title, std::span<const MenuItem> items);
    bool HandleMessage(HWND window, UINT message, WPARAM w, LPARAM l, LRESULT& result);
    void PaintMenuBarFrame(HWND window);
}
