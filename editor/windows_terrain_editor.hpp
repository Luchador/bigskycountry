#pragma once

#include <windows.h>
#include <functional>

// Modeless tools for terrain in the main editor viewport.
class WindowsTerrainEditor
{
public:
    ~WindowsTerrainEditor();
    bool Open(HWND owner, std::function<void()> onStateChanged);
    void Close();
    HWND Window() const { return window; }

private:
    static INT_PTR CALLBACK Dialog(HWND, UINT, WPARAM, LPARAM);
    void Layout();
    HWND window = nullptr;
    std::function<void()> stateChanged;
};
