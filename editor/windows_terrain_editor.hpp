#pragma once

#include <windows.h>
#include <functional>
#include <array>
#include "terrain_sculpt.hpp"

// Modeless tools for terrain in the main editor viewport.
class WindowsTerrainEditor
{
public:
    ~WindowsTerrainEditor();
    bool Open(HWND owner, std::function<void()> onStateChanged);
    void Close();
    HWND Window() const { return window; }
    const TerrainBrushSettings& Settings() const { return settings; }
    void SetRadius(double radius);
    void Disable();

private:
    static INT_PTR CALLBACK Dialog(HWND, UINT, WPARAM, LPARAM);
    void Layout();
    void Sync();
    void ReadSettings();
    TerrainBrushSettings settings;
    std::array<HBITMAP, 6> icons{};
    HWND window = nullptr;
    std::function<void()> stateChanged;
};
