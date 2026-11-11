#pragma once
#include "material_editor.hpp"
#include <windows.h>
#include <optional>

class WindowsMaterialEditor
{
public:
    bool Open(HWND owner, TextureCatalog& assets, const std::string& path);
    HWND Window() const { return window; }
    std::optional<MaterialSlot> SlotAt(POINT screen) const;
    void Hover(std::optional<MaterialSlot> slot);
    void Drop(MaterialSlot slot, const std::string& path);
    void RefreshImages();

private:
    static INT_PTR CALLBACK Dialog(HWND, UINT, WPARAM, LPARAM);
    void Layout();
    void Sync();
    void Paint(const DRAWITEMSTRUCT& draw);
    MaterialEditor editor;
    HWND window = nullptr;
    std::optional<MaterialSlot> hover;
};
