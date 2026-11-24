#pragma once
#include <SDL3/SDL.h>
#include <windows.h>
#include <optional>

class MeshEditorWindow;

// The SDL frame owns native controls and a separate child SDL/GPU viewport.
class WindowsMeshControls
{
public:
    explicit WindowsMeshControls(MeshEditorWindow& editor) : editor(editor) {}
    ~WindowsMeshControls();
    SDL_Window* Create(SDL_Window* parent);
    void Reload();
    void RefreshSlots();
    void InvalidateSlots();
    void Sync(std::size_t level);
    bool Focused() const;
    bool ContainsScreen(float x, float y) const;
    std::optional<std::size_t> SlotAtScreen(float x, float y) const;
    bool HandleMessage(MSG& message);

private:
    static LRESULT CALLBACK FrameProc(HWND, UINT, WPARAM, LPARAM, UINT_PTR, DWORD_PTR);
    static LRESULT CALLBACK SplitterProc(HWND, UINT, WPARAM, LPARAM, UINT_PTR, DWORD_PTR);
    static LRESULT CALLBACK ViewProc(HWND, UINT, WPARAM, LPARAM);
    static INT_PTR CALLBACK PanelProc(HWND, UINT, WPARAM, LPARAM);
    int Scale(int value) const;
    void Layout();
    void UpdateFont();
    void UpdateRemove();
    void RemoveMaterial();
    void CancelResize(bool restore);
    void PaintSlot(const DRAWITEMSTRUCT& draw);
    MeshEditorWindow& editor;
    HWND frame = nullptr, view = nullptr, panel = nullptr, splitter = nullptr, hint = nullptr;
    SDL_Window* viewport = nullptr;
    HFONT font = nullptr;
    bool resizing = false, menuActive = false;
    float oldWidth = 280;
    int grabOffset = 0;
};
