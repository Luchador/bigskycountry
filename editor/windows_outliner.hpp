#pragma once

#include "scene_outliner.hpp"
#include <SDL3/SDL.h>
#include <windows.h>
#include <deque>

class WindowsOutliner
{
  public:
    HWND Create(HWND frame, HINSTANCE module);
    void Layout(int x, int y, int width, int height);
    void Sync(const SceneOutliner& model);
    std::optional<OutlineCommand> Take(const SDL_Event& event);
    bool QueueTerrainFill(bool repeat);
    HWND Window() const { return panel; }

  private:
    static INT_PTR CALLBACK Dialog(HWND, UINT, WPARAM, LPARAM);
    void Queue(OutlineCommand cmd);
    std::set<SceneKey> Selected() const;
    void Name(bool create);
    void BeginDrag(int row);
    void EndDrag();
    void Hover(POINT p, bool scroll = true);
    void SetDrop(std::optional<OutlineDrop> next);
    static LRESULT CALLBACK DrawInsertion(HWND, UINT, WPARAM, LPARAM, UINT_PTR, DWORD_PTR);
    static LRESULT CALLBACK BadgeProc(HWND, UINT, WPARAM, LPARAM, UINT_PTR, DWORD_PTR);

    HWND panel = nullptr, list = nullptr, badge = nullptr;
    using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;
    Surface checkIcon{nullptr, SDL_DestroySurface}, blockIcon{nullptr, SDL_DestroySurface};
    std::wstring dragLabel;
    SIZE badgeSize{};
    int iconSize = 16, padding = 3;
    const SceneOutliner* model = nullptr;
    std::vector<OutlineRow> rows;
    std::set<SceneKey> displayed, dragged;
    std::deque<OutlineCommand> pending;
    Uint32 eventType = 0;
    unsigned version = 0;
    std::optional<OutlineDrop> drop;
    bool syncing = false;
};
