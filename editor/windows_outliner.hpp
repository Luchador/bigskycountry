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
    HWND Window() const { return panel; }

  private:
    static INT_PTR CALLBACK Dialog(HWND, UINT, WPARAM, LPARAM);
    void Queue(OutlineCommand cmd);
    std::set<SceneKey> Selected() const;
    void Name(bool create);
    void Hover(POINT p);

    HWND panel = nullptr, list = nullptr;
    const SceneOutliner* model = nullptr;
    std::vector<OutlineRow> rows;
    std::set<SceneKey> displayed, dragged;
    std::deque<OutlineCommand> pending;
    Uint32 eventType = 0;
    unsigned version = 0;
    int drop = -1;
    bool syncing = false;
};
