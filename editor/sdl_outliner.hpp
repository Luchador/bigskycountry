#pragma once
#include "scene_outliner.hpp"
#include <SDL3/SDL.h>

class SdlOutliner
{
  public:
    void LoadIcons(SDL_Renderer* renderer);
    bool DrawDragBadge(SDL_Renderer* renderer, SDL_Rect panel) const;
    bool Draw(SDL_Renderer* renderer, const SceneOutliner& model, int width, int height);
    bool Key(SDL_Window* window, const SDL_Event& event, const SceneOutliner& model,
             std::optional<OutlineCommand>& cmd);
    void Pointer(SDL_Window* window, const SDL_Event& e, SDL_FPoint p, int width, int height,
                 const SceneOutliner& model, std::optional<OutlineCommand>& cmd);
    void Blur(SDL_Window* window);
    bool Focused() const { return focused; }
    bool Tracking() const { return pressed.has_value(); }

  private:
    void StartName(SDL_Window* window, bool create, const SceneOutliner& model);
    bool focused = false, dragging = false, singleOnRelease = false, selectText = false;
    int scroll = 0;
    std::optional<SceneKey> anchor, pressed, drop, previousClick;
    std::optional<OutlineCommand> edit;
    std::string text;
    SDL_FPoint start{}, cursor{};
    std::set<SceneKey> dragged;
    std::string dragLabel;
    SDL_Texture* checkIcon = nullptr;
    SDL_Texture* blockIcon = nullptr; // Both textures are owned by the main renderer.
    std::optional<SceneKey> lastSingle;
};
