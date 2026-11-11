#pragma once

#include <SDL3/SDL.h>
#include <memory>
#include "scene_outliner.hpp"

struct PropertiesInspector;
class ContentBrowser;
class FreeCamera;
struct RoadEditorState;
struct Road;
enum class RoadAction;
class TransformSettings;

// The top-level SDL window receives viewport input. A separate sibling HWND
// owns the swapchain so presenting a frame can never cover a native control.
class WindowsPanels
{
public:
    WindowsPanels();
    ~WindowsPanels();
    SDL_Window* Create(SDL_Window* frame, ContentBrowser& browser);
    bool ViewportFocused() const;
    void ToViewport(SDL_Window* frame, SDL_Event& event, const SDL_Rect& scene) const;
    struct ModelDrop
    {
        std::string path;
        SDL_FPoint viewportFraction;
    };
    std::optional<ModelDrop> TakeModelDrop(const SDL_Event& event);
    struct MaterialDrop
    {
        std::string path;
        std::optional<SDL_FPoint> viewportFraction; // Empty when dropped on the inspector.
        std::optional<TransformTarget> inspectorTarget;
    };
    std::optional<MaterialDrop> TakeMaterialDrop(const SDL_Event& event);
    bool Command(const SDL_Event& event, RoadAction& action) const;
    bool HandleInput(const SDL_Event& event, FreeCamera& camera, TransformSettings& settings);
    SDL_Keymod TakePressModifiers();
    std::optional<InspectorEdit> TakeInspectorEdit(const SDL_Event& event);
    void SyncOutliner(const SceneOutliner& model);
    std::optional<OutlineCommand> TakeOutlineCommand(const SDL_Event& event);
    void Sync(const RoadEditorState& state, const Road* current, bool plan, const FreeCamera& camera, const TransformSettings& settings, const TransformFields& fields, const PropertiesInspector& properties);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
