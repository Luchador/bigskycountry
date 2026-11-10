#pragma once

#include "application.hpp"
#include "road_action.hpp"
#include "scene_outliner.hpp"
#ifdef _WIN32
#include "windows_panels.hpp"
#endif
#ifndef _WIN32
#include "camera_toolbar.hpp"
#include "sdl_transform_panel.hpp"
#include "sdl_outliner.hpp"
#endif
#include "content_browser.hpp"
#include "edit_history.hpp"
#include "road_overlay.hpp"
#include "viewport_picker.hpp"
#include "transform_gizmo.hpp"
#include "editor_menu.hpp"
#include <SDL3/SDL.h>

class RoadEditor : public EditorTool
{
public:
#ifdef _WIN32
    SDL_Window* CreateRenderWindow(SDL_Window* frame) override { return panels.Create(frame, browser); }
    bool ViewportFocused() const override { return panels.ViewportFocused(); }
#else
    bool ViewportFocused() const override { return !transformPanel.Focused() && !outlinePanel.Focused(); }
#endif
    bool Initialize(SDL_Window* window, MeshAssets& models) override;
    void SubmitGizmos(WorldRenderer& renderer, const WorldDocument& world, const FreeCamera& camera) const override;
    bool DraggingGizmo() const override { return drag == Drag::Gizmo; }
    const Road* Selected(const WorldDocument& world) const override;
    bool WireframeEnabled() const override { return menu.WireframeEnabled(); }
    bool IsPlanView() const override { return planView; }
    bool SetContentViewport(SDL_Renderer* renderer) override;
    EditorEvent HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event, WorldDocument& world, FreeCamera& camera) override;
    bool Render(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera) override;

private:
    enum class Drag
    {
        None,
        Move,
        Rotate,
        Pan,
        Measure,
        Gizmo
    };
    using Action = RoadAction;
#ifndef _WIN32
    struct Button
    {
        Action action;
        SDL_FRect rect;
        const char* label;
        bool active = false;
        bool enabled = true;
    };

    std::vector<Button> Buttons(const WorldDocument& world) const;
#endif
    RoadStyle& Style(WorldDocument& world);
    bool EditingDraft() const { return planView && state.drawing; }
    Vector3 ToWorld(float x, float y) const;
    SDL_FPoint ToScreen(Vector3 point) const;
    SDL_FPoint Handle(const Road& road, int index) const;
    bool RenderPlan(SDL_Renderer* renderer, const WorldDocument& world) const;
    bool DrawMeasurement(SDL_Renderer* renderer) const;
#ifndef _WIN32
    bool RenderRoadPanel(SDL_Renderer* renderer, const WorldDocument& world) const;
#endif
    void ChangeView(WorldDocument& world, FreeCamera& camera);
    void OpenViewportMenu(WorldDocument& world, FreeCamera& camera);
    void BeginDraft(WorldDocument& world);
    std::optional<GizmoPlacement> SelectedGizmoPlacement(const WorldDocument& world, const FreeCamera& camera) const;
    void PickWorldRoad(WorldDocument& world, const FreeCamera& camera, SDL_Keymod modifiers);
    void BeginRoadDrag(const Road& road, Drag kind, bool shift);
    void MovePoint(Road& road, Vector3 position);
    void MoveGizmo(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer);
    void ApplyOutline(const OutlineCommand& cmd, WorldDocument& world);
    void ApplyInspectorEdit(WorldDocument& world, const TransformEdit& edit);
    void SelectAt(const WorldDocument& world, Vector3 pos);
    void Activate(Action action, WorldDocument& world, FreeCamera& camera);
    void ChangeStyle(Action action, WorldDocument& world);
    void PressCanvas(WorldDocument& world, float x, float y, SDL_Keymod modifiers);
    void DeleteSelection(WorldDocument& world);
    void Motion(WorldDocument& world, const SDL_MouseMotionEvent& motion);
    void Release(WorldDocument& world);
    bool Save(WorldDocument& world);
    void RunCommand(EditorCommand command, WorldDocument& world, FreeCamera& camera);
    void CommitTransform(WorldDocument& world, EditSnapshot before);
    void Commit(WorldDocument& world, int roadIndex, EditSnapshot before, const Road* after = nullptr);

    double scale = 4.0; // pixels per meter
    Vector3 center{};
    SDL_Rect scene{};
#ifndef _WIN32
    SDL_Rect browserPanel{}, roadPanel{}, transformArea{}, outlineArea{};
    SdlOutliner outlinePanel;
    SdlTransformPanel transformPanel;
    int roadScroll = 0, roadScrollMax = 0;
#endif
    struct ViewportClick
    {
        SDL_FPoint windowPoint;
        ViewportPicker picker;
        SDL_FPoint motion{};
    };
    std::optional<ViewportClick> rightClick;
    Vector3 creationPoint{};
    RoadEditorState state;
    EditHistory history;
    SceneOutliner outliner;
    EditorMenu menu;
    RoadOverlay overlay;
    TransformGizmo gizmo;
    std::optional<GizmoDrag> gizmoDrag;
    std::vector<RoadPoint> originalPoints;
    Vector3 originalRotation{};
    std::optional<int> extensionSource; // Original endpoint index for the entire Shift-drag.
#ifndef _WIN32
    CameraToolbar cameraToolbar;
#endif
    ContentBrowser browser;
#ifdef _WIN32
    WindowsPanels panels;
#endif
    bool planView = false;
    bool dragChanged = false;
    bool saveFailed = false;
    std::optional<EditSnapshot> dragStart;
    bool initialized = false;
    Drag drag = Drag::None;
    Vector3 measureStart{}, measureEnd{}; // World-space endpoints while Drag::Measure is active.
    Vector3 grabOffset{};
    SDL_FPoint mouse{};
    std::string status;
};
