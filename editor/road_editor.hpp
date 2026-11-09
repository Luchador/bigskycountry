#pragma once

#include "application.hpp"
#include "camera_toolbar.hpp"
#include "edit_history.hpp"
#include "editor_menu.hpp"
#include <SDL3/SDL.h>

class RoadEditor : public EditorTool
{
public:
    bool Initialize(SDL_Window* window) override { return menu.Attach(window); }
    const Road* Selected(const WorldDocument& world) const override;
    bool IsPlanView() const override { return planView; }
    bool SetContentViewport(SDL_Renderer* renderer) override;
    EditorEvent HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event,
                            WorldDocument& world, FreeCamera& camera) override;
    bool Render(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera) override;

private:
    enum class Drag
    {
        None,
        Move,
        Rotate,
        Pan
    };
    enum class Action
    {
        ChangeView,
        New,
        Finish,
        Cancel,
        Straight,
        Curve,
        Fewer,
        More,
        TwoNorthbound,
        TwoSouthbound,
        CenterTurn,
        Left,
        Right,
        NoEdge,
        Transition,
        Curb,
        Remove
    };
    struct Button
    {
        Action action;
        SDL_FRect rect;
        const char* label;
        bool active = false;
        bool enabled = true;
    };

    std::vector<Button> Buttons(const WorldDocument& world) const;
    RoadStyle& Style(WorldDocument& world);
    bool EditingDraft() const { return planView && state.drawing; }
    Vector3 ToWorld(float x, float y) const;
    SDL_FPoint ToScreen(Vector3 point) const;
    SDL_FPoint Handle(const Road& road, int index) const;
    bool RenderPlan(SDL_Renderer* renderer, const WorldDocument& world) const;
    bool RenderRoadPanel(SDL_Renderer* renderer, const WorldDocument& world) const;
    void ChangeView(WorldDocument& world, FreeCamera& camera);
    void BeginDraft(WorldDocument& world);
    void PickWorldRoad(const WorldDocument& world, const FreeCamera& camera);
    void SelectAt(const WorldDocument& world, Vector3 pos);
    void Activate(Action action, WorldDocument& world, FreeCamera& camera);
    void ChangeStyle(Action action, WorldDocument& world);
    void PressCanvas(WorldDocument& world, float x, float y);
    void Motion(WorldDocument& world, const SDL_MouseMotionEvent& motion);
    void Release(WorldDocument& world);
    bool Save(WorldDocument& world);
    void RunCommand(EditorCommand command, WorldDocument& world, FreeCamera& camera);
    void Commit(WorldDocument& world, int roadIndex, EditSnapshot before, const Road* after = nullptr);

    double scale = 4.0; // pixels per meter
    Vector3 center{};
    int width = 0;
    int height = 0;
    RoadEditorState state;
    EditHistory history;
    EditorMenu menu;
    CameraToolbar cameraToolbar;
    bool planView = false;
    bool dragChanged = false;
    bool saveFailed = false;
    std::optional<EditSnapshot> dragStart;
    bool initialized = false;
    Drag drag = Drag::None;
    Vector3 grabOffset{};
    SDL_FPoint mouse{};
    std::string status;
};
