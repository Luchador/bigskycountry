#pragma once

#include "application.hpp"
#include "edit_history.hpp"
#include "editor_menu.hpp"
#include <SDL3/SDL.h>

class RoadEditor : public EditorTool
{
public:
    bool Initialize(SDL_Window* window) override { return menu.Attach(window); }
    bool IsPlanView() const override { return planView; }
    EditorEvent HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event,
                            RoadMap& map, FreeCamera& camera) override;
    bool Render(SDL_Renderer* renderer, const RoadMap& map, const FreeCamera& camera) override;

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

    std::vector<Button> Buttons(const RoadMap& map) const;
    RoadStyle& Style(RoadMap& map);
    const Road* Selected(const RoadMap& map) const;
    bool EditingDraft() const { return planView && state.drawing; }
    Vector3 ToWorld(float x, float y) const;
    SDL_FPoint ToScreen(Vector3 point) const;
    SDL_FPoint Handle(const Road& road, int index) const;
    bool RenderPlan(SDL_Renderer* renderer, const RoadMap& map) const;
    bool RenderToolbar(SDL_Renderer* renderer, const RoadMap& map) const;
    void ChangeView(RoadMap& map, FreeCamera& camera);
    void BeginDraft(RoadMap& map);
    void PickWorldRoad(const RoadMap& map, const FreeCamera& camera);
    void SelectAt(const RoadMap& map, Vector3 pos);
    void Activate(Action action, RoadMap& map, FreeCamera& camera);
    void ChangeStyle(Action action, RoadMap& map);
    void PressCanvas(RoadMap& map, float x, float y);
    void Motion(RoadMap& map, const SDL_MouseMotionEvent& motion);
    void Release(RoadMap& map);
    bool Save(RoadMap& map);
    void RunCommand(EditorCommand command, RoadMap& map, FreeCamera& camera);
    void Commit(RoadMap& map, int roadIndex, EditSnapshot before, const Road* after = nullptr);

    double scale = 4.0; // pixels per meter
    Vector3 center{};
    int width = 0;
    int height = 0;
    RoadEditorState state;
    EditHistory history;
    EditorMenu menu;
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
