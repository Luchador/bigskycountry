#pragma once

#include "application.hpp"
#include "road.hpp"
#include <SDL3/SDL.h>

class RoadEditor : public EditorTool
{
public:
    ToolResult Run(SDL_Window* window, SDL_Renderer* renderer, RoadMap& map, FreeCamera& camera) override;

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
        Back,
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
        Remove,
        Save
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
    Vector3 ToWorld(float x, float y) const;
    SDL_FPoint ToScreen(Vector3 point) const;
    SDL_FPoint Handle(const Road& road, int index) const;
    bool Render(SDL_Renderer* renderer, const RoadMap& map) const;
    void Activate(Action action, RoadMap& map);
    void PressCanvas(RoadMap& map, float x, float y);
    void Motion(RoadMap& map, const SDL_MouseMotionEvent& motion);
    void Release(RoadMap& map);
    void Save(RoadMap& map);

    double scale = 4.0; // pixels per meter
    Vector3 center{};
    int width = 0;
    int height = 0;
    int selected = -1;
    int point = -1;
    bool drawing = false;
    bool back = false;
    bool edited = false;
    bool initialized = false;
    RoadStyle defaults;
    Road draft;
    Drag drag = Drag::None;
    Vector3 grabOffset{};
    SDL_FPoint mouse{};
    std::string status;
};
