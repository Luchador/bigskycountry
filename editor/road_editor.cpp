#include "road_editor.hpp"
#include "road_render.hpp"
#include "free_camera.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{
    constexpr float panel = 218.0f;
    constexpr float pickRadius = 9.0f;
    constexpr float handleLength = 35.0f;
    constexpr float threeLanePanelHeight = 104.0f;

    bool Near(SDL_FPoint a, SDL_FPoint b)
    {
        return std::hypot(a.x - b.x, a.y - b.y) <= pickRadius;
    }

    bool HitTriangle(Vector3 p, const std::array<Vector3, 3>& triangle)
    {
        const auto side = [](Vector3 a, Vector3 b, Vector3 p)
        { return (b.x - a.x) * (p.z - a.z) - (b.z - a.z) * (p.x - a.x); };
        const double area = side(triangle[0], triangle[1], triangle[2]);
        if (std::abs(area) < 1.0e-10)
        {
            return false;
        }
        for (int i = 0; i < 3; ++i)
        {
            if (area * side(triangle[i], triangle[(i + 1) % 3], p) < 0.0)
            {
                return false;
            }
        }
        return true;
    }

    bool Fill(SDL_Renderer* renderer, SDL_FRect rect, SDL_Color color)
    {
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255) &&
               SDL_RenderFillRect(renderer, &rect);
    }

    bool Text(SDL_Renderer* renderer, float x, float y, const char* text, SDL_Color color)
    {
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255) &&
               SDL_RenderDebugText(renderer, x, y, text);
    }
}

const Road* RoadEditor::Selected(const RoadMap& map) const
{
    return selected >= 0 ? &map.roads[selected] : nullptr;
}

RoadStyle& RoadEditor::Style(RoadMap& map)
{
    if (drawing)
    {
        return draft.style;
    }
    return selected >= 0 ? map.roads[selected].style : defaults;
}

Vector3 RoadEditor::ToWorld(float x, float y) const
{
    return {center.x + (x - (width + panel) * 0.5) / scale, 0.0, center.z - (y - height * 0.5) / scale};
}

SDL_FPoint RoadEditor::ToScreen(Vector3 p) const
{
    return {static_cast<float>((p.x - center.x) * scale + (width + panel) * 0.5),
            static_cast<float>((center.z - p.z) * scale + height * 0.5)};
}

SDL_FPoint RoadEditor::Handle(const Road& road, int index) const
{
    return ToScreen(road.points[index].position + road.Tangent(index) * (handleLength / scale));
}

std::vector<RoadEditor::Button> RoadEditor::Buttons(const RoadMap& map) const
{
    const Road* current = drawing ? &draft : Selected(map);
    const RoadStyle& style = current ? current->style : defaults;
    const float offset = style.lanes == 3 ? threeLanePanelHeight : 0.0f;
    std::vector<Button> buttons{
        {Action::Back, {12, 44, 194, 28}, "Back to 3D   [R]"},
        {Action::New, {12, 84, 194, 28}, "New road", false, !drawing},
        {Action::Straight, {12, 148, 94, 28}, "Straight", !style.curved},
        {Action::Curve, {112, 148, 94, 28}, "Bezier", style.curved},
        {Action::Fewer, {12, 210, 32, 28}, "-", false, style.lanes > 1},
        {Action::More, {174, 210, 32, 28}, "+", false, style.lanes < std::numeric_limits<int>::max()},
        {Action::Left, {12, 274 + offset, 94, 28}, "Left", style.leftShoulder},
        {Action::Right, {112, 274 + offset, 94, 28}, "Right", style.rightShoulder},
        {Action::Finish, {12, 350 + offset, 194, 28}, "Finish road  [Enter]", false, drawing && draft.points.size() >= 2},
        {Action::Cancel, {12, 384 + offset, 194, 28}, "Cancel draft [Esc]", false, drawing},
        {Action::Remove, {12, 424 + offset, 194, 28}, "Delete selected road", false, current && !drawing},
        {Action::Save, {12, 458 + offset, 194, 28}, "Save   [Ctrl+S]"}};

    if (style.lanes == 3)
    {
        buttons.push_back({Action::TwoNorthbound, {12, 268, 194, 26}, "2 north / 1 south",
                           style.threeLaneLayout == ThreeLaneLayout::TwoNorthbound});
        buttons.push_back({Action::TwoSouthbound, {12, 298, 194, 26}, "1 north / 2 south",
                           style.threeLaneLayout == ThreeLaneLayout::TwoSouthbound});
        buttons.push_back({Action::CenterTurn, {12, 328, 194, 26}, "Center turn lane",
                           style.threeLaneLayout == ThreeLaneLayout::CenterTurn});
    }
    return buttons;
}

void RoadEditor::Save(RoadMap& map)
{
    edited = !map.Save();
    if (edited)
    {
        status = "Save failed - use Save to retry";
        SDL_Log("%s", SDL_GetError());
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save roads", SDL_GetError(), nullptr);
    }
    else
    {
        status = "Saved";
    }
}

void RoadEditor::Activate(Action action, RoadMap& map)
{
    if (action == Action::Save && drawing)
    {
        if (draft.points.size() < 2)
        {
            status = "Place at least two points to save the draft";
            return;
        }
        action = Action::Finish;
    }
    RoadStyle& style = Style(map);
    switch (action)
    {
    case Action::Back:
        back = true;
        return;
    case Action::New:
        draft = {};
        draft.style = style;
        drawing = true;
        selected = point = -1;
        status.clear();
        return;
    case Action::Cancel:
        draft = {};
        drawing = false;
        status.clear();
        return;
    case Action::Finish:
        if (!drawing || draft.points.size() < 2)
        {
            return;
        }
        draft.Rebuild();
        defaults = draft.style;
        map.roads.push_back(std::move(draft));
        draft = {};
        drawing = false;
        selected = static_cast<int>(map.roads.size()) - 1;
        point = static_cast<int>(map.roads.back().points.size()) - 1;
        Save(map);
        return;
    case Action::Remove:
        if (selected >= 0 && !drawing)
        {
            map.roads.erase(map.roads.begin() + selected);
            selected = point = -1;
            Save(map);
        }
        return;
    case Action::Save:
        Save(map);
        return;
    case Action::Straight:
        style.curved = false;
        break;
    case Action::Curve:
        style.curved = true;
        break;
    case Action::Fewer:
        --style.lanes;
        break;
    case Action::More:
        ++style.lanes;
        break;
    case Action::TwoNorthbound:
        style.threeLaneLayout = ThreeLaneLayout::TwoNorthbound;
        break;
    case Action::TwoSouthbound:
        style.threeLaneLayout = ThreeLaneLayout::TwoSouthbound;
        break;
    case Action::CenterTurn:
        style.threeLaneLayout = ThreeLaneLayout::CenterTurn;
        break;
    case Action::Left:
        style.leftShoulder = !style.leftShoulder;
        break;
    case Action::Right:
        style.rightShoulder = !style.rightShoulder;
        break;
    }
    defaults = style;
    if (!drawing && selected >= 0)
    {
        map.roads[selected].Rebuild();
        Save(map);
    }
}

void RoadEditor::PressCanvas(RoadMap& map, float x, float y)
{
    const Vector3 pos = ToWorld(x, y);
    const SDL_FPoint cursor{x, y};
    if (drawing)
    {
        if (draft.points.empty() || Math::Length(pos - draft.points.back().position) > 0.05)
        {
            draft.points.push_back({pos, {}});
        }
        return;
    }

    if (selected >= 0 && point >= 0 && Near(cursor, Handle(map.roads[selected], point)))
    {
        drag = Drag::Rotate;
        return;
    }

    // Selected controls get priority where roads overlap.
    auto pickPoint = [&](int roadIndex)
    {
        const auto& points = map.roads[roadIndex].points;
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (Near(cursor, ToScreen(points[i].position)))
            {
                selected = roadIndex;
                point = static_cast<int>(i);
                grabOffset = points[i].position - pos;
                drag = Drag::Move;
                return true;
            }
        }
        return false;
    };
    if (selected >= 0 && pickPoint(selected))
    {
        return;
    }
    for (int i = static_cast<int>(map.roads.size()) - 1; i >= 0; --i)
    {
        if (pickPoint(i))
        {
            return;
        }
    }
    selected = point = -1;
    for (int i = static_cast<int>(map.roads.size()) - 1; i >= 0; --i)
    {
        for (const auto& triangle : map.roads[i].triangles)
        {
            if (HitTriangle(pos, triangle))
            {
                selected = i;
                return;
            }
        }
    }
}

void RoadEditor::Motion(RoadMap& map, const SDL_MouseMotionEvent& motion)
{
    mouse = {motion.x, motion.y};
    if (drag == Drag::Pan)
    {
        center.x -= motion.xrel / scale;
        center.z += motion.yrel / scale;
    }
    else if (drag == Drag::Move || drag == Drag::Rotate)
    {
        Road& road = map.roads[selected];
        RoadPoint& anchor = road.points[point];
        Vector3 pos = ToWorld(mouse.x, mouse.y);
        if (drag == Drag::Move)
        {
            pos = pos + grabOffset;
            if ((point > 0 && Math::Length(pos - road.points[point - 1].position) < 0.05) ||
                (point + 1 < static_cast<int>(road.points.size()) &&
                 Math::Length(pos - road.points[point + 1].position) < 0.05))
            {
                return;
            }
            anchor.position = pos;
        }
        else
        {
            const Vector3 offset = pos - anchor.position;
            if (Math::Length(offset) * scale < 2.0)
            {
                return;
            }
            road.RotatePoint(point, std::atan2(offset.z, offset.x));
        }
        road.Rebuild();
        edited = true;
    }
}

void RoadEditor::Release(RoadMap& map)
{
    if ((drag == Drag::Move || drag == Drag::Rotate) && edited)
    {
        Save(map);
    }
    drag = Drag::None;
}

bool RoadEditor::Render(SDL_Renderer* renderer, const RoadMap& map) const
{
    constexpr SDL_Color ink{221, 228, 234, 255}, muted{159, 174, 183, 255};
    constexpr SDL_Color amber{255, 191, 84, 255}, cyan{99, 207, 205, 255};
    if (!SDL_SetRenderDrawColor(renderer, 36, 45, 49, 255) || !SDL_RenderClear(renderer))
    {
        return false;
    }
    const SDL_Rect canvas{static_cast<int>(panel), 0, std::max(0, width - static_cast<int>(panel)), height};
    if (!SDL_SetRenderClipRect(renderer, &canvas))
    {
        return false;
    }
    double spacing = std::pow(10.0, std::floor(std::log10(100.0 / scale)));
    if (spacing * scale < 35.0)
    {
        spacing *= 5.0;
    }
    const float step = static_cast<float>(spacing * scale);
    const SDL_FPoint origin = ToScreen({});
    if (!SDL_SetRenderDrawColor(renderer, 54, 65, 69, 255))
    {
        return false;
    }
    for (float x = panel + std::fmod(origin.x - panel, step); x < width; x += step)
    {
        if (!SDL_RenderLine(renderer, x, 0, x, static_cast<float>(height)))
        {
            return false;
        }
    }
    for (float y = std::fmod(origin.y, step); y < height; y += step)
    {
        if (!SDL_RenderLine(renderer, panel, y, static_cast<float>(width), y))
        {
            return false;
        }
    }

    std::optional<Road> preview;
    if (drawing && !draft.points.empty())
    {
        preview = draft;
        if (mouse.x >= panel)
        {
            preview->points.push_back({ToWorld(mouse.x, mouse.y), {}});
        }
        preview->Rebuild();
    }
    for (RoadLayer layer : {RoadLayer::Pavement, RoadLayer::Stripes})
    {
        for (const Road& road : map.roads)
        {
            if (!DrawPlanRoad(renderer, road, center, scale, canvas, layer))
            {
                return false;
            }
        }
        if (preview && !DrawPlanRoad(renderer, *preview, center, scale, canvas, layer))
        {
            return false;
        }
    }

    auto drawPoints = [&](const Road& road, bool active)
    {
        for (size_t i = 0; i < road.points.size(); ++i)
        {
            const SDL_FPoint pos = ToScreen(road.points[i].position);
            if (pos.x < panel - 8 || pos.x > width + 8 || pos.y < -8 || pos.y > height + 8)
            {
                continue;
            }
            const bool selectedPoint = active && static_cast<int>(i) == point;
            const float r = active ? 5.0f : 3.0f;
            if (!Fill(renderer, {pos.x - r - 1, pos.y - r - 1, 2 * r + 2, 2 * r + 2}, {30, 36, 40, 255}) ||
                !Fill(renderer, {pos.x - r, pos.y - r, 2 * r, 2 * r},
                      selectedPoint ? amber : (active ? cyan : muted)))
            {
                return false;
            }
        }
        return true;
    };
    for (int i = 0; i < static_cast<int>(map.roads.size()); ++i)
    {
        if (!drawPoints(map.roads[i], i == selected))
        {
            return false;
        }
    }
    if (drawing && !drawPoints(draft, true))
    {
        return false;
    }
    if (!drawing && selected >= 0 && point >= 0)
    {
        const Road& road = map.roads[selected];
        const SDL_FPoint a = ToScreen(road.points[point].position);
        const SDL_FPoint b = Handle(road, point);
        if (a.x > panel - 40 && a.x < width + 40 && a.y > -40 && a.y < height + 40)
        {
            if (!SDL_SetRenderDrawColor(renderer, amber.r, amber.g, amber.b, 255) ||
                !SDL_RenderLine(renderer, a.x, a.y, b.x, b.y) || !Fill(renderer, {b.x - 4, b.y - 4, 8, 8}, amber))
            {
                return false;
            }
        }
    }
    if (!SDL_SetRenderClipRect(renderer, nullptr) ||
        !Fill(renderer, {0, 0, panel, static_cast<float>(height)}, {24, 32, 38, 255}) ||
        !Text(renderer, 14, 18, "Road editor", ink))
    {
        return false;
    }

    for (const Button& b : Buttons(map))
    {
        const SDL_Color fill = b.active ? SDL_Color{45, 101, 102, 255} : SDL_Color{47, 59, 66, 255};
        if (!Fill(renderer, b.rect, b.enabled ? fill : SDL_Color{34, 44, 50, 255}) ||
            !Text(renderer, b.rect.x + 8, b.rect.y + 10, b.label, b.enabled ? ink : muted))
        {
            return false;
        }
    }
    const Road* current = drawing ? &draft : Selected(map);
    const RoadStyle& style = current ? current->style : defaults;
    const float offset = style.lanes == 3 ? threeLanePanelHeight : 0.0f;
    if (style.lanes == 3 && !Text(renderer, 12, 252, "Three-lane layout", muted))
    {
        return false;
    }
    if (!Text(renderer, 12, 132, "Alignment", muted) || !Text(renderer, 12, 194, "Lanes (total)", muted) ||
        !Text(renderer, 12, 258 + offset, "Shoulders (5 m each)", muted) ||
        !SDL_RenderDebugTextFormat(renderer, 89, 220, "%d", style.lanes) ||
        !SDL_RenderDebugTextFormat(renderer, 12, 320 + offset, "Width: %.0f m", style.LeftWidth() + style.RightWidth()))
    {
        return false;
    }

    const char* hint = drawing ? "Click to add points | Enter to finish | Esc to cancel"
                               : "Drag a point to move | Drag its gold handle to rotate";
    if (!Text(renderer, panel + 12, 12, hint, ink) ||
        !Text(renderer, panel + 12, 28, "Wheel: zoom | Right drag: pan", muted) ||
        !SDL_RenderDebugTextFormat(renderer, panel + 12, static_cast<float>(height) - 20,
                                   "Grid: %.0f m    Roads: %zu    %s", spacing, map.roads.size(), status.c_str()))
    {
        return false;
    }
    return SDL_RenderPresent(renderer);
}

ToolResult RoadEditor::Run(SDL_Window* window, SDL_Renderer* renderer, RoadMap& map, FreeCamera& camera)
{
    back = false;
    drag = Drag::None;
    if (!initialized)
    {
        if (!map.roads.empty())
        {
            center = map.roads.front().points.front().position;
        }
        drawing = map.roads.empty();
        draft.style = defaults;
        initialized = true;
    }
    int vsync = 0;
    SDL_GetRenderVSync(renderer, &vsync);
    while (!back)
    {
        if (!SDL_GetCurrentRenderOutputSize(renderer, &width, &height))
        {
            return ToolResult::Error;
        }
        SDL_Event e;
        while (!back && SDL_PollEvent(&e))
        {
            if (e.type == SDL_EVENT_QUIT)
            {
                Release(map);
                return ToolResult::Quit;
            }
            if (SDL_GetWindowFromEvent(&e) != window)
            {
                continue;
            }
            if (!SDL_ConvertEventToRenderCoordinates(renderer, &e))
            {
                return ToolResult::Error;
            }
            switch (e.type)
            {
            case SDL_EVENT_KEY_DOWN:
                if (e.key.repeat)
                {
                    break;
                }
                if (e.key.key == SDLK_ESCAPE && drawing)
                {
                    Activate(Action::Cancel, map);
                }
                else if (e.key.key == SDLK_ESCAPE || e.key.key == SDLK_R)
                {
                    back = true;
                }
                else if (e.key.key == SDLK_RETURN)
                {
                    Activate(Action::Finish, map);
                }
                else if (e.key.key == SDLK_S && (e.key.mod & SDL_KMOD_CTRL))
                {
                    Activate(Action::Save, map);
                }
                break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
                if (drag != Drag::None)
                {
                    break;
                }
                mouse = {e.button.x, e.button.y};
                if (e.button.x >= panel)
                {
                    if (e.button.button == SDL_BUTTON_RIGHT)
                    {
                        drag = Drag::Pan;
                    }
                    if (e.button.button == SDL_BUTTON_LEFT)
                    {
                        PressCanvas(map, e.button.x, e.button.y);
                    }
                }
                else if (e.button.button == SDL_BUTTON_LEFT)
                {
                    for (const Button& button : Buttons(map))
                    {
                        if (button.enabled && SDL_PointInRectFloat(&mouse, &button.rect))
                        {
                            Activate(button.action, map);
                            break;
                        }
                    }
                }
                break;
            case SDL_EVENT_MOUSE_MOTION:
                Motion(map, e.motion);
                break;
            case SDL_EVENT_MOUSE_BUTTON_UP:
                if ((drag == Drag::Pan && e.button.button == SDL_BUTTON_RIGHT) ||
                    (drag != Drag::Pan && e.button.button == SDL_BUTTON_LEFT))
                {
                    Release(map);
                }
                break;
            case SDL_EVENT_WINDOW_FOCUS_LOST:
                Release(map);
                break;
            case SDL_EVENT_MOUSE_WHEEL:
                if (drag == Drag::None && e.wheel.mouse_x >= panel)
                {
                    const Vector3 before = ToWorld(e.wheel.mouse_x, e.wheel.mouse_y);
                    const float amount = e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -e.wheel.y : e.wheel.y;
                    scale = std::clamp(scale * std::pow(1.25, amount), 0.02, 50.0);
                    center = center + (before - ToWorld(e.wheel.mouse_x, e.wheel.mouse_y));
                    mouse = {e.wheel.mouse_x, e.wheel.mouse_y};
                }
                break;
            }
        }
        if (!back && !Render(renderer, map))
        {
            return ToolResult::Error;
        }
        if (!vsync)
        {
            SDL_Delay(16);
        }
    }
    Release(map);
    if (!map.roads.empty())
    {
        const double distance = std::max(15.0, std::min(width - panel, static_cast<float>(height)) * 0.5 / scale);
        camera.LookAt(center + Vector3{0.0, distance, -distance}, center);
    }
    return ToolResult::Closed;
}
