#include "road_editor.hpp"
#include "world_state.hpp"
#include "world_document.hpp"
#include "road_render.hpp"
#include "free_camera.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
    constexpr float panel = 218.0f;
    constexpr float pickRadius = 9.0f;
    constexpr float handleLength = 35.0f;
    constexpr float threeLanePanelHeight = 104.0f;
    constexpr SDL_Color ink{221, 228, 234, 255}, muted{159, 174, 183, 255};

    bool Near(SDL_FPoint a, SDL_FPoint b)
    {
        return std::hypot(a.x - b.x, a.y - b.y) <= pickRadius;
    }

    bool HitTriangle(Vector3 p, const std::array<Vector3, 3>& triangle)
    {
        const auto side = [](Vector3 a, Vector3 b, Vector3 p)
        { 
            return (b.x - a.x) * (p.z - a.z) - (b.z - a.z) * (p.x - a.x); 
        };

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
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255) && SDL_RenderFillRect(renderer, &rect);
    }

    bool Text(SDL_Renderer* renderer, float x, float y, const char* text, SDL_Color color)
    {
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 255) && SDL_RenderDebugText(renderer, x, y, text);
    }
}

const Road* RoadEditor::Selected(const WorldDocument& world) const
{
    return state.selected >= 0 ? &world.roads[state.selected] : nullptr;
}

RoadStyle& RoadEditor::Style(WorldDocument& world)
{
    if (EditingDraft())
    {
        return state.draft->style;
    }

    return state.selected >= 0 ? world.roads[state.selected].style : state.defaults;
}

Vector3 RoadEditor::ToWorld(float x, float y) const
{
    return {center.x + (x - (width + panel) * 0.5) / scale, 0.0, center.z - (y - height * 0.5) / scale};
}

SDL_FPoint RoadEditor::ToScreen(Vector3 p) const
{
    return {static_cast<float>((p.x - center.x) * scale + (width + panel) * 0.5), static_cast<float>((center.z - p.z) * scale + height * 0.5)};
}

SDL_FPoint RoadEditor::Handle(const Road& road, int index) const
{
    return ToScreen(road.points[index].position + road.Tangent(index) * (handleLength / scale));
}

std::vector<RoadEditor::Button> RoadEditor::Buttons(const WorldDocument& world) const
{
    const Road* current = EditingDraft() ? &*state.draft : Selected(world);
    const RoadStyle& style = current ? current->style : state.defaults;
    const int laneCount = style.GetLaneCount();
    const float offset = laneCount == 3 ? threeLanePanelHeight : 0.0f;
    const float removeY = (planView ? 424.0f : 350.0f) + offset;

    std::vector<Button> buttons{
        {Action::ChangeView, {12, 44, 194, 28}, planView ? "Back to 3D   [R]" : "Top-down view   [R]"},
        {Action::New, {12, 84, 194, 28}, state.draft && !EditingDraft() ? "Resume draft" : "New road", false, !EditingDraft()},
        {Action::Straight, {12, 148, 94, 28}, "Straight", !style.curved},
        {Action::Curve, {112, 148, 94, 28}, "Bezier", style.curved},
        {Action::Fewer, {12, 210, 32, 28}, "-", false, laneCount > 1},
        {Action::More, {174, 210, 32, 28}, "+", false, laneCount < RoadStyle::maxLanes},
        {Action::Left, {12, 274 + offset, 94, 28}, "Left", style.leftShoulder},
        {Action::Right, {112, 274 + offset, 94, 28}, "Right", style.rightShoulder},
        {Action::Remove, {12, removeY, 194, 28}, "Delete selected road", false, current && !EditingDraft()}};

    if (planView)
    {
        buttons.push_back({Action::Finish, {12, 350 + offset, 194, 28}, "Finish road  [Enter]", false,
                           state.drawing && state.draft->points.size() >= 2});
        buttons.push_back({Action::Cancel, {12, 384 + offset, 194, 28}, "Cancel draft [Esc]", false, state.drawing});
    }

    if (laneCount == 3)
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

bool RoadEditor::Save(WorldDocument& world)
{
    saveFailed = !world.Save();
    if (saveFailed)
    {
        status = "Save failed - use File > Save to retry";
        SDL_Log("%s", SDL_GetError());
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save world", SDL_GetError(), nullptr);
        return false;
    }
    status = "Saved";
    return true;
}

void RoadEditor::Commit(WorldDocument& world, int roadIndex, EditSnapshot before, const Road* after)
{
    if (history.Record(roadIndex, std::move(before), EditSnapshot(state, after)) && roadIndex >= 0)
    {
        Save(world);
    }
}

void RoadEditor::RunCommand(EditorCommand command, WorldDocument& world, FreeCamera& camera)
{
    Release(world);
    if (command == EditorCommand::Save)
    {
        if (!saveFailed && EditingDraft())
        {
            if (state.draft->points.size() < 2)
            {
                status = "Place at least two points to save the draft";
                return;
            }
            Activate(Action::Finish, world, camera);
        }
        else
        {
            Save(world);
        }
        return;
    }

    const bool undo = command == EditorCommand::Undo;
    const auto result = undo ? history.Undo(world.roads, state) : history.Redo(world.roads, state);
    if (result != EditHistory::Result::Unavailable &&
        (result != EditHistory::Result::MapChanged || Save(world)))
    {
        status = undo ? "Undone" : "Redone";
    }
}

void RoadEditor::BeginDraft(WorldDocument& world)
{
    // Resuming an existing draft is navigation, creating one is an edit.
    std::optional<EditSnapshot> before;
    if (!state.draft)
    {
        before.emplace(state);
        const RoadStyle style = Style(world);
        state.draft.emplace().style = style;
    }

    state.drawing = true;
    state.selected = state.point = -1;
    status.clear();
    if (before)
    {
        Commit(world, -1, std::move(*before));
    }
}

void RoadEditor::Activate(Action action, WorldDocument& world, FreeCamera& camera)
{
    switch (action)
    {
    case Action::ChangeView:
        ChangeView(world, camera);
        return;
    case Action::New:
        if (!planView)
        {
            ChangeView(world, camera);
        }
        if (!state.drawing)
        {
            BeginDraft(world);
        }
        return;
    case Action::Cancel:
    {
        EditSnapshot before(state);
        state.draft.reset();
        state.drawing = false;
        status.clear();
        Commit(world, -1, std::move(before));
        return;
    }
    case Action::Finish:
    {
        if (!EditingDraft() || state.draft->points.size() < 2)
        {
            return;
        }
        EditSnapshot before(state);
        state.draft->Rebuild();
        state.defaults = state.draft->style;
        world.roads.push_back(std::move(*state.draft));
        state.draft.reset();
        state.drawing = false;
        state.selected = static_cast<int>(world.roads.size()) - 1;
        state.point = static_cast<int>(world.roads.back().points.size()) - 1;
        Commit(world, state.selected, std::move(before), &world.roads.back());
        return;
    }
    case Action::Remove:
        if (state.selected >= 0 && !EditingDraft())
        {
            const int index = state.selected;
            EditSnapshot before(state, &world.roads[index]);
            world.roads.erase(world.roads.begin() + index);
            state.selected = state.point = -1;
            Commit(world, index, std::move(before));
        }
        return;
    default:
        ChangeStyle(action, world);
        return;
    }
}

void RoadEditor::ChangeStyle(Action action, WorldDocument& world)
{
    RoadStyle& current = Style(world);
    RoadStyle style = current;
    switch (action)
    {
    case Action::Straight:
        style.curved = false;
        break;
    case Action::Curve:
        style.curved = true;
        break;
    case Action::Fewer:
        style.SetLaneCount(style.GetLaneCount() - 1);
        break;
    case Action::More:
        style.SetLaneCount(style.GetLaneCount() + 1);
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
    default:
        return;
    }
    if (style == current)
    {
        return;
    }
    const int index = EditingDraft() ? -1 : state.selected;
    Road* road = index >= 0 ? &world.roads[index] : nullptr;
    EditSnapshot before(state, road);
    current = style;
    state.defaults = style;
    if (road)
    {
        road->Rebuild();
    }
    Commit(world, index, std::move(before), road);
}

void RoadEditor::PressCanvas(WorldDocument& world, float x, float y)
{
    const Vector3 pos = ToWorld(x, y);
    const SDL_FPoint cursor{x, y};
    if (state.drawing)
    {
        if (state.draft->points.empty() || Math::Length(pos - state.draft->points.back().position) > 0.05)
        {
            EditSnapshot before(state);
            state.draft->points.push_back({pos, {}});
            Commit(world, -1, std::move(before));
        }
        return;
    }

    if (state.selected >= 0 && state.point >= 0 && Near(cursor, Handle(world.roads[state.selected], state.point)))
    {
        dragStart.emplace(state, Selected(world));
        drag = Drag::Rotate;
        return;
    }

    // Selected controls get priority where roads overlap.
    auto pickPoint = [&](int roadIndex)
    {
        const auto& points = world.roads[roadIndex].points;
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (Near(cursor, ToScreen(points[i].position)))
            {
                state.selected = roadIndex;
                state.point = static_cast<int>(i);
                grabOffset = points[i].position - pos;
                dragStart.emplace(state, Selected(world));
                drag = Drag::Move;
                return true;
            }
        }
        return false;
    };
    if (state.selected >= 0 && pickPoint(state.selected))
    {
        return;
    }
    for (int i = static_cast<int>(world.roads.size()) - 1; i >= 0; --i)
    {
        if (pickPoint(i))
        {
            return;
        }
    }
    SelectAt(world, pos);
}

void RoadEditor::SelectAt(const WorldDocument& world, Vector3 pos)
{
    state.selected = state.point = -1;
    // Later roads are drawn last, so pick them first wherever pavement overlaps.
    for (int i = static_cast<int>(world.roads.size()) - 1; i >= 0; --i)
    {
        for (const auto& triangle : world.roads[i].triangles)
        {
            if (HitTriangle(pos, triangle))
            {
                state.selected = i;
                return;
            }
        }
    }
}

void RoadEditor::Motion(WorldDocument& world, const SDL_MouseMotionEvent& motion)
{
    mouse = {motion.x, motion.y};
    if (drag == Drag::Pan)
    {
        center.x -= motion.xrel / scale;
        center.z += motion.yrel / scale;
    }
    else if (drag == Drag::Move || drag == Drag::Rotate)
    {
        Road& road = world.roads[state.selected];
        RoadPoint& anchor = road.points[state.point];
        Vector3 pos = ToWorld(mouse.x, mouse.y);
        if (drag == Drag::Move)
        {
            pos = pos + grabOffset;
            if ((state.point > 0 && Math::Length(pos - road.points[state.point - 1].position) < 0.05) ||
                (state.point + 1 < static_cast<int>(road.points.size()) &&
                 Math::Length(pos - road.points[state.point + 1].position) < 0.05))
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
            road.RotatePoint(state.point, std::atan2(offset.z, offset.x));
        }
        road.Rebuild();
        dragChanged = true;
    }
}

void RoadEditor::Release(WorldDocument& world)
{
    if (dragStart && dragChanged)
    {
        Commit(world, state.selected, std::move(*dragStart), Selected(world));
    }
    dragStart.reset();
    dragChanged = false;
    drag = Drag::None;
}

bool RoadEditor::RenderPlan(SDL_Renderer* renderer, const WorldDocument& world) const
{
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
    if (state.drawing && !state.draft->points.empty())
    {
        preview = state.draft;
        if (mouse.x >= panel)
        {
            preview->points.push_back({ToWorld(mouse.x, mouse.y), {}});
        }
        preview->Rebuild();
    }
    for (RoadLayer layer : {RoadLayer::Pavement, RoadLayer::Stripes})
    {
        for (const Road& road : world.roads)
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

    if (const Road* road = Selected(world))
    {
        if (!DrawPlanRoad(renderer, *road, center, scale, canvas, RoadLayer::Outline))
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
            const bool selectedPoint = active && static_cast<int>(i) == state.point;
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
    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
    {
        if (!drawPoints(world.roads[i], i == state.selected))
        {
            return false;
        }
    }
    if (state.drawing && !drawPoints(*state.draft, true))
    {
        return false;
    }
    if (!state.drawing && state.selected >= 0 && state.point >= 0)
    {
        const Road& road = world.roads[state.selected];
        const SDL_FPoint a = ToScreen(road.points[state.point].position);
        const SDL_FPoint b = Handle(road, state.point);
        if (a.x > panel - 40 && a.x < width + 40 && a.y > -40 && a.y < height + 40)
        {
            if (!SDL_SetRenderDrawColor(renderer, amber.r, amber.g, amber.b, 255) ||
                !SDL_RenderLine(renderer, a.x, a.y, b.x, b.y) || !Fill(renderer, {b.x - 4, b.y - 4, 8, 8}, amber))
            {
                return false;
            }
        }
    }
    const char* hint = state.drawing ? "Click to add points | Enter to finish | Esc to cancel"
                               : "Drag a point to move | Drag its gold handle to rotate";
    if (!Text(renderer, panel + 12, 12, hint, ink) ||
        !Text(renderer, panel + 12, 28, "Wheel: zoom | Right drag: pan", muted) ||
        !SDL_RenderDebugTextFormat(renderer, panel + 12, static_cast<float>(height) - 20,
                                   "Grid: %.0f m    Roads: %zu    %s", spacing, world.roads.size(), status.c_str()))
    {
        return false;
    }
    return SDL_SetRenderClipRect(renderer, nullptr);
}

bool RoadEditor::RenderToolbar(SDL_Renderer* renderer, const WorldDocument& world) const
{
    if (!Fill(renderer, {0, 0, panel, static_cast<float>(height)}, {24, 32, 38, 255}))
    {
        return false;
    }

    for (const Button& b : Buttons(world))
    {
        const SDL_Color fill = b.active ? SDL_Color{45, 101, 102, 255} : SDL_Color{47, 59, 66, 255};
        if (!Fill(renderer, b.rect, b.enabled ? fill : SDL_Color{34, 44, 50, 255}) ||
            !Text(renderer, b.rect.x + 8, b.rect.y + 10, b.label, b.enabled ? ink : muted))
        {
            return false;
        }
    }
    const Road* current = EditingDraft() ? &*state.draft : Selected(world);
    const RoadStyle& style = current ? current->style : state.defaults;
    const int laneCount = style.GetLaneCount();
    const float offset = laneCount == 3 ? threeLanePanelHeight : 0.0f;
    if (laneCount == 3 && !Text(renderer, 12, 252, "Three-lane layout", muted))
    {
        return false;
    }
    if (laneCount > 3 && style.HasCenterTurnLane())
    {
        if (!SDL_RenderDebugTextFormat(renderer, 12, 243, "%d each way + center turn", laneCount / 2))
        {
            return false;
        }
    }
    if (!Text(renderer, 12, 132, "Alignment", muted) || !Text(renderer, 12, 194, "Lanes (total)", muted) ||
        !Text(renderer, 12, 258 + offset, "Shoulders (5 m each)", muted) ||
        !SDL_RenderDebugTextFormat(renderer, 89, 220, "%d", laneCount) ||
        !SDL_RenderDebugTextFormat(renderer, 12, 320 + offset, "Width: %.0f m", style.LeftWidth() + style.RightWidth()))
    {
        return false;
    }

    if (EditingDraft())
    {
        return Text(renderer, 14, 18, "Road editor | Draft", ink);
    }
    if (state.selected >= 0)
    {
        return SDL_SetRenderDrawColor(renderer, ink.r, ink.g, ink.b, 255) &&
               SDL_RenderDebugTextFormat(renderer, 14, 18, "Road editor | Road %d", state.selected + 1);
    }
    return Text(renderer, 14, 18, "Road editor", ink);
}

bool RoadEditor::Render(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera)
{
    menu.SetHistoryEnabled(history.CanUndo() || dragChanged, history.CanRedo() && !dragChanged);
    if (!SDL_GetCurrentRenderOutputSize(renderer, &width, &height))
    {
        return false;
    }
    if (planView)
    {
        if (!RenderPlan(renderer, world))
        {
            return false;
        }
    }
    else
    {
        const Road* road = Selected(world);
        const int minutes = static_cast<int>(g_WorldState.GetTimeOfDayMinutes());
        if ((road && !DrawWorldRoadOutline(renderer, camera, *road, {0, 0, width, height})) ||
            !Text(renderer, panel + 12, 12, "Click a road to select | Right drag to look", ink) ||
            !SDL_RenderDebugTextFormat(renderer, panel + 12, 28, "Sky %02d:%02d CDT   [ earlier   ] later",
                                       minutes / 60, minutes % 60) ||
            !Text(renderer, panel + 12, static_cast<float>(height) - 20, status.c_str(), ink))
        {
            return false;
        }
    }
    return RenderToolbar(renderer, world);
}

void RoadEditor::ChangeView(WorldDocument& world, FreeCamera& camera)
{
    Release(world);
    planView = !planView;
    if (planView && !initialized)
    {
        if (world.roads.empty())
        {
            BeginDraft(world);
        }
        else if (!Selected(world))
        {
            center = world.roads.front().points.front().position;
        }
        initialized = true;
    }
    if (!planView && !world.roads.empty())
    {
        const double distance = std::max(15.0, std::min(width - panel, static_cast<float>(height)) * 0.5 / scale);
        camera.LookAt(center + Vector3{0.0, distance, -distance}, center);
    }
}

void RoadEditor::PickWorldRoad(const WorldDocument& world, const FreeCamera& camera)
{
    state.selected = state.point = -1;
    const double focal = std::min(width, height) * 0.5;
    if (focal <= 0.0)
    {
        return;
    }
    const double x = (mouse.x - width * 0.5) / focal;
    const double y = (height * 0.5 - mouse.y) / focal;
    if (const auto hit = camera.PickGround(x, y, roadNearDepth))
    {
        SelectAt(world, *hit);
        if (state.selected >= 0)
        {
            state.drawing = false;
            center = *hit;
        }
    }
}

EditorEvent RoadEditor::HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event,
                                   WorldDocument& world, FreeCamera& camera)
{
    if (event.type == SDL_EVENT_QUIT)
    {
        Release(world);
        return EditorEvent::Pass;
    }
    if (const auto command = menu.Command(event))
    {
        RunCommand(*command, world, camera);
        return EditorEvent::Handled;
    }
    if (SDL_GetWindowFromEvent(&event) != window)
    {
        return EditorEvent::Pass;
    }
    if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST)
    {
        Release(world);
        return EditorEvent::Pass;
    }
    if (!planView && event.type == SDL_EVENT_KEY_DOWN &&
        !(event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_GUI | SDL_KMOD_SHIFT)) &&
        (event.key.key == SDLK_LEFTBRACKET || event.key.key == SDLK_RIGHTBRACKET))
    {
        const int minutes = static_cast<int>(g_WorldState.GetTimeOfDayMinutes());
        g_WorldState.SetTimeOfDayMinutes(minutes + (event.key.key == SDLK_RIGHTBRACKET ? 30 : -30));
        return EditorEvent::Handled;
    }
    if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat)
    {
        if (event.key.key == SDLK_ESCAPE && EditingDraft())
        {
            Activate(Action::Cancel, world, camera);
            return EditorEvent::Handled;
        }
        if (event.key.key == SDLK_R || (event.key.key == SDLK_ESCAPE && planView))
        {
            ChangeView(world, camera);
            return EditorEvent::Handled;
        }
        if (event.key.key == SDLK_RETURN && planView)
        {
            Activate(Action::Finish, world, camera);
            return EditorEvent::Handled;
        }
    }

    // Input owns relative mouse motion and the right-button release during 3D look.
    if (!planView && (event.type != SDL_EVENT_MOUSE_BUTTON_DOWN || SDL_GetWindowRelativeMouseMode(window)))
    {
        return EditorEvent::Pass;
    }

    SDL_Event e = event;
    if (!SDL_ConvertEventToRenderCoordinates(renderer, &e) ||
        !SDL_GetCurrentRenderOutputSize(renderer, &width, &height))
    {
        return EditorEvent::Error;
    }
    switch (e.type)
    {
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        if (drag != Drag::None)
        {
            break;
        }
        mouse = {e.button.x, e.button.y};
        if (mouse.x < panel)
        {
            if (e.button.button == SDL_BUTTON_LEFT)
            {
                for (const Button& button : Buttons(world))
                {
                    if (button.enabled && SDL_PointInRectFloat(&mouse, &button.rect))
                    {
                        Activate(button.action, world, camera);
                        break;
                    }
                }
            }
        }
        else if (e.button.button == SDL_BUTTON_LEFT)
        {
            if (planView)
            {
                PressCanvas(world, mouse.x, mouse.y);
            }
            else
            {
                PickWorldRoad(world, camera);
            }
        }
        else if (!planView)
        {
            return EditorEvent::Pass;
        }
        else if (e.button.button == SDL_BUTTON_RIGHT)
        {
            drag = Drag::Pan;
        }
        break;
    case SDL_EVENT_MOUSE_MOTION:
        Motion(world, e.motion);
        break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
        if ((drag == Drag::Pan && e.button.button == SDL_BUTTON_RIGHT) ||
            (drag != Drag::Pan && e.button.button == SDL_BUTTON_LEFT))
        {
            Release(world);
        }
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
    return EditorEvent::Handled;
}
