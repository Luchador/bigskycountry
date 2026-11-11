#include "road_editor.hpp"
#include "editor_theme.hpp"
#include "editor_colors.hpp"
#include "viewport_picker.hpp"
#include "world_state.hpp"
#include "world_document.hpp"
#include "road_render.hpp"
#include "world_renderer.hpp"
#include "free_camera.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
    constexpr int panelWidth = 218;
    constexpr float pickRadius = 9.0f;
    constexpr float handleLength = 35.0f;
#ifndef _WIN32
    constexpr float threeLanePanelHeight = 104.0f;

    float RoadStyleBottom(const RoadStyle& style)
    {
        return (style.IsDivided() ? 514.0f : 406.0f) +
               (style.GetLaneCount() == 3 ? threeLanePanelHeight : 0.0f);
    }
#endif
    constexpr auto ink = EditorTheme::text, muted = EditorTheme::secondary;

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

bool RoadEditor::Initialize(SDL_Window* window, MeshAssets& models)
{
    if (!menu.Attach(window))
    {
        return false;
    }
    browser.Refresh();
    gizmo.Load(models);
#ifndef _WIN32
    cameraToolbar.LoadIcons(SDL_GetRenderer(window));
#endif
    return true;
}

std::optional<GizmoPlacement> RoadEditor::SelectedGizmoPlacement(const WorldDocument& world, const FreeCamera& camera) const
{
    if (!state.segments.empty())
    {
        return {};
    }
    const auto target = state.Target();
    if (target.kind == SelectionKind::Point)
    {
        const Road* road = Selected(world);
        if (!road || state.point >= static_cast<int>(road->points.size()))
        {
            return {};
        }
        const RoadPoint& point = road->points[state.point];
        auto placement = RoadOverlay::PointPlacement(point.position, camera, scene);
        if (placement && gizmo.settings.space == GizmoSpace::Local)
        {
            placement->localAxes = road->PointAxes(state.point);
        }
        return placement;
    }
    const TransformFields fields = Inspect(world, target);
    if (fields.target.kind == SelectionKind::None)
    {
        return {};
    }
    auto placement = GizmoPlacement::At(fields.value.position, camera, scene);
    if (placement && gizmo.settings.space == GizmoSpace::Local)
    {
        const auto frame = ObjectTransform{{}, fields.value.rotation}.ToMeshTransform();
        placement->localAxes = {frame.x, frame.y, frame.z};
    }
    return placement;
}

void RoadEditor::SubmitGizmos(WorldRenderer& renderer, const WorldDocument& world, const FreeCamera& camera) const
{
    if (const Road* road = Selected(world))
    {
        overlay.Draw(renderer, *road, state, camera, scene);
    }
    if (const auto placement = SelectedGizmoPlacement(world, camera))
    {
        // A building or other centered mesh can completely enclose its own handles.
        if (state.meshSelection.kind == SelectionKind::Object) renderer.BeginForeground();
        gizmo.Draw(renderer, *placement, gizmoDrag ? &*gizmoDrag : nullptr);
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
    return {center.x + (x - scene.w * 0.5) / scale, 0.0, center.z - (y - scene.h * 0.5) / scale};
}

SDL_FPoint RoadEditor::ToScreen(Vector3 p) const
{
    return {static_cast<float>((p.x - center.x) * scale + scene.w * 0.5), static_cast<float>((center.z - p.z) * scale + scene.h * 0.5)};
}

SDL_FPoint RoadEditor::Handle(const Road& road, int index) const
{
    const Vector3 tangent = road.Tangent(index);
    const double heading = std::hypot(tangent.x, tangent.z) > 1.0e-8
        ? std::atan2(tangent.z, tangent.x) : road.points[index].heading.value_or(0.0);
    const Vector3 direction{std::cos(heading), 0, std::sin(heading)};
    return ToScreen(road.points[index].position + direction * (handleLength / scale));
}

#ifndef _WIN32
std::vector<RoadEditor::Button> RoadEditor::Buttons(const WorldDocument& world) const
{
    const Road* current = EditingDraft() ? &*state.draft : Selected(world);
    const RoadStyle& style = current ? current->style : state.defaults;
    const int laneCount = style.GetLaneCount();
    const float offset = laneCount == 3 ? threeLanePanelHeight : 0.0f;

    std::vector<Button> buttons{
        {Action::ChangeView, {12, 44, 194, 28}, planView ? "Back to 3D   [R]" : "Top-down view   [R]"},
        {Action::New, {12, 84, 194, 28}, state.draft && !EditingDraft() ? "Resume draft" : "New road", false, !EditingDraft()},
        {Action::Straight, {12, 148, 94, 28}, "Straight", !style.curved},
        {Action::Curve, {112, 148, 94, 28}, "Bezier", style.curved},
        {Action::Fewer, {12, 210, 32, 28}, "-", false, laneCount > (style.IsDivided() ? 2 : 1)},
        {Action::More, {174, 210, 32, 28}, "+", false, laneCount < RoadStyle::maxLanes},
        {Action::Left, {12, 274 + offset, 94, 28}, "Left", style.leftShoulder},
        {Action::Right, {112, 274 + offset, 94, 28}, "Right", style.rightShoulder},
        {Action::NoEdge, {12, 328 + offset, 46, 28}, "None", style.edge == RoadEdge::None},
        {Action::Transition, {62, 328 + offset, 94, 28}, "Transition", style.edge == RoadEdge::Transition},
        {Action::Curb, {160, 328 + offset, 46, 28}, "Curb", style.edge == RoadEdge::Curb},
        {Action::Divide, {12, 368 + offset, 194, 28}, "Divided road", style.IsDivided(), style.CanDivide()}};

    if (style.IsDivided())
    {
        buttons.push_back({Action::NarrowerGap, {12, 422 + offset, 32, 28}, "-", false,
                           style.DividedGap() > RoadStyle::minSeparation});
        buttons.push_back({Action::WiderGap, {174, 422 + offset, 32, 28}, "+"});
        const bool hasMedian = style.HalfMedianWidth() > 0;
        buttons.push_back({Action::ConcreteMedian, {12, 476 + offset, 94, 28}, "Concrete",
                           hasMedian && style.median == RoadMedian::Concrete, hasMedian});
        buttons.push_back({Action::GrassMedian, {112, 476 + offset, 94, 28}, "Grass",
                           hasMedian && style.median == RoadMedian::Grass, hasMedian});
    }
    const float styleBottom = RoadStyleBottom(style);

    if (planView)
    {
        buttons.push_back({Action::Finish, {12, styleBottom + 30, 194, 28}, "Finish road  [Enter]", false,
                           state.drawing && state.draft->points.size() >= 2});
        buttons.push_back({Action::Cancel, {12, styleBottom + 64, 194, 28}, "Cancel draft [Esc]", false, state.drawing});
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

#endif

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
    if (command == EditorCommand::ToggleWireframe)
    {
        menu.ToggleWireframe();
        return;
    }
    Release(world);
    if (command == EditorCommand::CreateRoad)
    {
        constexpr double initialLength = 10.0; // meters along world X
        EditSnapshot before(state);
        Road road;
        road.style = Style(world);
        road.points = {{creationPoint, {}}, {creationPoint + Vector3{initialLength, 0, 0}, {}}};
        road.Rebuild();
        world.roads.push_back(std::move(road));
        state.drawing = false;
        state.SelectPoint(static_cast<int>(world.roads.size()) - 1, 1);
        center = creationPoint;
        Commit(world, state.selected, std::move(before), &world.roads.back());
        return;
    }
    if (command == EditorCommand::CreateTestTerrain)
    {
        constexpr char modelPath[] = "terrain/test_grid.glb";
        const ObjectTransform transform{creationPoint};
        EditSnapshot before(state);
        if (!world.PlaceTerrain(modelPath, transform))
        {
            status = SDL_GetError();
            SDL_Log("%s", status.c_str());
            return;
        }
        state.drawing = false;
        state.SelectPoint(-1, -1);
        state.meshSelection = {SelectionKind::Terrain, static_cast<int>(world.Terrain().size()) - 1};
        EditSnapshot after(state);
        after.addedTerrain = EditSnapshot::TerrainPlacement{modelPath, transform};
        history.Record(-1, std::move(before), std::move(after));
        Save(world);
        return;
    }
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
    const auto result = undo ? history.Undo(world, state) : history.Redo(world, state);
    if (result == EditHistory::Result::Failed)
    {
        status = SDL_GetError();
        return;
    }
    if (result == EditHistory::Result::MapChanged) outliner.Invalidate();
    if (result != EditHistory::Result::Unavailable &&
        (result != EditHistory::Result::MapChanged || Save(world)))
    {
        status = undo ? "Undone" : "Redone";
    }
}

void RoadEditor::OpenViewportMenu(WorldDocument& world, FreeCamera& camera)
{
    auto& picker = rightClick->picker;
    for (const Road& road : world.roads) picker.TestRoad(road);
    creationPoint = picker.PlacementPosition();
    if (const auto command = menu.OpenViewport(rightClick->windowPoint, picker.mouse, scene))
    {
        RunCommand(*command, world, camera);
    }
}

void RoadEditor::PlaceProp(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer, const std::string& path)
{
    if (planView || drag != Drag::None || pointer.x < 0 || pointer.y < 0 || pointer.x >= scene.w || pointer.y >= scene.h)
    {
        return;
    }
    const auto mesh = world.models.Load(path);
    if (!mesh)
    {
        status = SDL_GetError();
        SDL_Log("Place prop: %s", status.c_str());
        return;
    }
    ViewportPicker picker(camera, scene, pointer);
    for (const Road& road : world.roads) picker.TestRoad(road);
    for (const auto& chunk : world.Terrain()) picker.TestMesh(chunk->Instance());

    // Respect the model's authored pivot, including meshes centered above or below it.
    double baseY = mesh->vertices[mesh->indices.front()].position.y;
    for (auto i : mesh->indices) baseY = std::min(baseY, mesh->vertices[i].position.y);
    const ObjectTransform transform{picker.PlacementPosition(baseY)};
    const int index = static_cast<int>(world.Objects().size());
    EditSnapshot before(state);
    if (!world.PlaceObject(ObjectType::Prop, path, transform))
    {
        status = SDL_GetError();
        return;
    }
    state.SelectRows({{SceneKind::Object, index}});
    history.Record(-1, std::move(before), EditSnapshot(state), EditHistory::ObjectEdit{index, 0, {}});
    Save(world);
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
    state.SelectPoint(-1, -1);
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
        state.SelectPoint(static_cast<int>(world.roads.size()) - 1, static_cast<int>(world.roads.back().points.size()) - 1);
        Commit(world, state.selected, std::move(before), &world.roads.back());
        return;
    }
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
        style.SetLaneCount(std::max(style.IsDivided() ? 2 : 1, style.GetLaneCount() - (style.IsDivided() ? 2 : 1)));
        break;
    case Action::More:
        style.SetLaneCount(style.GetLaneCount() + (style.IsDivided() ? 2 : 1));
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
    case Action::NoEdge:
        style.edge = RoadEdge::None;
        break;
    case Action::Transition:
        style.edge = RoadEdge::Transition;
        break;
    case Action::Curb:
        style.edge = RoadEdge::Curb;
        break;
    case Action::Divide:
        style.SetDivided(!style.IsDivided());
        break;
    case Action::NarrowerGap:
    case Action::WiderGap:
        if (!style.IsDivided()) return;
        style.SetDividedGap(style.DividedGap() + (action == Action::WiderGap ? 0.5 : -0.5));
        break;
    case Action::ConcreteMedian:
    case Action::GrassMedian:
        if (style.HalfMedianWidth() <= 0) return;
        style.median = action == Action::GrassMedian ? RoadMedian::Grass : RoadMedian::Concrete;
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

void RoadEditor::PressCanvas(WorldDocument& world, float x, float y, SDL_Keymod modifiers)
{
    const Vector3 pos = ToWorld(x, y);
    const SDL_FPoint cursor{x, y};
    const bool shift = (modifiers & SDL_KMOD_SHIFT) != 0;
    const bool subtract = (modifiers & SDL_KMOD_CTRL) != 0;
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

    if (!shift && !subtract && state.selected >= 0 && state.point >= 0 && Near(cursor, Handle(world.roads[state.selected], state.point)))
    {
        dragStart.emplace(state, Selected(world));
        drag = Drag::Rotate;
        return;
    }

    auto grabPoint = [&](int roadIndex, int point)
    {
        const Road& road = world.roads[roadIndex];
        state.PickPoint(roadIndex, point, shift, subtract);
        if (subtract)
        {
            return;
        }
        grabOffset = road.points[point].position - pos;
        BeginRoadDrag(road, Drag::Move, shift);
    };
    // An elevated endpoint can project exactly onto the preceding point.
    if (state.selected >= 0 && state.point >= 0 &&
        Near(cursor, ToScreen(world.roads[state.selected].points[state.point].position)))
    {
        grabPoint(state.selected, state.point);
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
                grabPoint(roadIndex, static_cast<int>(i));
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
    if (state.point < 0 || (!shift && !subtract))
    {
        SelectAt(world, pos);
    }
}

void RoadEditor::SelectAt(const WorldDocument& world, Vector3 pos)
{
    state.SelectPoint(-1, -1);
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
        for (const auto& edge : world.roads[i].edgeTriangles)
        {
            if (HitTriangle(pos, {edge[0].position, edge[1].position, edge[2].position}))
            {
                state.selected = i;
                return;
            }
        }
    }
}

void RoadEditor::BeginRoadDrag(const Road& road, Drag kind, bool shift)
{
    dragStart.emplace(state, &road);
    drag = kind;
    const bool translate = kind == Drag::Move || gizmo.mode == GizmoMode::Translate;
    const bool endpoint = state.point == 0 || state.point + 1 == static_cast<int>(road.points.size());
    if (state.point >= 0 && translate && endpoint && shift)
    {
        extensionSource = state.point;
    }
    if (kind == Drag::Gizmo || extensionSource)
    {
        originalPoints = road.points;
        originalRotation = road.rotation;
    }
}

void RoadEditor::MovePoint(Road& road, Vector3 position)
{
    if (road.points[state.point].position == position)
    {
        return;
    }
    // Allow the smallest snapped step while retaining the free-drag spacing guard.
    const double minSpacing = drag == Drag::Gizmo && gizmo.settings.snapEnabled
        ? std::min(0.05, gizmo.settings.StepMeters() * 0.5) : 0.05;
    if (extensionSource)
    {
        const bool inserted = road.points.size() != originalPoints.size();
        if (Math::Length(position - originalPoints[*extensionSource].position) < minSpacing)
        {
            // Returning to the starting point withdraws the preview.
            if (inserted)
            {
                road.points = originalPoints;
                state.additionalPoints = dragStart->additionalPoints;
                state.point = *extensionSource;
                road.Rebuild();
                dragChanged = false;
            }
            return;
        }
        if (!inserted)
        {
            state.additionalPoints.clear();
            state.point = *extensionSource == 0 ? 0 : static_cast<int>(road.points.size());
            road.points.insert(road.points.begin() + state.point, RoadPoint{position, {}});
        }
    }
    else if ((state.point > 0 && Math::Length(position - road.points[state.point - 1].position) < minSpacing) ||
             (state.point + 1 < static_cast<int>(road.points.size()) &&
              Math::Length(position - road.points[state.point + 1].position) < minSpacing))
    {
        return;
    }

    road.points[state.point].position = position;
    road.Rebuild();
    dragChanged = true;
}

void RoadEditor::Motion(WorldDocument& world, const SDL_MouseMotionEvent& motion)
{
    mouse = {motion.x, motion.y};
    if (drag == Drag::Measure)
    {
        if (!(motion.state & SDL_BUTTON_MMASK))
        {
            Release(world);
            return;
        }
        // Keep the moving end on the canvas when a captured drag crosses into a panel.
        const float x = std::clamp(mouse.x, 0.0f, static_cast<float>(std::max(0, scene.w - 1)));
        const float y = std::clamp(mouse.y, 0.0f, static_cast<float>(std::max(0, scene.h - 1)));
        measureEnd = ToWorld(x, y);
    }
    else if (drag == Drag::Pan)
    {
        center.x -= motion.xrel / scale;
        center.z += motion.yrel / scale;
    }
    else if (drag == Drag::Move)
    {
        MovePoint(world.roads[state.selected], ToWorld(mouse.x, mouse.y) + grabOffset);
    }
    else if (drag == Drag::Rotate)
    {
        Road& road = world.roads[state.selected];
        const Vector3 offset = ToWorld(mouse.x, mouse.y) - road.points[state.point].position;
        if (std::hypot(offset.x, offset.z) * scale >= 2.0)
        {
            road.RotatePoint(state.point, std::atan2(offset.z, offset.x));
            road.Rebuild();
            dragChanged = true;
        }
    }
}

void RoadEditor::Release(WorldDocument& world)
{
    if (dragStart && dragChanged)
    {
        CommitTransform(world, std::move(*dragStart));
    }

    originalPoints.clear();
    extensionSource.reset();
    gizmoDrag.reset();
    dragStart.reset();
    dragChanged = false;
    drag = Drag::None;
}

bool RoadEditor::DrawMeasurement(SDL_Renderer* renderer) const
{
    const SDL_FPoint a = ToScreen(measureStart), b = ToScreen(measureEnd);
    if (!EditorTheme::SetColor(renderer, ink) || !SDL_RenderLine(renderer, a.x, a.y, b.x, b.y))
    {
        return false;
    }

    const float dx = b.x - a.x, dy = b.y - a.y;
    const float length = std::hypot(dx, dy);
    if (length > 0.0f)
    {
        constexpr float halfTick = 4.0f;
        const SDL_FPoint tick{-dy / length * halfTick, dx / length * halfTick};
        for (SDL_FPoint p : {a, b})
        {
            if (!SDL_RenderLine(renderer, p.x - tick.x, p.y - tick.y, p.x + tick.x, p.y + tick.y))
            {
                return false;
            }
        }
    }

    const double meters = std::hypot(measureEnd.x - measureStart.x, measureEnd.z - measureStart.z);
    char label[64];
    SDL_snprintf(label, sizeof(label), "%.3f m", meters);
    constexpr float pad = 3.0f, glyph = SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE;
    const float width = static_cast<float>(SDL_strlen(label)) * glyph + 2 * pad;
    const float height = glyph + 2 * pad;
    const float x = (a.x + b.x - width) * 0.5f;
    float y = (a.y + b.y - height) * 0.5f;
    if (length < width + 8.0f)
    {
        y = std::min(a.y, b.y) - height - pad;
    }
    const SDL_FRect box{std::clamp(x, 0.0f, std::max(0.0f, scene.w - width)),
                        std::clamp(y, 0.0f, std::max(0.0f, scene.h - height)), width, height};
    return Fill(renderer, box, EditorTheme::preview) && Text(renderer, box.x + pad, box.y + pad, label, ink);
}

bool RoadEditor::RenderPlan(SDL_Renderer* renderer, const WorldDocument& world) const
{
    constexpr SDL_Color amber = EditorColors::ToBytes(EditorColors::amber);
    constexpr SDL_Color blue = EditorColors::ToBytes(EditorColors::blue);

    if (!EditorTheme::SetColor(renderer, EditorTheme::planBackground) || !SDL_RenderClear(renderer))
    {
        return false;
    }

    const SDL_Rect canvas{0, 0, scene.w, scene.h};
    double spacing = std::pow(10.0, std::floor(std::log10(100.0 / scale)));

    if (spacing * scale < 35.0)
    {
        spacing *= 5.0;
    }

    const float step = static_cast<float>(spacing * scale);
    const SDL_FPoint origin = ToScreen({});

    if (!EditorTheme::SetColor(renderer, EditorTheme::grid))
    {
        return false;
    }

    for (float x = std::fmod(origin.x, step); x < scene.w; x += step)
    {
        if (!SDL_RenderLine(renderer, x, 0, x, static_cast<float>(scene.h)))
        {
            return false;
        }
    }

    for (float y = std::fmod(origin.y, step); y < scene.h; y += step)
    {
        if (!SDL_RenderLine(renderer, 0, y, static_cast<float>(scene.w), y))
        {
            return false;
        }
    }

    std::optional<Road> preview;

    if (state.drawing && !state.draft->points.empty())
    {
        preview = state.draft;
        if (drag != Drag::Measure && mouse.x >= 0 && mouse.x < scene.w && mouse.y >= 0 && mouse.y < scene.h)
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
            if (pos.x < -8 || pos.x > scene.w + 8 || pos.y < -8 || pos.y > scene.h + 8)
            {
                continue;
            }
            const bool selectedPoint = active && state.PointSelected(static_cast<int>(i));
            const float r = active ? 5.0f : 3.0f;
            if (!Fill(renderer, {pos.x - r - 1, pos.y - r - 1, 2 * r + 2, 2 * r + 2}, EditorTheme::border) ||
                !Fill(renderer, {pos.x - r, pos.y - r, 2 * r, 2 * r},
                      selectedPoint ? amber : (active ? blue : muted)))
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
        if (a.x > -40 && a.x < scene.w + 40 && a.y > -40 && a.y < scene.h + 40)
        {
            if (!SDL_SetRenderDrawColor(renderer, amber.r, amber.g, amber.b, 255) ||
                !SDL_RenderLine(renderer, a.x, a.y, b.x, b.y) || !Fill(renderer, {b.x - 4, b.y - 4, 8, 8}, amber))
            {
                return false;
            }
        }
    }

    if (!SDL_RenderDebugTextFormat(renderer, 12, static_cast<float>(scene.h) - 20, "Grid: %.0f m    Roads: %zu    %s", spacing, world.roads.size(), status.c_str()))
    {
        return false;
    }

    return drag != Drag::Measure || DrawMeasurement(renderer);
}

#ifndef _WIN32
bool RoadEditor::RenderRoadPanel(SDL_Renderer* renderer, const WorldDocument& world) const
{
    if (!Fill(renderer, {0, 0, static_cast<float>(roadPanel.w), static_cast<float>(roadPanel.h + roadScroll)}, EditorTheme::panel))
    {
        return false;
    }

    for (const Button& b : Buttons(world))
    {
        const SDL_Color fill = b.active ? EditorTheme::selected : EditorTheme::button;
        if (!Fill(renderer, b.rect, b.enabled ? fill : EditorTheme::header) ||
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
    const float styleBottom = RoadStyleBottom(style);
    if (style.IsDivided() &&
        (!Text(renderer, 12, 406 + offset, "Separation (m)", muted) ||
         !SDL_RenderDebugTextFormat(renderer, 80, 432 + offset, "%.2f", style.DividedGap()) ||
         !Text(renderer, 12, 460 + offset, "Median", muted))) return false;
    if (!Text(renderer, 12, 132, "Alignment", muted) || !Text(renderer, 12, 194, "Lanes (total)", muted) ||
        !Text(renderer, 12, 258 + offset, "Shoulders (5 m each)", muted) ||
        !Text(renderer, 12, 312 + offset, "Road edges", muted) ||
        !SDL_RenderDebugTextFormat(renderer, 89, 220, "%d", laneCount) ||
        !SDL_RenderDebugTextFormat(renderer, 12, styleBottom, "Width: %.2f m",
                                   style.LeftWidth() + style.RightWidth() + 2.0 * style.EdgeWidth()))
    {
        return false;
    }

    if (!planView && current && state.point >= 0)
    {
        if (!SDL_SetRenderDrawColor(renderer, ink.r, ink.g, ink.b, 255) ||
            !SDL_RenderDebugTextFormat(renderer, 12, styleBottom + 28, "Point %d   Y: %.2f m",
                                       state.point + 1, current->points[state.point].position.y))
        {
            return false;
        }
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

#endif

bool RoadEditor::SetContentViewport(SDL_Renderer* renderer)
{
    int width, height;
    if (!SDL_GetCurrentRenderOutputSize(renderer, &width, &height))
    {
        return false;
    }
#ifdef _WIN32
    scene = {0, 0, width, height};
#else
    const int top = std::min(EditorMenu::barHeight + CameraToolbar::height, height);
    const int right = std::min(322, width / 2);
    const int left = std::min(panelWidth, (width - right) / 2);
    browserPanel = {0, top, left, height - top};
    const int outlineHeight = std::min(250, (height - top) / 3);
    outlineArea = {width - right, top, right, outlineHeight};
    const int transformHeight = std::min(height - top - outlineHeight, SdlTransformPanel::height);
    transformArea = {width - right, top + outlineHeight, right, transformHeight};
    roadPanel = {width - right, transformArea.y + transformHeight, right, height - transformArea.y - transformHeight};
    scene = {left, top, width - left - right, height - top};
#endif
    return SDL_SetRenderViewport(renderer, &scene);
}

bool RoadEditor::Render(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera)
{
    menu.SetHistoryEnabled(history.CanUndo() || dragChanged, history.CanRedo() && !dragChanged);

    if (planView)
    {
        if (!RenderPlan(renderer, world))
        {
            return false;
        }
    }
    else
    {
        const int minutes = static_cast<int>(g_WorldState.GetTimeOfDayMinutes());

        if (!SDL_SetRenderDrawColor(renderer, ink.r, ink.g, ink.b, ink.a) ||
            !SDL_RenderDebugTextFormat(renderer, 12, 12, "Sky %02d:%02d CDT", minutes / 60, minutes % 60) ||
            !Text(renderer, 12, static_cast<float>(scene.h) - 20, status.c_str(), ink))
        {
            return false;
        }
    }

    outliner.Update(world, state);
    auto fields = Inspect(world, state.Target());

    if (state.outlineSelection.size() > 1)
    {
        fields.selectionLabel = std::to_string(state.outlineSelection.size()) + " items selected";
    }
    else if (!state.outlineSelection.empty())
    {
        const auto key = *state.outlineSelection.begin();
        if (key.kind == SceneKind::Mesh) fields.selectionLabel = "Static mesh " + std::to_string(key.index + 1);
        if (key.kind == SceneKind::Folder) fields.selectionLabel = "Folder";
        if (key.kind == SceneKind::World) fields.selectionLabel = "World";
    }
#ifdef _WIN32
    panels.SyncOutliner(outliner);
    panels.Sync(state, EditingDraft() ? &*state.draft : Selected(world), planView, camera, gizmo.settings, fields);
    return true;
#else
    if (!draggedModel.empty() && mouse.x >= 0 && mouse.y >= 0 && mouse.x < scene.w && mouse.y < scene.h &&
        !Text(renderer, mouse.x + 14, mouse.y + 14, draggedModel.c_str(), ink))
    {
        return false;
    }
    if (!browser.MaterialWindow().Draw() || !menu.DrawViewportMenu(renderer))
    {
        return false;
    }
    const Road* current = EditingDraft() ? &*state.draft : Selected(world);
    const RoadStyle& style = current ? current->style : state.defaults;
    const int contentHeight = static_cast<int>(RoadStyleBottom(style)) + (planView ? 102 : 50);
    roadScrollMax = std::max(0, contentHeight - roadPanel.h);
    roadScroll = std::min(roadScroll, roadScrollMax);
    const int width = roadPanel.x + roadPanel.w;
    const SDL_Rect toolbar{0, EditorMenu::barHeight, width, CameraToolbar::height};
    const SDL_Rect shifted{roadPanel.x, roadPanel.y - roadScroll, roadPanel.w, roadPanel.h + roadScroll};
    const SDL_Rect clip{0, roadScroll, roadPanel.w, roadPanel.h};
    return SDL_SetRenderViewport(renderer, &browserPanel) && browser.Draw(renderer, browserPanel.w, browserPanel.h) &&
           SDL_SetRenderViewport(renderer, &outlineArea) && outlinePanel.Draw(renderer, outliner, outlineArea.w, outlineArea.h) &&
           SDL_SetRenderViewport(renderer, &transformArea) && transformPanel.Draw(renderer, transformArea.w, fields) &&
           SDL_SetRenderViewport(renderer, &shifted) && SDL_SetRenderClipRect(renderer, &clip) &&
           RenderRoadPanel(renderer, world) && SDL_SetRenderClipRect(renderer, nullptr) &&
           SDL_SetRenderViewport(renderer, &toolbar) && cameraToolbar.Draw(renderer, width, camera, gizmo.settings) &&
           SDL_SetRenderViewport(renderer, nullptr) && menu.DrawBar(renderer, width);
#endif
}

void RoadEditor::ChangeView(WorldDocument& world, FreeCamera& camera)
{
    Release(world);
    rightClick.reset();
    planView = !planView;
    if (planView && !initialized)
    {
        if (state.meshSelection.kind != SelectionKind::None)
        {
            center = Inspect(world, state.Target()).value.position;
        }
        else if (world.roads.empty())
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
        const double distance = std::max(15.0, std::min(scene.w, scene.h) * 0.5 / scale);
        camera.LookAt(center + Vector3{0.0, distance, -distance}, center);
    }
}

void RoadEditor::PickWorldRoad(WorldDocument& world, const FreeCamera& camera, SDL_Keymod modifiers)
{
    const bool shift = (modifiers & SDL_KMOD_SHIFT) != 0;
    const bool subtract = (modifiers & SDL_KMOD_CTRL) != 0;
    ViewportPicker picker(camera, scene, mouse);
    const Road* selected = Selected(world);
    const RoadPick hit = overlay.Pick(picker, world, selected, camera, scene);
    const auto placement = SelectedGizmoPlacement(world, camera);
    // A marker at the road pivot must still be selectable when the handles overlap it.
    const bool pickPoint = hit.kind == RoadPickKind::Point && (state.point < 0 || (shift && hit.index != state.point));
    const bool addSegment = state.point < 0 && shift && hit.kind == RoadPickKind::Segment;
    if (placement && !subtract && !pickPoint && !addSegment)
    {
        ViewportPicker handles(camera, scene, mouse);
        gizmoDrag = gizmo.BeginDrag(state.meshSelection.kind == SelectionKind::Object ? handles : picker, *placement);
        if (gizmoDrag)
        {
            if (selected)
            {
                BeginRoadDrag(*selected, Drag::Gizmo, shift);
            }
            else
            {
                dragStart.emplace(state);
                dragStart->meshTransform = Inspect(world, state.meshSelection).value;
                drag = Drag::Gizmo;
            }
            return;
        }
    }

    // Keep the current point or segment selection when a modified click misses it.
    if ((!state.segments.empty() || state.point >= 0) && (shift || subtract) && (hit.kind == RoadPickKind::Road || hit.kind == RoadPickKind::Empty))
    {
        return;
    }
    switch (hit.kind)
    {
    case RoadPickKind::Segment:
        if (subtract)
        {
            state.segments.erase(hit.index);
        }
        else
        {
            if (!shift)
            {
                state.segments.clear();
            }
            state.segments.insert(hit.index);
            state.point = -1;
            state.additionalPoints.clear();
        }
        break;
    case RoadPickKind::Point:
        state.PickPoint(state.selected, hit.index, shift, subtract);
        break;
    case RoadPickKind::Road:
    {
        state.drawing = false;
        center = picker.HitPosition();
        state.SelectPoint(hit.index, -1);
        break;
    }
    case RoadPickKind::Object:
    case RoadPickKind::Terrain:
        state.drawing = false;
        state.SelectPoint(-1, -1);
        state.meshSelection = {hit.kind == RoadPickKind::Object ? SelectionKind::Object : SelectionKind::Terrain, hit.index};
        break;
    case RoadPickKind::Mesh:
        state.SelectRows({{SceneKind::Mesh, hit.index}});
        break;
    case RoadPickKind::Empty:
        state.SelectPoint(-1, -1);
        break;
    }
}

void RoadEditor::DeleteSelection(WorldDocument& world)
{
    if (EditingDraft() || drag != Drag::None) return;
    if (state.meshSelection.kind == SelectionKind::Object)
    {
        const int index = state.meshSelection.index;
        const int folder = world.hierarchy.Parent({SceneKind::Object, index});
        EditSnapshot before(state);
        auto object = world.TakeObject(index);
        if (!object)
        {
            status = SDL_GetError();
            return;
        }
        state.SelectRows({});
        history.Record(-1, std::move(before), EditSnapshot(state), EditHistory::ObjectEdit{index, folder, std::move(object)});
        Save(world);
        return;
    }
    const Road* road = Selected(world);
    if (!road)
    {
        return;
    }

    std::vector<Road> remaining;
    if (!state.segments.empty())
    {
        // Each uninterrupted run of surviving segments owns both of its end points.
        const int segmentCount = static_cast<int>(road->points.size()) - 1;
        int start = 0;
        for (int deleted : state.segments)
        {
            if (start < deleted)
            {
                remaining.push_back(road->Section(start, deleted));
            }
            start = deleted + 1;
        }
        if (start < segmentCount)
        {
            remaining.push_back(road->Section(start, segmentCount));
        }
    }
    else if (state.point >= 0)
    {
        Road connected;
        connected.style = road->style;
        connected.rotation = road->rotation;
        for (int i = 0; i < static_cast<int>(road->points.size()); ++i)
        {
            if (!state.PointSelected(i))
            {
                connected.points.push_back(road->points[i]);
            }
        }
        if (connected.points.size() >= 2)
        {
            connected.Rebuild();
            remaining.push_back(std::move(connected));
        }
    }

    history.ReplaceSelectedRoad(world, state, std::move(remaining));
    Save(world);
}

void RoadEditor::ApplyOutline(const OutlineCommand& cmd, WorldDocument& world)
{
    Release(world);
    if (cmd.op == OutlineOp::Remove && cmd.target.kind == SceneKind::Object)
    {
        if (state.meshSelection == TransformTarget{SelectionKind::Object, cmd.target.index}) DeleteSelection(world);
    }
    else if (outliner.Apply(cmd, world, state, history))
    {
        Save(world);
    }
    else if (cmd.op == OutlineOp::Create || cmd.op == OutlineOp::Rename)
    {
        const auto& folders = world.hierarchy.Folders();
        const auto it = folders.find(cmd.target.index);
        if (cmd.op == OutlineOp::Create || it == folders.end() || it->second.name != cmd.name)
        {
            status = "Use a unique, nonempty folder name (up to 255 UTF-8 bytes)";
        }
    }
    outliner.Update(world, state);
}

void RoadEditor::ApplyInspectorEdit(WorldDocument& world, const TransformEdit& edit)
{
    if (edit.target != state.Target() || drag != Drag::None) return;
    const TransformFields fields = Inspect(world, edit.target);
    if (!fields.CanEdit(edit.field)) return;
    const auto number = ParseTransformValue(edit.text);
    if (!number)
    {
        status = "Enter a finite number";
        return;
    }
    if (*number == fields.DisplayValue(edit.field)) return;
    const Road* road = Selected(world);
    EditSnapshot before(state, road);
    if (!road) before.meshTransform = fields.value;
    if (!ApplyTransform(world, fields, edit.field, *number))
    {
        status = SDL_GetError();
        return;
    }
    CommitTransform(world, std::move(before));
}

void RoadEditor::CommitTransform(WorldDocument& world, EditSnapshot before)
{
    if (before.meshTransform)
    {
        EditSnapshot after(state);
        after.meshTransform = Inspect(world, state.meshSelection).value;
        if (history.Record(-1, std::move(before), std::move(after)))
        {
            Save(world);
        }
    }
    else
    {
        Commit(world, state.selected, std::move(before), Selected(world));
    }
}

void RoadEditor::MoveGizmo(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer)
{
    mouse = pointer;
    const auto amount = gizmoDrag->Amount(camera, scene, pointer, gizmo.settings);
    if (!amount)
    {
        return;
    }
    if (state.point < 0)
    {
        const bool mesh = dragStart->meshTransform.has_value();
        const ObjectTransform start = mesh ? *dragStart->meshTransform
                                           : ObjectTransform{originalPoints.front().position, originalRotation};
        ObjectTransform pose = start;
        if (gizmo.mode == GizmoMode::Translate)
        {
            pose.position = start.position + gizmoDrag->axis * amount.value();
        }
        else
        {
            pose.Rotate(gizmoDrag->axis, *amount);
        }
        if (mesh)
        {
            if (SetMeshTransform(world, state.meshSelection, pose))
            {
                dragChanged = pose != start;
            }
            return;
        }
        Road& road = world.roads[state.selected];
        if (pose.position == road.points.front().position && pose.rotation == road.rotation)
        {
            return;
        }
        // Drag amounts are absolute from mouse-down. Reuse the original points and orientation each frame.
        road.points = originalPoints;
        road.rotation = originalRotation;
        if (road.SetPose(pose.position, pose.rotation))
        {
            dragChanged = pose != start;
        }
        else
        {
            road.Rebuild();
            dragChanged = false;
        }
        return;
    }
    Road& road = world.roads[state.selected];
    if (gizmo.mode == GizmoMode::Translate)
    {
        const int source = extensionSource.value_or(state.point);
        MovePoint(road, originalPoints[source].position + gizmoDrag->axis * amount.value());
        return;
    }

    road.points = originalPoints;
    if (*amount != 0.0)
    {
        road.RotatePoint(state.point, gizmoDrag->axis, *amount);
    }
    road.Rebuild();
    dragChanged = true;
}

EditorEvent RoadEditor::HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event,
                                   WorldDocument& world, FreeCamera& camera)
{
#ifndef _WIN32
    if (browser.MaterialWindow().Handle(event)) return EditorEvent::Handled;
#endif
    if (event.type == SDL_EVENT_QUIT)
    {
        Release(world);
        return EditorEvent::Pass;
    }
    if (rightClick && SDL_GetWindowFromEvent(&event) == window)
    {
        if (event.type == SDL_EVENT_MOUSE_MOTION)
        {
            // Relative mode can warp the absolute cursor. Measure the drag in relative pixels.
            rightClick->motion.x += event.motion.xrel;
            rightClick->motion.y += event.motion.yrel;
            constexpr float dragThreshold = 4.0f;
            if (std::hypot(rightClick->motion.x, rightClick->motion.y) >= dragThreshold)
            {
                rightClick.reset();
            }
        }
        else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_RIGHT)
        {
            if (!planView && ViewportFocused() && SDL_GetWindowRelativeMouseMode(window) &&
                camera.GetPosition() == rightClick->picker.eye)
            {
                if (!SDL_SetWindowRelativeMouseMode(window, false))
                {
                    return EditorEvent::Error;
                }
                SDL_WarpMouseInWindow(window, rightClick->windowPoint.x, rightClick->windowPoint.y);
                OpenViewportMenu(world, camera);
                rightClick.reset();
                return EditorEvent::Handled;
            }
            rightClick.reset();
        }
        else if (event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_KEY_UP ||
                 event.type == SDL_EVENT_MOUSE_WHEEL || event.type == SDL_EVENT_MOUSE_BUTTON_DOWN ||
                 event.type == SDL_EVENT_WINDOW_FOCUS_LOST || event.type == SDL_EVENT_WINDOW_RESIZED)
        {
            rightClick.reset();
        }
    }
#ifndef _WIN32
    if (SDL_GetWindowFromEvent(&event) == window)
    {
        if (browser.HandleMaterialPopup(window, renderer, event, browserPanel)) return EditorEvent::Handled;
        if (!draggedModel.empty())
        {
            if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT)
            {
                SDL_Event release = event;
                if (!SetContentViewport(renderer) || !SDL_ConvertEventToRenderCoordinates(renderer, &release))
                    return EditorEvent::Error;
                if (std::hypot(event.button.x - modelPress.x, event.button.y - modelPress.y) >= 4.0f)
                    PlaceProp(world, camera, {release.button.x, release.button.y}, draggedModel);
                draggedModel.clear();
                return EditorEvent::Handled;
            }
            if (event.type == SDL_EVENT_MOUSE_MOTION)
            {
                SDL_Event motion = event;
                if (!SetContentViewport(renderer) || !SDL_ConvertEventToRenderCoordinates(renderer, &motion))
                    return EditorEvent::Error;
                mouse = {motion.motion.x, motion.motion.y};
                if (!(motion.motion.state & SDL_BUTTON_LMASK)) draggedModel.clear();
                return EditorEvent::Handled;
            }
            if (event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
            {
                if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN || event.key.key == SDLK_ESCAPE) draggedModel.clear();
                return EditorEvent::Handled;
            }
            if (event.type == SDL_EVENT_MOUSE_WHEEL) return EditorEvent::Handled;
            if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) draggedModel.clear();
        }
        std::optional<EditorCommand> command;
        if (menu.HandleViewportMenu(renderer, scene, event, command) ||
            (drag == Drag::None && !SDL_GetWindowRelativeMouseMode(window) && menu.HandleBar(renderer, event, command)))
        {
            if (command)
            {
                RunCommand(*command, world, camera);
            }
            return EditorEvent::Handled;
        }
        std::optional<OutlineCommand> outlineCommand;
        if (outlinePanel.Key(window, event, outliner, outlineCommand))
        {
            if (outlineCommand) ApplyOutline(*outlineCommand, world);
            return EditorEvent::Handled;
        }
        std::optional<TransformEdit> edit;
        if (transformPanel.Key(window, event, edit))
        {
            if (edit) ApplyInspectorEdit(world, *edit);
            return EditorEvent::Handled;
        }
        if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN || event.type == SDL_EVENT_WINDOW_FOCUS_LOST)
            transformPanel.Blur(window);
    }
#endif
    if (const auto command = menu.Command(event))
    {
        RunCommand(*command, world, camera);
        return EditorEvent::Handled;
    }
#ifdef _WIN32
    if (const auto drop = panels.TakeModelDrop(event))
    {
        if (!SetContentViewport(renderer)) return EditorEvent::Error;
        PlaceProp(world, camera, {drop->viewportFraction.x * scene.w, drop->viewportFraction.y * scene.h}, drop->path);
        return EditorEvent::Handled;
    }
    if (const auto cmd = panels.TakeOutlineCommand(event))
    {
        ApplyOutline(*cmd, world);
        return EditorEvent::Handled;
    }
    if (const auto edit = panels.TakeTransformEdit(event))
    {
        ApplyInspectorEdit(world, *edit);
        return EditorEvent::Handled;
    }
    Action action;
    if (panels.Command(event, action))
    {
        Release(world);
        Activate(action, world, camera);
        return EditorEvent::Handled;
    }
    if (panels.HandleInput(event, camera, gizmo.settings))
    {
        return EditorEvent::Handled;
    }
#endif
    if (SDL_GetWindowFromEvent(&event) != window)
    {
        return EditorEvent::Pass;
    }
    if (event.type == SDL_EVENT_WINDOW_MOUSE_LEAVE && drag == Drag::None)
    {
        mouse = {-1, -1};
    }
    if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST)
    {
#ifndef _WIN32
        cameraToolbar.CancelDrag();
#endif
        Release(world);
        return EditorEvent::Pass;
    }
    if (ViewportFocused() && event.type == SDL_EVENT_KEY_DOWN &&
        !(event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_GUI | SDL_KMOD_SHIFT)))
    {
        if (event.key.key == SDLK_LEFTBRACKET || event.key.key == SDLK_RIGHTBRACKET)
        {
            if (!event.key.repeat)
            {
                gizmo.settings.ChangeStep(event.key.key == SDLK_RIGHTBRACKET ? 1 : -1);
                if (DraggingGizmo() && gizmo.mode == GizmoMode::Translate)
                {
                    MoveGizmo(world, camera, mouse);
                }
            }
            return EditorEvent::Handled;
        }
        if (!planView && (event.key.key == SDLK_COMMA || event.key.key == SDLK_PERIOD))
        {
            const int minutes = static_cast<int>(g_WorldState.GetTimeOfDayMinutes());
            g_WorldState.SetTimeOfDayMinutes(minutes + (event.key.key == SDLK_PERIOD ? 30 : -30));
            return EditorEvent::Handled;
        }
    }
    if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat)
    {
        if (event.key.key == SDLK_DELETE && ViewportFocused() && !SDL_GetWindowRelativeMouseMode(window))
        {
            DeleteSelection(world);
            return EditorEvent::Handled;
        }
        if (drag == Drag::Measure && event.key.key == SDLK_ESCAPE)
        {
            Release(world);
            return EditorEvent::Handled;
        }
        if ((DraggingGizmo() || extensionSource) && event.key.key == SDLK_ESCAPE)
        {
            if (dragStart->meshTransform)
            {
                SetMeshTransform(world, state.meshSelection, *dragStart->meshTransform);
            }
            else
            {
                Road& road = world.roads[state.selected];
                state.point = extensionSource.value_or(state.point);
                state.additionalPoints = dragStart->additionalPoints;
                road.points = std::move(originalPoints);
                road.rotation = originalRotation;
                road.Rebuild();
            }
            dragChanged = false;
            Release(world);
            return EditorEvent::Handled;
        }
        if (!planView && state.Target().kind != SelectionKind::None && !SDL_GetWindowRelativeMouseMode(window) &&
            !(event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_GUI)) &&
            (event.key.key == SDLK_W || event.key.key == SDLK_E))
        {
            if (!DraggingGizmo())
            {
                gizmo.mode = event.key.key == SDLK_W ? GizmoMode::Translate : GizmoMode::Rotate;
            }
            return EditorEvent::Handled;
        }
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

    // Relative mouse motion and right-button release belong to 3D look.
    if (SDL_GetWindowRelativeMouseMode(window) ||
        (event.type != SDL_EVENT_MOUSE_BUTTON_DOWN && event.type != SDL_EVENT_MOUSE_BUTTON_UP &&
         event.type != SDL_EVENT_MOUSE_MOTION && event.type != SDL_EVENT_MOUSE_WHEEL))
    {
        return EditorEvent::Pass;
    }

    if (!SetContentViewport(renderer))
    {
        return EditorEvent::Error;
    }
    SDL_Event e = event;
#ifdef _WIN32
    panels.ToViewport(window, e, scene);
#else
    // Convert window pixels once, relative to the full area under the toolbar.
    const int windowWidth = roadPanel.x + roadPanel.w;
    const SDL_Rect body{0, scene.y, windowWidth, scene.h};
    if (!SDL_SetRenderViewport(renderer, &body) || !SDL_ConvertEventToRenderCoordinates(renderer, &e))
    {
        return EditorEvent::Error;
    }
#endif
    SDL_FPoint pointer;
    switch (e.type)
    {
    case SDL_EVENT_MOUSE_WHEEL: pointer = {e.wheel.mouse_x, e.wheel.mouse_y}; break;
    case SDL_EVENT_MOUSE_MOTION: pointer = {e.motion.x, e.motion.y}; break;
    default: pointer = {e.button.x, e.button.y}; break;
    }

    if (drag == Drag::None)
    {
#ifdef _WIN32
        if (pointer.x < 0 || pointer.y < 0 || pointer.x >= scene.w || pointer.y >= scene.h)
        {
            mouse = {-1, -1};
            return EditorEvent::Handled;
        }
#else
        const SDL_FPoint localOutline{pointer.x - outlineArea.x, pointer.y};
        if (outlinePanel.Tracking() || (localOutline.x >= 0 && localOutline.x < outlineArea.w &&
                                       localOutline.y >= 0 && localOutline.y < outlineArea.h))
        {
            mouse = {-1,-1};
            outliner.Update(world, state);
            std::optional<OutlineCommand> cmd;
            outlinePanel.Pointer(window, e, localOutline, outlineArea.w, outlineArea.h, outliner, cmd);
            if (cmd) ApplyOutline(*cmd, world);
            return EditorEvent::Handled;
        }
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) outlinePanel.Blur(window);
        if (e.type == SDL_EVENT_MOUSE_WHEEL && pointer.y < 0)
        {
            return EditorEvent::Pass; // The toolbar wheel adjusts free-camera speed.
        }
        if (cameraToolbar.HandlePointer(e, windowWidth, camera, gizmo.settings))
        {
            mouse = {-1, -1};
            return EditorEvent::Handled;
        }
        if (pointer.y < 0 || pointer.y >= scene.h || pointer.x < 0 || pointer.x >= windowWidth)
        {
            mouse = {-1, -1};
            return EditorEvent::Handled;
        }
        if (pointer.x < scene.x)
        {
            mouse = {-1, -1};
            auto path = browser.HandlePointer(e, pointer, browserPanel.w, browserPanel.h);
            if (!planView && !path.empty())
            {
                draggedModel = std::move(path);
                modelPress = {event.button.x, event.button.y};
            }
            return EditorEvent::Handled;
        }
        if (pointer.x >= roadPanel.x)
        {
            mouse = {-1, -1};
            if (e.type == SDL_EVENT_MOUSE_WHEEL && roadScrollMax > 0 && pointer.y >= outlineArea.h + transformArea.h)
            {
                const float wheel = e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -e.wheel.y : e.wheel.y;
                roadScroll = std::clamp(roadScroll - static_cast<int>(wheel * 32), 0, roadScrollMax);
                return EditorEvent::Handled;
            }
            if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_LEFT)
            {
                if (pointer.y < outlineArea.h + transformArea.h)
                {
                    transformPanel.Click(window, {pointer.x - transformArea.x, pointer.y - outlineArea.h}, transformArea.w, Inspect(world, state.Target()));
                    return EditorEvent::Handled;
                }
                const SDL_FPoint local{pointer.x - roadPanel.x, pointer.y - outlineArea.h - transformArea.h + roadScroll};
                for (const Button& button : Buttons(world))
                {
                    if (button.enabled && SDL_PointInRectFloat(&local, &button.rect))
                    {
                        Activate(button.action, world, camera);
                        break;
                    }
                }
            }
            return e.type == SDL_EVENT_MOUSE_WHEEL && !planView ? EditorEvent::Pass : EditorEvent::Handled;
        }
#endif
    }
    if (!planView && !DraggingGizmo() && e.type != SDL_EVENT_MOUSE_BUTTON_DOWN)
    {
        return EditorEvent::Pass;
    }

    pointer.x -= scene.x;
    switch (e.type)
    {
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
        if (drag != Drag::None)
        {
            break;
        }
        mouse = pointer;
        if (e.button.button == SDL_BUTTON_LEFT)
        {
#ifdef _WIN32
            const SDL_Keymod modifiers = panels.TakePressModifiers();
#else
            const SDL_Keymod modifiers = SDL_GetModState();
#endif
            if (planView)
            {
                PressCanvas(world, mouse.x, mouse.y, modifiers);
            }
            else
            {
                PickWorldRoad(world, camera, modifiers);
            }
        }
        else if (!planView)
        {
            if (e.button.button == SDL_BUTTON_RIGHT)
            {
                rightClick.emplace(ViewportClick{{event.button.x, event.button.y}, ViewportPicker(camera, scene, pointer)});
            }
            return EditorEvent::Pass;
        }
        else if (e.button.button == SDL_BUTTON_MIDDLE)
        {
            measureStart = ToWorld(pointer.x, pointer.y);
            measureEnd = measureStart;
            drag = Drag::Measure;
        }
        else if (e.button.button == SDL_BUTTON_RIGHT)
        {
            drag = Drag::Pan;
        }
        break;
    case SDL_EVENT_MOUSE_MOTION:
        if (DraggingGizmo())
        {
            MoveGizmo(world, camera, pointer);
        }
        else
        {
            e.motion.x = pointer.x;
            Motion(world, e.motion);
        }
        break;
    case SDL_EVENT_MOUSE_BUTTON_UP:
    {
        const auto button = drag == Drag::Measure ? SDL_BUTTON_MIDDLE :
                            drag == Drag::Pan ? SDL_BUTTON_RIGHT : SDL_BUTTON_LEFT;
        if (e.button.button == button)
        {
            if (DraggingGizmo())
            {
                MoveGizmo(world, camera, pointer);
            }
            else if (extensionSource)
            {
                MovePoint(world.roads[state.selected], ToWorld(pointer.x, pointer.y) + grabOffset);
            }
            Release(world);
        }
        break;
    }
    case SDL_EVENT_MOUSE_WHEEL:
        if (drag == Drag::None)
        {
            const Vector3 before = ToWorld(pointer.x, pointer.y);
            const float amount = e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -e.wheel.y : e.wheel.y;
            scale = std::clamp(scale * std::pow(1.25, amount), 0.02, 50.0);
            center = center + (before - ToWorld(pointer.x, pointer.y));
            mouse = pointer;
        }
        break;
    }
    return EditorEvent::Handled;
}
