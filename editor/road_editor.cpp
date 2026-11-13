#include "road_editor.hpp"
#include "marker.hpp"
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
#include <limits>
#include <numeric>
#include <utility>

namespace
{
    constexpr int panelWidth = 218;
    constexpr float pickRadius = 9.0f;
    constexpr float handleLength = 35.0f;
#ifndef _WIN32
    constexpr SDL_FRect roadNamesBox{12, 55, 194, 22};
    constexpr SDL_FRect markerTextBox{12, 219, 194, 22};
    constexpr SDL_FRect markerColorBox{115, 249, 91, 22};
    constexpr SDL_FRect markerColorSwatch{96, markerColorBox.y + 3, 15, 15};
    constexpr SDL_FRect markerClearButton{12, 178, 128, 21};
    constexpr SDL_FRect propertyMaterialSlot{12, 118, 194, 54};
    float LaneOptionsHeight(const PropertiesInspector& properties)
    {
        if (properties.OneLane()) return 54.0f;
        if (properties.ThreeLanes()) return 104.0f;
        return 0.0f;
    }

    SDL_FRect MaterialSlotRect(const PropertiesInspector& properties, bool strip = false)
    {
        const float y = (properties.AllDivided() ? 514.0f : 406.0f) +
                        LaneOptionsHeight(properties);
        return {12, y + (strip ? 58 : 0), 194, 54};
    }

    float RoadStyleBottom(const PropertiesInspector& properties)
    {
        const auto slot = MaterialSlotRect(properties, properties.HasTransitionSlot());
        return slot.y + slot.h + 8;
    }
#endif
    constexpr auto ink = EditorTheme::text, muted = EditorTheme::secondary;

    bool Near(SDL_FPoint a, SDL_FPoint b)
    {
        return std::hypot(a.x - b.x, a.y - b.y) <= pickRadius;
    }

    bool AnchorOccupied(const WorldDocument& world, IntersectionAnchor anchor)
    {
        return std::ranges::any_of(world.ConnectedEnds(anchor.intersectionId),
            [anchor](const auto& end) { return end.anchor == anchor; });
    }

    int AttachedPoint(const Road& road, IntersectionAnchor anchor)
    {
        if (road.points.front().anchor == anchor) return 0;
        if (road.points.back().anchor == anchor) return static_cast<int>(road.points.size()) - 1;
        return -1;
    }

    RoadPoint AttachedEndpoint(const Intersection& junction, IntersectionAnchor anchor, bool first)
    {
        const Vector3 tangent = Intersection::Directions(junction.yaw)[anchor.side] * (first ? 1.0 : -1.0);
        return {junction.AnchorPosition(anchor.side), std::atan2(tangent.z, tangent.x), 0, 0, anchor};
    }

    // Project a screen delta from the authored anchor, even when the press was off-center on its raised marker.
    std::optional<Vector3> DragAtElevation(const FreeCamera& camera, SDL_Rect scene, Vector3 anchor, SDL_FPoint delta)
    {
        const Vector3 view = camera.WorldToView(anchor);
        const double focal = ViewportFocalLength(scene);
        if (view.z < worldNearDepth || focal <= 0) return {};
        const double height = anchor.y - camera.GetPosition().y;
        if (std::abs(height) < 0.05 * view.z)
        {
            // Viewed edge-on, the horizontal plane has no usable ray intersection.
            // Map screen right/up to camera right/forward at the anchor's depth.
            const Vector3 right = camera.ViewRay(1, 0) - camera.ViewRay(0, 0);
            const Vector3 forward{-right.z, 0, right.x};
            return anchor + (right * delta.x - forward * delta.y) * (view.z / focal);
        }
        const Vector3 ray = camera.ViewRay(view.x / view.z + delta.x / focal, view.y / view.z - delta.y / focal);
        if (std::abs(ray.y) < 0.01 || height / ray.y < worldNearDepth) return {};
        const Vector3 hit = camera.GetPosition() + ray * (height / ray.y);
        return Vector3{hit.x, anchor.y, hit.z};
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
    frame = window;
    cleanTitle = SDL_GetWindowTitle(window);
    if (!menu.Attach(window))
    {
        return false;
    }
    browser.Refresh();
    gizmo.Load(models);
#ifndef _WIN32
    resizeCursor = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_NS_RESIZE);
    cameraToolbar.LoadIcons(SDL_GetRenderer(window));
    outlinePanel.LoadIcons(SDL_GetRenderer(window));
#endif
    return true;
}

SDL_FPoint RoadEditor::PlanGroupHandle(const ObjectTransform& frame) const
{
    const Vector3 axis = frame.ToMeshTransform().x;
    const double heading = std::atan2(axis.z, axis.x);
    return ToScreen(frame.position + Vector3{std::cos(heading), 0, std::sin(heading)} * (handleLength / scale));
}

std::optional<GizmoPlacement> RoadEditor::SelectedGizmoPlacement(const WorldDocument& world, const FreeCamera& camera) const
{
    if (!state.segments.empty())
    {
        return {};
    }
    const auto target = state.Target();
    if (target.kind == SelectionKind::Anchor) return {};
    if (target.kind == SelectionKind::Point)
    {
        const Road* road = Selected(world);
        if (!road || state.point >= static_cast<int>(road->points.size()))
        {
            return {};
        }
        const RoadPoint& point = road->points[state.point];
        if (point.ramp) return {};
        auto placement = RoadOverlay::PointPlacement(point.position, camera, scene);
        if (placement && gizmo.settings.space == GizmoSpace::Local)
        {
            placement->localAxes = road->PointAxes(state.point);
        }
        return placement;
    }
    const auto fields = Inspect(world, target);
    if (fields.target.kind == SelectionKind::None || fields.roadBound) return {};
    auto placement = GizmoPlacement::At(fields.value.position, camera, scene);
    if (placement) placement->yawOnly = target.YawOnly();
    if (placement && gizmo.settings.space == GizmoSpace::Local)
    {
        const auto frame = ObjectTransform{{}, fields.value.rotation}.ToMeshTransform();
        placement->localAxes = {frame.x, frame.y, frame.z};
    }
    return placement;
}

std::optional<SDL_FPoint> RoadEditor::GizmoPointer() const
{
    if (!frame || SDL_GetWindowRelativeMouseMode(frame)) return {};
#ifdef _WIN32
    return panels.ViewportPointer(scene);
#else
    if (SDL_GetMouseFocus() != frame || !(SDL_GetWindowFlags(frame) & SDL_WINDOW_INPUT_FOCUS) ||
        menu.IsOpen() || browser.MaterialPopupOpen() || !draggedAsset.empty()) return {};
    SDL_FPoint cursor, point;
    SDL_GetMouseState(&cursor.x, &cursor.y);
    if (!SDL_RenderCoordinatesFromWindow(SDL_GetRenderer(frame), cursor.x, cursor.y, &point.x, &point.y) ||
        point.x < 0 || point.y < 0 || point.x >= scene.w || point.y >= scene.h) return {};
    return point;
#endif
}

bool RoadEditor::GizmoInForeground() const
{
    // An object can enclose its pivot, and a group pivot can land inside terrain.
    return state.meshSelection.kind == SelectionKind::Object || state.RoadGroupSelected();
}

void RoadEditor::UpdateGizmoHover(const WorldDocument& world, const FreeCamera& camera, const GizmoPlacement& placement)
{
    const auto pointer = GizmoPointer();
    if (!pointer)
    {
        hoveredAxis.reset();
        lastHoverPoint = {-1, -1};
        return;
    }
    if (pointer->x == lastHoverPoint.x && pointer->y == lastHoverPoint.y) return;
    lastHoverPoint = *pointer;
    ViewportPicker handles(camera, scene, *pointer);
    hoveredAxis = gizmo.PickAxis(handles, placement, gizmoDrag ? &*gizmoDrag : nullptr);
    if (!hoveredAxis) return;

    const auto modifiers = SDL_GetModState();
    ViewportPicker surfaces(camera, scene, *pointer);
    const RoadPick hit = overlay.Pick(surfaces, world, Selected(world), camera, scene,
                                      (modifiers & (SDL_KMOD_SHIFT | SDL_KMOD_CTRL)) != 0);
    if ((!gizmoDrag && !GizmoTakesClick(hit, modifiers)) || (!GizmoInForeground() && !handles.CloserThan(surfaces)))
        hoveredAxis.reset();
}

void RoadEditor::SubmitGizmos(WorldRenderer& renderer, const WorldDocument& world, const FreeCamera& camera)
{
    if (pendingRamp)
    {
        overlay.DrawDirection(renderer, pendingRamp->preview, camera, scene);
        return;
    }
    overlay.DrawIntersections(renderer, world, state, camera, scene);
    if (const Road* road = Selected(world))
    {
        overlay.Draw(renderer, *road, state, camera, scene);
    }
    if (const auto placement = SelectedGizmoPlacement(world, camera))
    {
        UpdateGizmoHover(world, camera, *placement);
        if (GizmoInForeground()) renderer.BeginForeground();
        gizmo.Draw(renderer, *placement, gizmoDrag ? &*gizmoDrag : nullptr, hoveredAxis);
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
std::vector<RoadEditor::Button> RoadEditor::Buttons(const PropertiesInspector& properties) const
{
    if (properties.page == PropertiesPage::Intersection)
        return {{Action::SmallerRadius, {12, 73, 32, 28}, "-", false, properties.radius > 0},
                {Action::LargerRadius, {174, 73, 32, 28}, "+"}};
    if (properties.page == PropertiesPage::Anchor)
        return {{Action::ToggleStopLine, {12, 73, 194, 28}, "Stop line", properties.stopLine}};
    if (properties.page == PropertiesPage::Object || properties.page == PropertiesPage::Marker)
        return {{Action::ToggleInvincible, {12, 73, 194, 28}, "Invincible", properties.invincible}};
    if (properties.page != PropertiesPage::Road) return {};
    const auto& style = properties.style;
    const float offset = LaneOptionsHeight(properties);

    std::vector<Button> buttons{
        {Action::TogglePaintMarkings, {12, 84, 194, 28}, "Paint markings", properties.paintMarkings,
         properties.roadNames.has_value(), properties.Mixed(RoadProperty::Markings)},
        {Action::Straight, {12, 148, 94, 28}, "Straight", !properties.Mixed(RoadProperty::Alignment) && !style.curved},
        {Action::Curve, {112, 148, 94, 28}, "Bezier", !properties.Mixed(RoadProperty::Alignment) && style.curved},
        {Action::Fewer, {12, 210, 32, 28}, "-", false, properties.CanStepLanes(false)},
        {Action::More, {174, 210, 32, 28}, "+", false, properties.CanStepLanes(true)},
        {Action::Left, {12, 274 + offset, 94, 28}, "Left", style.leftShoulder, true, properties.Mixed(RoadProperty::Left)},
        {Action::Right, {112, 274 + offset, 94, 28}, "Right", style.rightShoulder, true, properties.Mixed(RoadProperty::Right)},
        {Action::NoEdge, {12, 328 + offset, 46, 28}, "None", !properties.Mixed(RoadProperty::Edge) && style.edge == RoadEdge::None},
        {Action::Transition, {62, 328 + offset, 94, 28}, "Transition", !properties.Mixed(RoadProperty::Edge) && style.edge == RoadEdge::Transition},
        {Action::Curb, {160, 328 + offset, 46, 28}, "Curb", !properties.Mixed(RoadProperty::Edge) && style.edge == RoadEdge::Curb},
        {Action::Divide, {12, 368 + offset, 194, 28}, "Divided road", properties.AllDivided(),
         properties.canDivide || properties.anyDivided, properties.Mixed(RoadProperty::Divided)}};

    if (properties.OneLane())
    {
        buttons.push_back({Action::LessExtraWidth, {12, 264, 32, 28}, "-", false, properties.maxExtraWidth > 0});
        buttons.push_back({Action::MoreExtraWidth, {174, 264, 32, 28}, "+"});
    }
    if (properties.AllDivided())
    {
        buttons.push_back({Action::NarrowerGap, {12, 422 + offset, 32, 28}, "-", false,
                           properties.maxGap > RoadStyle::minSeparation});
        buttons.push_back({Action::WiderGap, {174, 422 + offset, 32, 28}, "+"});
        const bool hasMedian = properties.HasMedian();
        buttons.push_back({Action::ConcreteMedian, {12, 476 + offset, 94, 28}, "Concrete",
                           hasMedian && !properties.Mixed(RoadProperty::Median) && style.median == RoadMedian::Concrete, hasMedian});
        buttons.push_back({Action::GrassMedian, {112, 476 + offset, 94, 28}, "Grass",
                           hasMedian && !properties.Mixed(RoadProperty::Median) && style.median == RoadMedian::Grass, hasMedian});
    }
    const float styleBottom = RoadStyleBottom(properties);

    if (planView)
    {
        buttons.push_back({Action::Finish, {12, styleBottom + 30, 194, 28}, "Finish road  [Enter]", false,
                           state.drawing && state.draft->points.size() >= 2});
        buttons.push_back({Action::Cancel, {12, styleBottom + 64, 194, 28}, "Cancel draft [Esc]", false, state.drawing});
    }

    if (properties.ThreeLanes())
    {
        buttons.push_back({Action::TwoNorthbound, {12, 268, 194, 26}, "2 north / 1 south",
                           !properties.Mixed(RoadProperty::Layout) && style.threeLaneLayout == ThreeLaneLayout::TwoNorthbound});
        buttons.push_back({Action::TwoSouthbound, {12, 298, 194, 26}, "1 north / 2 south",
                           !properties.Mixed(RoadProperty::Layout) && style.threeLaneLayout == ThreeLaneLayout::TwoSouthbound});
        buttons.push_back({Action::CenterTurn, {12, 328, 194, 26}, "Center turn lane",
                           !properties.Mixed(RoadProperty::Layout) && style.threeLaneLayout == ThreeLaneLayout::CenterTurn});
    }

    return buttons;
}

#endif

bool RoadEditor::UpdateWindowTitle()
{
#ifdef _WIN32
    const bool pendingText = panels.HasPendingEdit();
#else
    const bool pendingText = transformPanel.HasPendingEdit();
#endif
    const bool dirty = history.WorldRevision() != savedRevision || dragChanged || pendingText ||
                       (state.draft && !state.draft->points.empty());
    if (dirty == titleShowsChanges || !frame) return true;
    if (!SDL_SetWindowTitle(frame, dirty ? (cleanTitle + " *").c_str() : cleanTitle.c_str())) return false;
    titleShowsChanges = dirty;
    return true;
}

bool RoadEditor::Save(WorldDocument& world)
{
    UpdateWindowTitle();
    world.UpdateIntersections();
    saveFailed = !world.Save();
    if (saveFailed)
    {
        status = "Save failed - use File > Save to retry";
        const std::string error = SDL_GetError();
        SDL_Log("%s", error.c_str());
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save world", error.c_str(), frame);
        return false;
    }
    savedRevision = history.WorldRevision();
    status.clear();
    return true;
}

void RoadEditor::Commit(WorldDocument& world, int roadIndex, EditSnapshot before, const Road* after)
{
    if (history.Record(roadIndex, std::move(before), EditSnapshot(state, after)) && roadIndex >= 0)
    {
        Save(world);
    }
}

std::optional<RampPlan> RoadEditor::PrepareRamp(EditorCommand command, const WorldDocument& world) const
{
    if (!roadMenuTarget || !roadMenuTarget->rampSite) return {};
    const auto& target = *roadMenuTarget;
    if (target.index < 0 || target.index >= static_cast<int>(world.roads.size())) return {};
    const auto& host = world.roads[target.index];
    if (host.revision != target.revision) return {};
    auto site = *target.rampSite;
    RoadStyle style;
    style.oneWay = true;
    style.SetLaneCount(1);
    bool exit = command == EditorCommand::CreateExitRamp;
    std::optional<RampLink> replacing;
    double length = RampLink{}.length;
    if (command == EditorCommand::ConnectRamp || command == EditorCommand::SetRampTaper)
    {
        const Road* ramp = Selected(world);
        if (!ramp || ramp == &host || ramp->points.size() < 2 || ramp->style.IsDivided() ||
            (!ramp->style.oneWay && ramp->style.GetLaneCount() != 1) || !state.additionalPoints.empty() ||
            (state.point != 0 && state.point != static_cast<int>(ramp->points.size()) - 1)) return {};
        const auto& point = ramp->points[state.point];
        if (point.anchor) return {};
        exit = state.point == 0;
        style = ramp->style;
        style.oneWay = true;
        replacing = point.ramp;
        if (replacing) length = replacing->length;
        if (command == EditorCommand::SetRampTaper)
        {
            if (!replacing || site.side != replacing->side) return {};
            const auto amount = RampTaperLength(host, *replacing, exit, target.surfaceHit);
            if (!amount) return {};
            const auto knot = std::ranges::find(host.points, replacing->station, &RoadPoint::rampStation);
            const int i = static_cast<int>(knot - host.points.begin());
            site = i + 1 < static_cast<int>(host.points.size()) ? RampSite{i, site.side, 0} : RampSite{i - 1, site.side, 1};
            length = *amount;
        }
    }
    auto plan = PlanRamp(host, site, world.NextRampStationId(), style, exit, length, replacing);
    if (plan && command == EditorCommand::ConnectRamp)
    {
        const Road& ramp = world.roads[state.selected];
        const auto& neighbor = ramp.points[exit ? 1 : ramp.points.size() - 2];
        if (!ValidRoadSpan(ramp, plan->seam.center, neighbor.position)) return {};
    }
    return plan;
}

void RoadEditor::CreateRamp(EditorCommand command, WorldDocument& world)
{
    auto plan = PrepareRamp(command, world);
    if (!plan) return;
    const bool create = command == EditorCommand::CreateExitRamp || command == EditorCommand::CreateEntranceRamp;
    const int hostIndex = roadMenuTarget->index;
    Road& host = world.roads[hostIndex];
    if (create)
    {
        const bool exit = command == EditorCommand::CreateExitRamp;
        auto preview = NewRamp(host.style, *plan, exit);
        pendingRamp.emplace(RampPlacement{hostIndex, host.id, host.revision, std::move(*plan), std::move(preview), exit});
        return;
    }
    const int index = state.selected, hostId = host.id;
    auto& ramp = world.roads[index];
    EditSnapshot before(state, &ramp);
    before.connectedRoadPoints.emplace(hostId, host.points);
    host.points = std::move(plan->hostPoints);
    host.Rebuild();
    ramp.style.oneWay = true;
    auto& end = ramp.points[state.point];
    end.ramp = plan->link;
    AlignRampEnd(end, plan->seam);
    ramp.Rebuild();
    world.RefreshRampConnections(true);
    EditSnapshot after(state, &world.roads[index]);
    after.connectedRoadPoints.emplace(hostId, world.roads[roadMenuTarget->index].points);
    outliner.Invalidate();
    if (history.Record(index, std::move(before), std::move(after))) Save(world);
}

void RoadEditor::AimRamp(const FreeCamera& camera, SDL_FPoint pointer)
{
    auto& placement = *pendingRamp;
    const auto& seam = placement.plan.seam;
    const auto normal = Math::Normalize(Math::Cross(seam.forward, seam.frame.left));
    const auto ray = ViewportRay(camera, scene, pointer);
    const double denominator = Math::Dot(ray, normal);
    if (std::abs(denominator) < 1e-8) return;
    const double depth = Math::Dot(seam.center - camera.GetPosition(), normal) / denominator;
    if (depth < worldNearDepth) return;
    const auto end = camera.GetPosition() + ray * depth;
    const auto offset = end - seam.center;
    // Start the branch on its own side with a nonzero approach in the travel direction.
    if (Math::Dot(offset, seam.forward) * (placement.exit ? 1 : -1) < Road::minPointSpacing ||
        Math::Dot(offset, seam.frame.left) > 0) return;
    auto& free = placement.exit ? placement.preview.points.back() : placement.preview.points.front();
    if (free.position == end) return;
    free.position = end;
    placement.preview.Rebuild();
}

void RoadEditor::FinishRamp(WorldDocument& world)
{
    auto placement = std::move(*pendingRamp);
    pendingRamp.reset();
    const int i = placement.hostIndex;
    if (i >= static_cast<int>(world.roads.size()) || world.roads[i].id != placement.hostId ||
        world.roads[i].revision != placement.revision) return;
    if (!(placement.preview.id = world.AllocateRoadId())) return;
    EditSnapshot before(state);
    before.connectedRoadPoints.emplace(placement.hostId, world.roads[i].points);
    world.roads[i].points = std::move(placement.plan.hostPoints);
    world.roads[i].Rebuild();
    world.roads.push_back(std::move(placement.preview));
    const int index = static_cast<int>(world.roads.size()) - 1;
    state.drawing = false;
    state.SelectPoint(index, placement.exit ? 1 : 0);
    const int folder = world.hierarchy.Parent({SceneKind::Road, i});
    if (folder) world.hierarchy.Move({{SceneKind::Road, index}}, folder);
    world.RefreshRampConnections(true);
    EditSnapshot after(state, &world.roads.back());
    after.roadFolder = folder;
    after.connectedRoadPoints.emplace(placement.hostId, world.roads[i].points);
    outliner.Invalidate();
    if (history.Record(index, std::move(before), std::move(after))) Save(world);
}

void RoadEditor::RunCommand(EditorCommand command, WorldDocument& world, FreeCamera& camera)
{
    if (command == EditorCommand::ToggleWireframe)
    {
        menu.ToggleWireframe();
        return;
    }
    const bool undoPreview = command == EditorCommand::Undo && (pendingRamp || drag == Drag::AnchorRoad);
    Release(world);
    if (undoPreview) return;
    if (command == EditorCommand::CreateExitRamp || command == EditorCommand::CreateEntranceRamp ||
        command == EditorCommand::ConnectRamp || command == EditorCommand::SetRampTaper)
    {
        CreateRamp(command, world);
        return;
    }
    if (command == EditorCommand::CreateRoad)
    {
        constexpr double initialLength = 10.0; // meters along world X
        EditSnapshot before(state);
        Road road;
        if (!(road.id = world.AllocateRoadId())) { status = SDL_GetError(); return; }
        road.style = Style(world);
        road.style.oneWay = false;
        road.style.material = RoadStyle::defaultMaterial;
        road.points = {{creationPoint, {}}, {creationPoint + Vector3{initialLength, 0, 0}, {}}};
        road.Rebuild();
        world.roads.push_back(std::move(road));
        state.drawing = false;
        state.SelectPoint(static_cast<int>(world.roads.size()) - 1, 1);
        center = creationPoint;
        Commit(world, state.selected, std::move(before), &world.roads.back());
        return;
    }
    if (command == EditorCommand::CreateIntersection)
    {
        if (roadMenuTarget)
        {
            auto target = std::move(*roadMenuTarget);
            roadMenuTarget.reset();
            if (!target.insertion || target.index >= static_cast<int>(world.roads.size()) ||
                world.roads[target.index].revision != target.revision) return;
            if (history.ReplaceRoad(world, state, target.index, std::move(target.insertion->approaches),
                                    &target.insertion->junction))
            {
                outliner.Invalidate();
                Save(world);
            }
            else status = SDL_GetError();
            return;
        }
        EditSnapshot before(state);
        const int id = world.CreateIntersection(creationPoint);
        if (!id) { status = SDL_GetError(); return; }
        before.junction = EditSnapshot::Junction{id, {}};
        state.SelectRows({{SceneKind::Intersection, id}});
        EditSnapshot after(state);
        after.junction = EditSnapshot::Junction{id, world.intersections.at(id)};
        history.Record(-1, std::move(before), std::move(after));
        Save(world);
        return;
    }
    if (command == EditorCommand::CreateMarker)
    {
        PlaceObject(world, ObjectType::Marker, Marker::modelPath, creationPoint);
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
            Activate(Action::Finish, world);
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
        status.clear();
    }
}

void RoadEditor::OpenViewportMenu(WorldDocument& world, FreeCamera& camera)
{
    roadMenuTarget.reset();
    auto picker = rightClick->picker;
    const RoadPick hit = overlay.Pick(picker, world, Selected(world), camera, scene, false, &creationPoint);
    bool canCreate = !(hit.kind == RoadPickKind::Intersection && hit.side >= 0 && AnchorOccupied(world, {hit.index, hit.side}));
    if (hit.kind == RoadPickKind::Road || hit.kind == RoadPickKind::Segment || hit.kind == RoadPickKind::Point)
    {
        const int index = hit.kind == RoadPickKind::Road ? hit.index : state.selected;
        const Road& road = world.roads[index];
        const auto position = hit.kind == RoadPickKind::Segment ? RoadOverlay::SplinePosition(picker, scene) :
                                                                 picker.HitPosition();
        roadMenuTarget = RoadMenuTarget{index, road.revision, PlanRoadIntersection(road, hit, position)};
        canCreate = roadMenuTarget->insertion.has_value();
        if (road.style.IsDivided())
        {
            // Pick the visible carriageway itself. Overlay boxes and spline ribbons can protrude beyond it.
            auto pavement = rightClick->picker;
            if (pavement.TestRoad(road))
            {
                roadMenuTarget->surfaceHit = pavement.HitPosition();
                roadMenuTarget->rampSite = PickRampSite(road, roadMenuTarget->surfaceHit);
            }
        }
    }

    const RampMenuOptions ramps{PrepareRamp(EditorCommand::CreateExitRamp, world).has_value(),
                                PrepareRamp(EditorCommand::CreateEntranceRamp, world).has_value(),
                                PrepareRamp(EditorCommand::ConnectRamp, world).has_value(),
                                PrepareRamp(EditorCommand::SetRampTaper, world).has_value()};
    if (const auto command = menu.OpenViewport(rightClick->windowPoint, picker.mouse, scene, canCreate, ramps))
    {
        RunCommand(*command, world, camera);
    }
#ifdef _WIN32
    roadMenuTarget.reset(); // The native popup closes before OpenViewport returns.
#endif
}

void RoadEditor::AssignMaterial(WorldDocument& world, const std::string& path, const FreeCamera& camera,
                                std::optional<SDL_FPoint> viewportPoint, bool strip)
{
    RoadPick target{RoadPickKind::Road, EditingDraft() ? -1 : state.selected};
    if (!viewportPoint && !EditingDraft())
    {
        if (state.meshSelection.kind == SelectionKind::Object) target = {RoadPickKind::Object, state.meshSelection.index};
        if (state.meshSelection.kind == SelectionKind::Terrain) target = {RoadPickKind::Terrain, state.meshSelection.index};
        if (state.meshSelection.kind == SelectionKind::Intersection) target = {RoadPickKind::Intersection, state.meshSelection.index};
    }
    if (viewportPoint)
    {
        if (planView || drag != Drag::None) return;
        ViewportPicker picker(camera, scene, *viewportPoint);
        target = overlay.Pick(picker, world, nullptr, camera, scene);
    }
    Road* road = nullptr;
    SceneKey key;
    switch (target.kind)
    {
    case RoadPickKind::Road:
        if (target.index >= 0) road = &world.roads.at(target.index);
        else if (!viewportPoint && EditingDraft()) road = &*state.draft;
        break;
    case RoadPickKind::Terrain: key = {SceneKind::Terrain, target.index}; break;
    case RoadPickKind::Intersection: key = {SceneKind::Intersection, target.index}; break;
    case RoadPickKind::Object: key = {SceneKind::Object, target.index}; break;
    default: return;
    }
    const bool group = !EditingDraft() && state.MultipleRoadsSelected() && target.kind == RoadPickKind::Road &&
                       (!viewportPoint || state.outlineSelection.contains({SceneKind::Road, target.index}));
    if (strip && !InspectProperties(world, state, EditingDraft()).HasTransitionSlot()) return;
    const auto materialField = strip ? &RoadStyle::transitionMaterial : &RoadStyle::material;
    auto* current = road ? &(road->style.*materialField) : FindPlacedMaterial(world, key);
    if ((!group && (!current || *current == path)) || (path.empty() && key.kind != SceneKind::Object)) return;
    Material definition;
    if (!path.empty() && (!Material::ValidAssetPath(path) || !definition.Load(browser.Textures().Root() + path)))
    {
        status = SDL_GetError();
        return;
    }
    if (strip && definition.type != MaterialType::Transition)
    {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION, "Transition strip material",
                                 "Choose a material with its type set to Transition.", frame);
        return;
    }
    if (group)
    {
        EditRoadGroup(world, [&](RoadProperties& p) { p.style.*materialField = path; });
        return;
    }
    if (road)
    {
        const Road* placed = target.index >= 0 ? road : nullptr;
        EditSnapshot before(state, placed);
        *current = path;
        if (strip) road->Rebuild();
        Commit(world, target.index, std::move(before), placed);
        return;
    }
    EditSnapshot before(state), after(state);
    before.placedMaterial = EditSnapshot::PlacedMaterial{key, *current};
    after.placedMaterial = EditSnapshot::PlacedMaterial{key, path};
    *current = path;
    history.Record(-1, std::move(before), std::move(after));
    Save(world);
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
    for (const auto& [id, junction] : world.IntersectionSurfaces()) picker.TestRoad(junction.geometry);
    for (const auto& chunk : world.Terrain()) picker.TestMesh(chunk->Instance());

    // Respect the model's authored pivot, including meshes centered above or below it.
    double baseY = mesh->vertices[mesh->indices.front()].position.y;
    for (auto i : mesh->indices) baseY = std::min(baseY, mesh->vertices[i].position.y);
    PlaceObject(world, ObjectType::Prop, path, picker.PlacementPosition(baseY));
}

void RoadEditor::PlaceObject(WorldDocument& world, ObjectType type, const std::string& path, Vector3 position)
{
    const int index = static_cast<int>(world.Objects().size());
    EditSnapshot before(state);
    if (!world.PlaceObject(type, path, ObjectTransform{position}))
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
        state.draft->style.oneWay = false;
        state.draft->style.material = RoadStyle::defaultMaterial;
    }

    state.drawing = true;
    state.SelectPoint(-1, -1);
    status.clear();
    if (before)
    {
        Commit(world, -1, std::move(*before));
    }
}

void RoadEditor::Activate(Action action, WorldDocument& world)
{
    switch (action)
    {
    case Action::ToggleStopLine:
    case Action::SmallerRadius:
    case Action::LargerRadius:
    {
        const auto target = state.Target();
        const bool editAnchor = action == Action::ToggleStopLine;
        if (target.kind != (editAnchor ? SelectionKind::Anchor : SelectionKind::Intersection)) return;
        auto& item = world.intersections.at(target.index);
        EditSnapshot before(state), after(state);
        before.junction = EditSnapshot::Junction{target.index, item};
        if (editAnchor) item.stopLines.flip(target.point);
        else item.cornerRadius = std::max(0.0, item.cornerRadius + (action == Action::LargerRadius ? 1.0 : -1.0));
        after.junction = EditSnapshot::Junction{target.index, item};
        if (history.Record(-1, std::move(before), std::move(after))) Save(world);
        return;
    }
    case Action::ToggleInvincible:
    {
        const auto target = state.Target();
        if (target.kind != SelectionKind::Object) return;
        auto& object = *world.Objects().at(target.index);
        EditSnapshot before(state), after(state);
        before.objectInvincible = object.invincible;
        object.invincible = !object.invincible;
        after.objectInvincible = object.invincible;
        history.Record(-1, std::move(before), std::move(after));
        Save(world);
        return;
    }
    case Action::TogglePaintMarkings:
    {
        if (!EditingDraft() && state.MultipleRoadsSelected())
        {
            const auto p = InspectProperties(world, state, false);
            const bool value = p.Mixed(RoadProperty::Markings) || !p.paintMarkings;
            EditRoadGroup(world, [value](RoadProperties& road) { road.paintMarkings = value; });
            return;
        }
        const int index = EditingDraft() ? -1 : state.selected;
        Road* placed = index >= 0 ? &world.roads.at(index) : nullptr;
        Road* road = EditingDraft() ? &*state.draft : placed;
        if (!road) return;
        EditSnapshot before(state, placed);
        road->paintMarkings = !road->paintMarkings;
        road->Rebuild();
        Commit(world, index, std::move(before), placed);
        return;
    }
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
        if (!(state.draft->id = world.AllocateRoadId())) { status = SDL_GetError(); return; }
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

void RoadEditor::EditRoadGroup(WorldDocument& world, const std::function<void(RoadProperties&)>& edit)
{
    EditSnapshot before(state), after(state);
    for (const auto key : state.SelectedRoadRows())
    {
        const RoadProperties original(world.roads.at(key.index));
        auto updated = original;
        edit(updated);
        if (updated == original) continue;
        before.roadProperties.emplace(key.index, original);
        after.roadProperties.emplace(key.index, std::move(updated));
    }
    if (after.roadProperties.empty()) return;
    for (const auto& [index, properties] : after.roadProperties) properties.Apply(world.roads.at(index));
    outliner.Invalidate();
    history.Record(-1, std::move(before), std::move(after));
    Save(world);
}


void RoadEditor::ChangeStyle(Action action, WorldDocument& world)
{
    const auto p = InspectProperties(world, state, EditingDraft());
    if (p.page != PropertiesPage::Road) return;
    if (!EditingDraft() && state.MultipleRoadsSelected())
    {
        EditRoadGroup(world, [&](RoadProperties& road) { p.ApplyRoadStyle(road.style, action); });
        return;
    }
    RoadStyle& current = Style(world);
    RoadStyle style = current;
    p.ApplyRoadStyle(style, action);
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

    if (!shift && state.RoadGroupSelected())
    {
        const auto frame = Inspect(world, state.Target()).value;
        const bool rotate = Near(cursor, PlanGroupHandle(frame));
        if (rotate || (!subtract && Near(cursor, ToScreen(frame.position))))
        {
            BeginGroupDrag(world, rotate ? Drag::Rotate : Drag::Move);
            planDragStart = pos;
            const Vector3 radial = pos - groupPivot;
            planRotation = DragAngle(std::atan2(radial.z, radial.x));
            snapRotation = subtract;
            return;
        }
    }
    if (!shift && state.selected >= 0 && state.point >= 0 && Near(cursor, Handle(world.roads[state.selected], state.point)))
    {
        Road& road = world.roads[state.selected];
        // At this zoom the rotation handle may overlap another selected point.
        if (std::ranges::none_of(state.additionalPoints, [&](int i) { return Near(cursor, ToScreen(road.points[i].position)); }))
        {
            const Vector3 radial = pos - road.points[state.point].position;
            planRotation = DragAngle(std::atan2(radial.z, radial.x));
            snapRotation = subtract;
            BeginRoadDrag(road, Drag::Rotate, false);
            return;
        }
    }

    // Give the nearest marker the click when zoom brings several within the hit radius.
    std::optional<IntersectionAnchor> closest;
    float best = pickRadius;
    for (const auto& [id, item] : world.intersections)
    {
        {
            const auto p = ToScreen(item.position);
            const float distance = std::hypot(p.x - x, p.y - y);
            if (distance <= best)
            {
                best = distance;
                closest.reset();
            }
        }
        for (int side = 0; side < IntersectionAnchor::sideCount; ++side)
        {
            const auto p = ToScreen(item.AnchorPosition(side));
            const float distance = std::hypot(p.x - x, p.y - y);
            if (distance < best)
            {
                closest = IntersectionAnchor{id, side};
                best = distance;
            }
        }
    }
    if (closest && (shift || subtract))
    {
        if (shift && BeginAnchorRoad(world, *closest, cursor, subtract)) return;
        if (state.point < 0 && !state.anchorSelection)
        {
            state.PickElement({SceneKind::Intersection, closest->intersectionId}, shift, subtract);
            return;
        }
        const Road* road = Selected(world);
        if (!road || AttachedPoint(*road, *closest) < 0)
        {
            SelectAnchor(world, *closest, subtract);
            return;
        }
        // An occupied anchor on this road is also a selectable endpoint.
    }

    auto grabPoint = [&](int roadIndex, int point)
    {
        const Road& road = world.roads[roadIndex];
        const bool alreadySelected = roadIndex == state.selected && state.PointSelected(point);
        state.PickPoint(roadIndex, point, shift || alreadySelected, subtract);
        if (subtract)
        {
            return;
        }
        planDragStart = pos;
        selectOnlyOnClick = !shift && alreadySelected && !state.additionalPoints.empty();
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
    if (closest && !shift && !subtract)
    {
        state.InspectAnchor(*closest);
        status.clear();
        return;
    }
    if (shift || subtract)
    {
        if (const auto key = PlanElementAt(world, pos))
        {
            if (key->kind != SceneKind::Road || key->index != state.selected || state.point < 0)
                state.PickElement(*key, shift, subtract);
        }
        return;
    }
    for (int i = static_cast<int>(world.roads.size()) - 1; i >= 0; --i)
    {
        if (pickPoint(i))
        {
            return;
        }
    }
    if (const auto key = PlanElementAt(world, pos)) state.SelectRows({*key});
    else state.SelectPoint(-1, -1);
}

std::optional<SceneKey> RoadEditor::PlanElementAt(const WorldDocument& world, Vector3 pos) const
{
    for (const auto& [id, item] : world.intersections)
    {
        if (Near(ToScreen(pos), ToScreen(item.position)))
        {
            return SceneKey{SceneKind::Intersection, id};
        }
    }
    const auto hitSurface = [&](const auto& surface)
    {
        return std::any_of(surface.begin(), surface.end(), [&](const auto& t)
        {
            return HitTriangle(pos, {t[0].position, t[1].position, t[2].position});
        });
    };
    const auto hitRoad = [&](const RoadGeometry& geometry)
    {
        return hitSurface(geometry.pavementTriangles) || hitSurface(geometry.edgeTriangles) ||
               std::ranges::any_of(geometry.transitionStrips, [&](const auto& entry) { return hitSurface(entry.second); });
    };
    for (auto it = world.IntersectionSurfaces().rbegin(); it != world.IntersectionSurfaces().rend(); ++it)
    {
        const auto& geometry = it->second.geometry;
        if (hitRoad(geometry))
        {
            return SceneKey{SceneKind::Intersection, it->first};
        }
    }
    // Later roads are drawn last, so pick them first wherever pavement overlaps.
    for (int i = static_cast<int>(world.roads.size()) - 1; i >= 0; --i)
    {
        const Road& road = world.roads[i];
        if (hitRoad(road))
        {
            return SceneKey{SceneKind::Road, i};
        }
    }
    return {};
}

void RoadEditor::BeginRoadDrag(const Road& road, Drag kind, bool shift)
{
    if (state.point >= 0 && road.points[state.point].ramp) return;
    dragStart.emplace(state, &road);
    drag = kind;
    const bool translate = kind == Drag::Move || (kind == Drag::Gizmo && gizmo.mode == GizmoMode::Translate);
    const bool endpoint = state.point == 0 || state.point + 1 == static_cast<int>(road.points.size());
    if (state.point >= 0 && state.additionalPoints.empty() && translate && endpoint && shift)
    {
        extensionSource = state.point;
    }
    originalPoints = road.points;
    originalRotation = road.rotation;
}

void RoadEditor::BeginGroupDrag(const WorldDocument& world, Drag kind)
{
    dragStart.emplace(state);
    dragStart->roadGroup = RoadGroupPose::Capture(world, state.SelectedRows());
    groupPivot = Inspect(world, state.Target()).value.position;
    groupOffset = {};
    drag = kind;
}

void RoadEditor::TransformGroup(WorldDocument& world, Vector3 offset, Vector3 axis, double angle)
{
    if (offset == groupOffset && angle == appliedAngle) return;
    if (dragStart->roadGroup->Transform(world, state.SelectedRows(), groupPivot, offset, axis, angle))
    {
        groupOffset = offset;
        appliedAngle = angle;
        dragChanged = offset != Vector3{} || angle != 0;
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
        ? std::min(Road::minPointSpacing, gizmo.settings.StepMeters() * 0.5) : Road::minPointSpacing;
    if (!extensionSource)
    {
        if (TranslateRoadPoints(road, originalPoints, state.point, state.additionalPoints, position, minSpacing))
            dragChanged = position != originalPoints[state.point].position;
        return;
    }
    const bool inserted = road.points.size() != originalPoints.size();
    if (Math::Length(position - originalPoints[*extensionSource].position) < minSpacing)
    {
        // Returning to the starting point withdraws the preview.
        if (inserted)
        {
            road.points = originalPoints;
            state.anchorSelection = dragStart->anchorSelection;
            state.point = *extensionSource;
            road.Rebuild();
            dragChanged = false;
        }
        return;
    }
    if (!inserted)
    {
        state.anchorSelection.reset();
        state.point = *extensionSource == 0 ? 0 : static_cast<int>(road.points.size());
        road.points.insert(road.points.begin() + state.point, RoadPoint{position, {}});
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
    else if (drag == Drag::Move || drag == Drag::Rotate)
    {
        UpdatePlanDrag(world, mouse);
    }
}

void RoadEditor::UpdatePlanDrag(WorldDocument& world, SDL_FPoint pointer)
{
    mouse = pointer;
    const Vector3 pos = ToWorld(mouse.x, mouse.y);
    if (dragStart->roadGroup)
    {
        if (drag == Drag::Move) TransformGroup(world, pos - planDragStart, {}, 0);
        else
        {
            const Vector3 radial = pos - groupPivot;
            if (std::hypot(radial.x, radial.z) * scale >= 2)
                planRotation.Update(std::atan2(radial.z, radial.x));
            TransformGroup(world, {}, {0,1,0}, -planRotation.Amount(snapRotation));
        }
    }
    else if (drag == Drag::Move)
    {
        const int source = extensionSource.value_or(state.point);
        if (pos != planDragStart) selectOnlyOnClick = false;
        MovePoint(world.roads[state.selected], originalPoints[source].position + (pos - planDragStart));
    }
    else RotatePlanPoint(world.roads[state.selected]);
}

void RoadEditor::RotatePlanPoint(Road& road)
{
    const Vector3 offset = ToWorld(mouse.x, mouse.y) - originalPoints[state.point].position;
    if (std::hypot(offset.x, offset.z) * scale >= 2.0)
    {
        planRotation.Update(std::atan2(offset.z, offset.x));
    }
    const double angle = planRotation.Amount(snapRotation);
    if (angle == appliedAngle) return;

    road.points = originalPoints;
    if (angle != 0)
    {
        const Vector3 tangent = road.Tangent(state.point);
        road.RotatePoint(state.point, std::atan2(tangent.z, tangent.x) + angle);
    }
    road.Rebuild();
    appliedAngle = angle;
    dragChanged = angle != 0;
}

void RoadEditor::Release(WorldDocument& world)
{
    if (dragStart && dragChanged)
    {
        CommitTransform(world, std::move(*dragStart));
    }

    originalPoints.clear();
    selectOnlyOnClick = false;
    anchorRoad.reset();
    pendingRamp.reset();
    extensionSource.reset();
    gizmoDrag.reset();
    appliedAngle = 0;
    snapRotation = false;
    dragStart.reset();
    dragChanged = false;
    drag = Drag::None;
}

bool RoadEditor::DrawTransformDelta(SDL_Renderer* renderer, Vector3 position) const
{
    if (!dragStart) return true;

    char label[96];
    if (drag == Drag::Rotate || (DraggingGizmo() && gizmo.mode == GizmoMode::Rotate))
    {
        const double angle = drag == Drag::Rotate && dragStart->roadGroup ? -appliedAngle : appliedAngle;
        const double degrees = angle / Math::DegreesToRadians(1.0);
        SDL_snprintf(label, sizeof(label), "%+.1f\xC2\xB0", std::abs(degrees) < 0.05 ? 0.0 : degrees);
    }
    else
    {
        Vector3 delta = groupOffset;
        if (!dragStart->roadGroup)
        {
            const int source = extensionSource.value_or(std::max(0, state.point));
            const Vector3 start = dragStart->meshTransform ? dragStart->meshTransform->position : originalPoints[source].position;
            delta = position - start;
        }
        const auto meters = [](double d) { return std::abs(d) < 0.005 ? 0.0 : d; };
        if (DraggingGizmo())
            SDL_snprintf(label, sizeof(label), "%c %+.2f m", gizmoDrag->AxisName(), meters(Math::Dot(delta, gizmoDrag->axis)));
        else
            SDL_snprintf(label, sizeof(label), "X %+.2f  Z %+.2f m", meters(delta.x), meters(delta.z));
    }

    constexpr float glyph = SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE, pad = 5, gap = 14;
    const float width = SDL_utf8strlen(label) * glyph + 2 * pad, height = glyph + 2 * pad;
    const float x = mouse.x + gap + width <= scene.w ? mouse.x + gap : mouse.x - gap - width;
    const float y = mouse.y + gap + height <= scene.h ? mouse.y + gap : mouse.y - gap - height;
    const SDL_FRect box{std::clamp(x, 0.0f, std::max(0.0f, scene.w - width)),
                        std::clamp(y, 0.0f, std::max(0.0f, scene.h - height)), width, height};
    return Fill(renderer, box, EditorTheme::preview) && Text(renderer, box.x + pad, box.y + pad, label, ink);
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

    for (const auto& chunk : world.Terrain())
    {
        const auto& mesh = chunk->Instance().mesh;
        if (!chunk->boundary || !mesh) continue;
        // Give each projected corner its own index. SDL's software renderer can
        // otherwise combine adjacent grid triangles into a rectangle across a diagonal.
        std::vector<SDL_FPoint> corners;
        corners.reserve(mesh->indices.size());
        for (auto index : mesh->indices) corners.push_back(ToScreen(mesh->vertices[index].position));
        std::vector<int> indices(corners.size());
        std::iota(indices.begin(), indices.end(), 0);
        const auto count = static_cast<int>(corners.size());
        if (!SDL_RenderGeometryRaw(renderer, nullptr, &corners.front().x, sizeof(SDL_FPoint), &TerrainChunk::groundColor, 0,
                                   nullptr, 0, count, indices.data(), count, sizeof(int))) return false;
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
        for (const auto& [id, junction] : world.IntersectionSurfaces())
        {
            if (!DrawPlanRoad(renderer, junction.geometry, center, scale, canvas, layer)) return false;
        }
        if (preview && !DrawPlanRoad(renderer, *preview, center, scale, canvas, layer))
        {
            return false;
        }
        if (const Road* road = PreviewRoad(); road && !DrawPlanRoad(renderer, *road, center, scale, canvas, layer))
            return false;
    }

    for (const SceneKey key : state.SelectedRows())
    {
        if (!GeometrySelected(key)) continue;
        if (key.kind == SceneKind::Road && !DrawPlanRoad(renderer, world.roads[key.index], center, scale, canvas, RoadLayer::Outline))
            return false;
        if (key.kind == SceneKind::Intersection)
        {
            const auto it = world.IntersectionSurfaces().find(key.index);
            if (it != world.IntersectionSurfaces().end() && !DrawPlanRoad(renderer, it->second.geometry, center, scale, canvas, RoadLayer::Outline))
                return false;
        }
    }

    const auto drawPoint = [&](SDL_FPoint p, float r, SDL_Color color)
    {
        return Fill(renderer, {p.x-r-1, p.y-r-1, 2*r+2, 2*r+2}, EditorTheme::border) &&
               Fill(renderer, {p.x-r, p.y-r, 2*r, 2*r}, color);
    };
    const bool group = state.RoadGroupSelected();
    for (const auto& [id, item] : world.intersections)
    {
        const auto anchors = item.AnchorPositions();
        const bool selected = GeometrySelected({SceneKind::Intersection, id});
        if (group && selected && !world.IntersectionSurfaces().contains(id))
        {
            std::array<SDL_FPoint, IntersectionAnchor::sideCount + 1> perimeter;
            std::transform(anchors.begin(), anchors.end(), perimeter.begin(), [&](Vector3 p) { return ToScreen(p); });
            perimeter.back() = perimeter.front();
            if (!EditorTheme::SetColor(renderer, EditorColors::ToBytes(EditorColors::orange)) ||
                !SDL_RenderLines(renderer, perimeter.data(), static_cast<int>(perimeter.size()))) return false;
        }
        for (int side = 0; side < IntersectionAnchor::sideCount; ++side)
        {
            const bool active = state.AnchorHighlighted({id, side});
            if (!drawPoint(ToScreen(anchors[side]), 5.0f, active ? amber : blue)) return false;
        }
        const auto p = ToScreen(item.position);
        const auto color = selected ? amber : blue;
        if (!EditorTheme::SetColor(renderer, color)) return false;
        const auto axes = Intersection::Directions(item.yaw);
        for (int arm : {0, 1})
        {
            const float x = static_cast<float>(axes[arm].x), y = static_cast<float>(-axes[arm].z);
            for (int k = -1; k <= 1; ++k)
            {
                if (!SDL_RenderLine(renderer, p.x - 7*x - k*y, p.y - 7*y + k*x,
                                    p.x + 7*x - k*y, p.y + 7*y + k*x)) return false;
            }
        }
    }

    auto drawPoints = [&](const Road& road, bool active)
    {
        for (size_t i = 0; i < road.points.size(); ++i)
        {
            if (!active && road.points[i].anchor) continue; // The anchor already marks this endpoint.
            const SDL_FPoint pos = ToScreen(road.points[i].position);
            if (pos.x < -8 || pos.x > scene.w + 8 || pos.y < -8 || pos.y > scene.h + 8)
            {
                continue;
            }
            const bool selectedPoint = active && state.PointSelected(static_cast<int>(i));
            const float r = active ? 5.0f : 3.0f;
            if (!drawPoint(pos, r, selectedPoint ? amber : (active ? blue : muted)))
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

    if (group)
    {
        const auto frame = Inspect(world, state.Target()).value;
        const auto pivot = ToScreen(frame.position), handle = PlanGroupHandle(frame);
        if (!EditorTheme::SetColor(renderer, amber) || !SDL_RenderLine(renderer, pivot.x, pivot.y, handle.x, handle.y) ||
            !drawPoint(pivot, 6, blue) || !drawPoint(handle, 4, amber)) return false;
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

    if (!SDL_RenderDebugTextFormat(renderer, 12, 12, "Grid: %.0f m    Roads: %zu", spacing, world.roads.size()))
    {
        return false;
    }

    return drag != Drag::Measure || DrawMeasurement(renderer);
}

#ifndef _WIN32
bool RoadEditor::RenderPropertiesPanel(SDL_Renderer* renderer, const WorldDocument& world, const PropertiesInspector& properties)
{
    if (!Fill(renderer, {0, 0, static_cast<float>(roadPanel.w), static_cast<float>(roadPanel.h + roadScroll)}, EditorTheme::panel))
    {
        return false;
    }

    for (const Button& b : Buttons(properties))
    {
        if (b.action == Action::ToggleStopLine || b.action == Action::TogglePaintMarkings ||
            b.action == Action::Left || b.action == Action::Right || b.action == Action::Divide)
        {
            const SDL_FRect box{b.rect.x + 2, b.rect.y + 6, 16, 16};
            if (!Fill(renderer, box, (b.active || b.mixed) ? EditorTheme::selected : EditorTheme::field) ||
                !EditorTheme::SetColor(renderer, muted) || !SDL_RenderRect(renderer, &box) ||
                !Text(renderer, box.x + 23, b.rect.y + 10, b.label, b.enabled ? ink : muted)) return false;
            if (b.mixed)
            {
                if (!EditorTheme::SetColor(renderer, ink) ||
                    !SDL_RenderLine(renderer, box.x + 3, box.y + 8, box.x + 13, box.y + 8)) return false;
            }
            else if (b.active)
            {
                const SDL_FPoint tick[]{{box.x + 3, box.y + 8}, {box.x + 7, box.y + 12}, {box.x + 13, box.y + 4}};
                if (!EditorTheme::SetColor(renderer, ink) || !SDL_RenderLines(renderer, tick, 3)) return false;
            }
            continue;
        }
        const SDL_Color fill = b.active ? EditorTheme::selected : EditorTheme::button;
        if (!Fill(renderer, b.rect, b.enabled ? fill : EditorTheme::header) ||
            !Text(renderer, b.rect.x + 8, b.rect.y + 10, b.label, b.enabled ? ink : muted))
        {
            return false;
        }
    }
    const Road* current = EditingDraft() ? &*state.draft : Selected(world);
    const auto& style = properties.style;
    for (bool strip : {false, true})
    {
        if (strip ? !properties.HasTransitionSlot() : !properties.materialSlot && properties.page != PropertiesPage::Road)
            continue;
        const auto& path = strip ? properties.style.transitionMaterial : properties.material;
        const auto property = strip ? RoadProperty::TransitionMaterial : RoadProperty::Material;
        const SDL_FRect slot = properties.page == PropertiesPage::Road ? MaterialSlotRect(properties, strip) : propertyMaterialSlot;
        const SDL_FRect icon{slot.x + 3, slot.y + 3, 48, 48};
        const char* caption = strip ? "Transition strip" : properties.MaterialCaption();
        if (!Fill(renderer, slot, EditorTheme::list) || !Fill(renderer, icon, EditorTheme::field) ||
            !Text(renderer, slot.x + 55, slot.y + 9, caption, muted)) return false;
        if (!path.empty())
        {
            if (auto* thumbnail = browser.RoadMaterialTexture(path, renderer, strip))
                if (!SDL_RenderTexture(renderer, thumbnail, nullptr, &icon)) return false;
        }
        const auto label = !properties.materialSlot ? "Select a road" : properties.Mixed(property) ? "Mixed" :
                           path.empty() ? (strip ? "Drop material" : "Mesh colors") : path.substr(path.rfind('/') + 1);
        if (!Text(renderer, slot.x + 55, slot.y + 29, label.c_str(), ink)) return false;
    }
    if (properties.page == PropertiesPage::Marker)
    {
        const auto& rgb = properties.textColor;
        if (!Fill(renderer, markerClearButton, properties.material.empty() ? EditorTheme::header : EditorTheme::button) ||
            !Text(renderer, markerClearButton.x + 6, markerClearButton.y + 7, "Clear material", properties.material.empty() ? muted : ink) ||
            !Text(renderer, 12, markerTextBox.y - 15, "Marker text", muted) ||
            !transformPanel.DrawField(renderer, markerTextBox, state.Target(), InspectorEdit::markerText, properties.markerText, true) ||
            !Text(renderer, 12, markerColorBox.y + 6, "Text color", muted) ||
            !Fill(renderer, markerColorSwatch, {rgb[0], rgb[1], rgb[2], 255}) ||
            !transformPanel.DrawField(renderer, markerColorBox, state.Target(), InspectorEdit::markerTextColor, FormatTextColor(rgb), true))
            return false;
    }
    if (!Text(renderer, 14, 18, properties.title.c_str(), ink)) return false;
    if (properties.page == PropertiesPage::Intersection)
        return Text(renderer, 12, 48, "Corner radius (m)", muted) &&
               SDL_RenderDebugTextFormat(renderer, 78, 83, "%.2f", properties.radius);
    if (properties.page != PropertiesPage::Road)
        return Text(renderer, 12, 48, properties.detail.c_str(), muted);
    if (!Text(renderer, 12, roadNamesBox.y - 15, "Road names", muted) ||
        !transformPanel.DrawField(renderer, roadNamesBox, state.Target(), InspectorEdit::roadNames,
                                  properties.Mixed(RoadProperty::Names) ? "Multiple values" : properties.roadNames.value_or(""),
                                  properties.roadNames.has_value())) return false;
    const int laneCount = style.GetLaneCount();
    const float offset = LaneOptionsHeight(properties);
    if (properties.OneLane() &&
        (!Text(renderer, 12, 248, "Extra width (m)", muted) ||
         !SDL_RenderDebugTextFormat(renderer, 80, 274, properties.minExtraWidth == properties.maxExtraWidth ? "%.2f" : "Mixed",
                                    properties.minExtraWidth))) return false;
    if (properties.ThreeLanes() && !Text(renderer, 12, 252, properties.Mixed(RoadProperty::Layout) ? "3-lane layout (mixed)" : "Three-lane layout", muted))
    {
        return false;
    }
    if (properties.minLanes == properties.maxLanes && laneCount > 3 && style.HasCenterTurnLane())
    {
        if (!SDL_RenderDebugTextFormat(renderer, 12, 243, "%d each way + center turn", laneCount / 2))
        {
            return false;
        }
    }
    const float styleBottom = RoadStyleBottom(properties);
    if (properties.AllDivided() &&
        (!Text(renderer, 12, 406 + offset, "Separation (m)", muted) ||
         !SDL_RenderDebugTextFormat(renderer, 80, 432 + offset, properties.minGap == properties.maxGap ? "%.2f" : "Mixed", style.DividedGap()) ||
         !Text(renderer, 12, 460 + offset, properties.Mixed(RoadProperty::Median) ? "Median (mixed)" : "Median", muted))) return false;
    if (!Text(renderer, 12, 132, properties.Mixed(RoadProperty::Alignment) ? "Alignment (mixed)" : "Alignment", muted) || !Text(renderer, 12, 194, "Lanes (total)", muted) ||
        !Text(renderer, 12, 258 + offset, "Shoulders (5 m each)", muted) ||
        !Text(renderer, 12, 312 + offset, properties.Mixed(RoadProperty::Edge) ? "Road edges (mixed)" : "Road edges", muted) ||
        !SDL_RenderDebugTextFormat(renderer, 89, 220, properties.minLanes == properties.maxLanes ? "%d" : "Mixed", laneCount) ||
        !SDL_RenderDebugTextFormat(renderer, 12, styleBottom, properties.Mixed(RoadProperty::Width) ? "Width: Mixed" : "Width: %.2f m",
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
    return true;
}

#endif

#ifndef _WIN32
void RoadEditor::LayoutSidebar(int windowWidth, std::optional<int> dragHeight)
{
    const int x = scene.x + scene.w, width = windowWidth - x;
    const int bar = std::min(8, scene.h);
    const int available = scene.h - bar;
    const int upper = outlinerSplit.Height(available, 250, 68, SdlTransformPanel::height + 60, dragHeight);
    outlineArea = {x, scene.y, width, upper};
    splitterArea = {x, scene.y + upper, width, bar};
    const int transformHeight = std::min(available - upper, SdlTransformPanel::height);
    transformArea = {x, splitterArea.y + bar, width, transformHeight};
    roadPanel = {x, transformArea.y + transformHeight, width, available - upper - transformHeight};
}

void RoadEditor::SetSplitterCursor(bool active)
{
    if (resizeCursor && active != (SDL_GetCursor() == resizeCursor))
        SDL_SetCursor(active ? resizeCursor : SDL_GetDefaultCursor());
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
    scene = {left, top, width - left - right, height - top};
    LayoutSidebar(width);
#endif
    return SDL_SetRenderViewport(renderer, &scene);
}

bool RoadEditor::Render(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera)
{
    if (!UpdateWindowTitle()) return false;
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
            !SDL_RenderDebugTextFormat(renderer, 12, 12, "Sky %02d:%02d CDT", minutes / 60, minutes % 60))
        {
            return false;
        }
    }

    const char* message = StatusText();
    if (*message && !Text(renderer, 12, 28, message, ink)) return false;
    outliner.Update(world, state);
    const auto fields = Inspect(world, state.Target());
    if (!DrawTransformDelta(renderer, fields.value.position)) return false;
    const auto properties = InspectProperties(world, state, EditingDraft());

#ifdef _WIN32
    panels.SyncOutliner(outliner);
    panels.Sync(state, EditingDraft() ? &*state.draft : Selected(world), planView, camera, gizmo.settings, fields, properties);
    return true;
#else
    if (!draggedAsset.empty() && mouse.x >= 0 && mouse.y >= 0 && mouse.x < scene.w && mouse.y < scene.h &&
        !Text(renderer, mouse.x + 14, mouse.y + 14, draggedAsset.c_str(), ink))
    {
        return false;
    }
    if (!browser.MaterialWindow().Draw() || !menu.DrawViewportMenu(renderer))
    {
        return false;
    }
    const int contentHeight = properties.page == PropertiesPage::Road ?
        static_cast<int>(RoadStyleBottom(properties)) + (planView ? 102 : 50) :
        properties.page == PropertiesPage::Marker ? static_cast<int>(markerColorBox.y + markerColorBox.h + 8) : properties.materialSlot ? 182 : 110;
    roadScrollMax = std::max(0, contentHeight - roadPanel.h);
    roadScroll = std::min(roadScroll, roadScrollMax);
    const int width = roadPanel.x + roadPanel.w;
    const SDL_Rect toolbar{0, EditorMenu::barHeight, width, CameraToolbar::height};
    const SDL_FRect grip{(splitterArea.w - 32) * 0.5f, (splitterArea.h - 2) * 0.5f, 32, 2};
    const SDL_Rect shifted{roadPanel.x, roadPanel.y - roadScroll, roadPanel.w, roadPanel.h + roadScroll};
    const SDL_Rect clip{0, roadScroll, roadPanel.w, roadPanel.h};
    return SDL_SetRenderViewport(renderer, &browserPanel) && browser.Draw(renderer, browserPanel.w, browserPanel.h) &&
           SDL_SetRenderViewport(renderer, &outlineArea) && outlinePanel.Draw(renderer, outliner, outlineArea.w, outlineArea.h) &&
           SDL_SetRenderViewport(renderer, &splitterArea) &&
           Fill(renderer, {0, 0, static_cast<float>(splitterArea.w), static_cast<float>(splitterArea.h)}, EditorTheme::chrome) &&
           Fill(renderer, grip, EditorTheme::disabled) &&
           SDL_SetRenderViewport(renderer, &transformArea) && transformPanel.Draw(renderer, transformArea.w, fields) &&
           SDL_SetRenderViewport(renderer, &shifted) && SDL_SetRenderClipRect(renderer, &clip) &&
           RenderPropertiesPanel(renderer, world, properties) && SDL_SetRenderClipRect(renderer, nullptr) &&
           SDL_SetRenderViewport(renderer, &toolbar) && cameraToolbar.Draw(renderer, width, camera, gizmo.settings) &&
           SDL_SetRenderViewport(renderer, nullptr) && menu.DrawBar(renderer, width) && outlinePanel.DrawDragBadge(renderer, outlineArea);
#endif
}

void RoadEditor::ChangeView(WorldDocument& world, FreeCamera& camera)
{
    Release(world);
    rightClick.reset();
    planView = !planView;
    if (planView && !initialized)
    {
        if (state.meshSelection.kind != SelectionKind::None || state.RoadGroupSelected())
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

void RoadEditor::SelectAnchor(const WorldDocument& world, IntersectionAnchor anchor, bool subtract)
{
    if (subtract)
    {
        if (state.anchorSelection == anchor) state.anchorSelection.reset();
        return;
    }
    state.anchorSelection.reset();
    const Road* road = Selected(world);
    if (!road || !state.additionalPoints.empty() || state.point < 0 ||
        (state.point != 0 && state.point + 1 != static_cast<int>(road->points.size())))
    {
        status = "Select one road endpoint first";
        return;
    }
    if (road->points[state.point].anchor || road->points[state.point].ramp)
    {
        status = "This road endpoint is already connected";
        return;
    }
    if (AnchorOccupied(world, anchor))
    {
        status = "This intersection anchor is already connected";
        return;
    }
    state.anchorSelection = anchor;
    status.clear();
}

bool RoadEditor::BeginAnchorRoad(const WorldDocument& world, IntersectionAnchor anchor, SDL_FPoint pointer, bool snap)
{
    if (!state.InspectingAnchor(anchor)) return false;
    if (AnchorOccupied(world, anchor))
    {
        status = "This intersection anchor is already connected";
        return true;
    }
    const auto& junction = world.intersections.at(anchor.intersectionId);
    Road& road = anchorRoad.emplace();
    road.style = state.defaults;
    road.style.oneWay = false;
    road.points.push_back(AttachedEndpoint(junction, anchor, true));
    anchorPress = pointer;
    snapRotation = snap;
    drag = Drag::AnchorRoad;
    status.clear();
    return true;
}

void RoadEditor::UpdateAnchorRoad(const FreeCamera& camera, SDL_FPoint pointer)
{
    mouse = pointer;
    dragChanged = false;
    const SDL_FPoint delta{pointer.x - anchorPress.x, pointer.y - anchorPress.y};
    if (std::hypot(delta.x, delta.y) < 4.0f || pointer.x < 0 || pointer.y < 0 ||
        pointer.x >= scene.w || pointer.y >= scene.h) return;

    Road& road = *anchorRoad;
    const Vector3 origin = road.points.front().position;
    auto end = planView ? std::optional{origin + Vector3{delta.x / scale, 0, -delta.y / scale}}
                        : DragAtElevation(camera, scene, origin, delta);
    if (!end || !ValidRoadSpan(road, origin, *end)) return;
    if (snapRotation)
    {
        const Vector3 offset = *end - origin;
        const double outward = *road.points.front().heading;
        const double angle = outward + DragAngle::Snap(std::atan2(offset.z, offset.x) - outward, true);
        const double radius = std::hypot(offset.x, offset.z);
        *end = origin + Vector3{std::cos(angle) * radius, 0, std::sin(angle) * radius};
    }
    if (road.points.size() == 1 || road.points.back().position != *end)
    {
        road.points.resize(2);
        road.points.back().position = *end;
        road.Rebuild();
    }
    dragChanged = true;
}

void RoadEditor::FinishAnchorRoad(WorldDocument& world)
{
    if (!PreviewRoad()) return;
    Road& road = *anchorRoad;
    const auto anchor = *road.points.front().anchor;
    const auto junction = world.intersections.find(anchor.intersectionId);
    if (!state.InspectingAnchor(anchor) || junction == world.intersections.end() ||
        junction->second.AnchorPosition(anchor.side) != road.points.front().position ||
        AnchorOccupied(world, anchor)) return;
    if (!(road.id = world.AllocateRoadId())) { status = SDL_GetError(); return; }
    EditSnapshot before(state);
    world.roads.push_back(std::move(road));
    state.SelectPoint(static_cast<int>(world.roads.size()) - 1, 1);
    Commit(world, state.selected, std::move(before), &world.roads.back());
}

void RoadEditor::FillSelection(WorldDocument& world)
{
    TerrainBoundary boundary;
    for (SceneKey key : state.outlineSelection)
    {
        if (key.kind == SceneKind::Road) boundary.roads.insert(world.roads.at(key.index).id);
        else boundary.intersections.insert(key.index);
    }
    for (std::size_t i = 0; i < world.Terrain().size(); ++i)
    {
        if (world.Terrain()[i]->boundary == boundary)
        {
            state.SelectRows({{SceneKind::Terrain, static_cast<int>(i)}});
            status = "Selected existing terrain fill";
            return;
        }
    }
    EditSnapshot before(state);
    if (!world.PlaceTerrainFill(boundary))
    {
        status = SDL_GetError();
        return;
    }
    const SceneKey terrain{SceneKind::Terrain, static_cast<int>(world.Terrain().size()) - 1};
    state.SelectRows({terrain});
    EditSnapshot after(state);
    after.addedTerrain = EditSnapshot::TerrainPlacement{{}, {}, std::move(boundary), {}, world.hierarchy.Parent(terrain)};
    history.Record(-1, std::move(before), std::move(after));
    Save(world);
}

void RoadEditor::ConnectEndpoint(WorldDocument& world)
{
    if (!state.anchorSelection || EditingDraft() || drag != Drag::None) return;
    const IntersectionAnchor anchor = *state.anchorSelection;
    const auto junction = world.intersections.find(anchor.intersectionId);
    if (junction == world.intersections.end()) return;
    SelectAnchor(world, anchor, false); // Recheck the selection and occupancy at commit time.
    if (!state.anchorSelection) return;
    Road& road = world.roads[state.selected];
    const bool first = state.point == 0;
    const RoadPoint point = AttachedEndpoint(junction->second, anchor, first);
    RoadPoint& previous = road.points[state.point];
    if (!ValidRoadSpan(road, previous.position, point.position))
    {
        status = SDL_GetError();
        return;
    }
    EditSnapshot before(state, &road);
    if (road.style.curved && !previous.heading)
    {
        // Extending a Bézier endpoint must preserve the shape of its existing span.
        const Vector3 tangent = road.Tangent(state.point);
        previous.heading = std::atan2(tangent.z, tangent.x);
        previous.pitch = std::atan2(tangent.y, std::hypot(tangent.x, tangent.z));
    }
    const int index = first ? 0 : static_cast<int>(road.points.size());
    road.points.insert(road.points.begin() + index, point);
    road.Rebuild();
    state.SelectPoint(state.selected, index);
    Commit(world, state.selected, std::move(before), &road);
}

bool RoadEditor::GizmoTakesClick(RoadPick hit, SDL_Keymod modifiers) const
{
    const bool shift = (modifiers & SDL_KMOD_SHIFT) != 0;
    const bool subtract = (modifiers & SDL_KMOD_CTRL) != 0;
    // The object's foreground translation handle also takes Shift-drags over roads and anchors.
    if (shift && !subtract && state.meshSelection.kind == SelectionKind::Object && gizmo.mode == GizmoMode::Translate)
        return true;
    const bool anchorHit = hit.kind == RoadPickKind::Intersection && hit.side >= 0;
    const bool snapPick = subtract && !shift && gizmo.mode == GizmoMode::Rotate;
    // A marker at the road pivot must still be selectable when the handles overlap it.
    const bool pickPoint = hit.kind == RoadPickKind::Point && (state.point < 0 || (shift && hit.index != state.point));
    const bool addSegment = state.point < 0 && shift && hit.kind == RoadPickKind::Segment;
    const bool addElement = shift && state.point < 0 &&
        (hit.kind == RoadPickKind::Intersection || (hit.kind == RoadPickKind::Road && hit.index != state.selected));
    return !addElement && !addSegment &&
           (snapPick || (!subtract && (!anchorHit || state.RoadGroupSelected()) && !pickPoint));
}

void RoadEditor::PickWorldRoad(WorldDocument& world, const FreeCamera& camera, SDL_Keymod modifiers)
{
    const bool shift = (modifiers & SDL_KMOD_SHIFT) != 0;
    const bool subtract = (modifiers & SDL_KMOD_CTRL) != 0;
    ViewportPicker picker(camera, scene, mouse);
    const Road* selected = Selected(world);
    const RoadPick hit = overlay.Pick(picker, world, selected, camera, scene, shift || subtract);
    const auto placement = SelectedGizmoPlacement(world, camera);
    if (placement && GizmoTakesClick(hit, modifiers))
    {
        ViewportPicker handles(camera, scene, mouse);
        gizmoDrag = gizmo.BeginDrag(GizmoInForeground() ? handles : picker, *placement);
        if (gizmoDrag)
        {
            snapRotation = subtract;
            if (state.RoadGroupSelected() || state.meshSelection.kind == SelectionKind::Intersection)
            {
                BeginGroupDrag(world, Drag::Gizmo);
            }
            else if (selected)
            {
                BeginRoadDrag(*selected, Drag::Gizmo, shift);
            }
            else
            {
                dragStart.emplace(state);
                dragStart->meshTransform = Inspect(world, state.meshSelection).value;
                drag = shift && state.meshSelection.kind == SelectionKind::Object && gizmo.mode == GizmoMode::Translate
                           ? Drag::DuplicateObject : Drag::Gizmo;
            }
            return;
        }
    }

    if (hit.kind == RoadPickKind::Intersection && hit.side >= 0)
    {
        const IntersectionAnchor anchor{hit.index, hit.side};
        if ((shift || subtract) && selected && state.point >= 0)
        {
            const int end = AttachedPoint(*selected, anchor);
            if (end >= 0)
            {
                state.PickPoint(state.selected, end, shift, subtract);
                return;
            }
        }
        if (shift && BeginAnchorRoad(world, anchor, mouse, subtract)) return;
        if ((shift || subtract) && (state.point >= 0 || state.anchorSelection)) SelectAnchor(world, anchor, subtract);
        else if (shift || subtract) state.PickElement({SceneKind::Intersection, hit.index}, shift, subtract);
        else
        {
            state.InspectAnchor(anchor);
            status.clear();
        }
        return;
    }

    // Keep the current point or segment selection when a modified click misses it.
    if ((!state.segments.empty() || state.point >= 0) && (shift || subtract) && ((hit.kind == RoadPickKind::Road && hit.index == state.selected) || hit.kind == RoadPickKind::Empty))
    {
        return;
    }
    switch (hit.kind)
    {
    case RoadPickKind::Segment:
        state.anchorSelection.reset();
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
        state.PickElement({SceneKind::Road, hit.index}, shift, subtract);
        break;
    }
    case RoadPickKind::Intersection:
        state.PickElement({SceneKind::Intersection, hit.index}, shift, subtract);
        break;
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
        if (!shift && !subtract) state.SelectPoint(-1, -1);
        break;
    }
}

void RoadEditor::SubdivideSelection(WorldDocument& world)
{
    if (drag != Drag::None || EditingDraft() || state.selected < 0 || state.segments.empty()) return;
    Road& road = world.roads.at(state.selected);
    EditSnapshot before(state, &road);
    if (!road.SubdivideSegments(state.segments))
    {
        status = "Cannot subdivide a segment with coincident endpoints";
        return;
    }
    const auto segments = std::move(state.segments);
    int inserted = 0;
    for (int segment : segments)
    {
        const int point = segment + inserted + 1;
        state.PickPoint(state.selected, point, inserted > 0, false);
        ++inserted;
    }
    Commit(world, state.selected, std::move(before), &road);
}

void RoadEditor::DeleteSelection(WorldDocument& world)
{
    if (EditingDraft() || drag != Drag::None) return;
    if (state.meshSelection.kind == SelectionKind::Terrain)
    {
        const int index = state.meshSelection.index;
        const auto& chunk = *world.Terrain().at(index);
        EditSnapshot before(state);
        before.addedTerrain = EditSnapshot::TerrainPlacement{
            chunk.boundary ? "" : chunk.Instance().mesh->assetPath, chunk.GetTransform(), chunk.boundary,
            chunk.material, world.hierarchy.Parent({SceneKind::Terrain, index})};
        world.RemoveTerrain(index);
        state.SelectRows({});
        EditSnapshot after(state);
        history.Record(-1, std::move(before), std::move(after));
        Save(world);
        return;
    }
    if (state.meshSelection.kind == SelectionKind::Intersection)
    {
        const int id = state.meshSelection.index;
        EditSnapshot before(state);
        before.junction = EditSnapshot::Junction{id, world.intersections.at(id)};
        before.hierarchy = world.hierarchy;
        before.detachedEnds = world.ConnectedEnds(id);
        for (const auto& end : before.detachedEnds)
        {
            auto& points = world.roads[end.road].points;
            (end.first ? points.front() : points.back()).anchor.reset();
        }
        world.hierarchy.Move({{SceneKind::Intersection, id}}, 0);
        world.intersections.erase(id);
        state.SelectPoint(-1, -1);
        EditSnapshot after(state);
        after.hierarchy = world.hierarchy;
        after.junction = EditSnapshot::Junction{id, {}};
        history.Record(-1, std::move(before), std::move(after));
        Save(world);
        return;
    }
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
        Road connected = road->NewSection();
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

    if (history.ReplaceRoad(world, state, state.selected, std::move(remaining))) Save(world);
    else status = SDL_GetError();
}

void RoadEditor::ApplyOutline(const OutlineCommand& cmd, WorldDocument& world)
{
    Release(world);
    if (cmd.op == OutlineOp::FillTerrain)
    {
        if (state.RoadGroupSelected() && state.outlineSelection == cmd.items) FillSelection(world);
    }
    else if (cmd.op == OutlineOp::Remove && (cmd.target.kind == SceneKind::Object ||
             cmd.target.kind == SceneKind::Intersection || cmd.target.kind == SceneKind::Terrain))
    {
        if (state.SelectedRows() == std::set<SceneKey>{cmd.target}) DeleteSelection(world);
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

void RoadEditor::ApplyInspectorEdit(WorldDocument& world, const InspectorEdit& edit)
{
    if (edit.target != state.Target() || drag != Drag::None) return;
    if (edit.field == InspectorEdit::roadNames)
    {
        if (!EditingDraft() && state.MultipleRoadsSelected())
        {
            Road parsed;
            if (!parsed.SetNameList(edit.text)) return;
            const auto names = parsed.NameList();
            EditRoadGroup(world, [&](RoadProperties& road) { road.names = names; });
            return;
        }
        const int index = EditingDraft() ? -1 : state.selected;
        Road* placed = index >= 0 ? &world.roads.at(index) : nullptr;
        Road* road = EditingDraft() ? &*state.draft : placed;
        if (!road) return;
        EditSnapshot before(state, placed);
        if (!road->SetNameList(edit.text)) return;
        if (placed) outliner.Invalidate();
        Commit(world, index, std::move(before), placed);
        return;
    }
    if (edit.field == InspectorEdit::markerText || edit.field == InspectorEdit::markerTextColor)
    {
        if (edit.target.kind != SelectionKind::Object) return;
        auto* marker = dynamic_cast<Marker*>(world.Objects().at(edit.target.index).get());
        if (!marker) return;
        EditSnapshot before(state), after(state);
        if (edit.field == InspectorEdit::markerTextColor)
        {
            const auto color = ParseTextColor(edit.text);
            if (!color) { status = "Enter a text color as #RRGGBB"; return; }
            if (marker->textColor == *color) return;
            before.markerTextColor = marker->textColor;
            marker->textColor = *color;
            after.markerTextColor = marker->textColor;
        }
        else
        {
            if (edit.text == marker->Text()) return;
            before.markerText = marker->Text();
            if (!marker->SetText(edit.text)) { status = SDL_GetError(); return; }
            after.markerText = marker->Text();
            outliner.Invalidate();
        }
        history.Record(-1, std::move(before), std::move(after));
        Save(world);
        return;
    }
    const TransformFields fields = Inspect(world, edit.target);
    if (!fields.CanEdit(edit.field)) return;
    const auto desired = fields.Parse(edit.field, edit.text);
    if (!desired)
    {
        status = SDL_GetError();
        return;
    }
    if (*desired == fields.value) return;
    const Road* road = Selected(world);
    EditSnapshot before(state, road);
    if (edit.target.kind == SelectionKind::Intersection || edit.target.kind == SelectionKind::RoadGroup)
        before.roadGroup = RoadGroupPose::Capture(world, state.SelectedRows());
    else if (!road) before.meshTransform = fields.value;
    if (!ApplyTransform(world, fields, edit.field, *desired))
    {
        status = SDL_GetError();
        return;
    }
    CommitTransform(world, std::move(before));
}

void RoadEditor::CommitTransform(WorldDocument& world, EditSnapshot before)
{
    if (drag == Drag::DuplicateObject)
    {
        const int index = state.meshSelection.index;
        const int folder = world.hierarchy.Parent({SceneKind::Object, index});
        // Undo removes the copy at its final pose. The original transform was only a drag reference.
        before.meshTransform.reset();
        history.Record(-1, std::move(before), EditSnapshot(state), EditHistory::ObjectEdit{index, folder, {}});
        Save(world);
    }
    else if (before.roadGroup)
    {
        EditSnapshot after(state);
        after.roadGroup = RoadGroupPose::Capture(world, state.SelectedRows());
        if (history.Record(-1, std::move(before), std::move(after))) Save(world);
    }
    else if (before.meshTransform)
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
        Road& road = world.roads[state.selected];
        // Deliberately moving or extending an attached end releases it. Undo restores its link.
        for (auto& point : road.points)
        {
            const bool interior = &point != &road.points.front() && &point != &road.points.back();
            if (interior) point.ramp.reset();
            if (!point.anchor) continue;
            const auto ref = *point.anchor;
            if ((&point != &road.points.front() && &point != &road.points.back()) ||
                point.position != world.intersections.at(ref.intersectionId).AnchorPosition(ref.side))
                point.anchor.reset();
        }
        Commit(world, state.selected, std::move(before), &road);
    }
}

void RoadEditor::MoveObjectCopy(WorldDocument& world, const ObjectTransform& pose)
{
    const int source = dragStart->meshSelection.index;
    if (pose == *dragStart->meshTransform)
    {
        if (dragChanged)
        {
            world.TakeObject(state.meshSelection.index);
            state.SelectRows({{SceneKind::Object, source}});
            dragChanged = false;
        }
        return;
    }

    if (dragChanged)
    {
        if (!SetPlacementTransform(world, state.meshSelection, pose)) status = SDL_GetError();
        return;
    }

    const int index = static_cast<int>(world.Objects().size());
    const int folder = world.hierarchy.Parent({SceneKind::Object, source});
    auto copy = world.Objects().at(source)->Clone();
    if (!copy->SetTransform(pose) || !world.InsertObject(index, std::move(copy), folder))
    {
        status = SDL_GetError();
        return;
    }
    state.SelectRows({{SceneKind::Object, index}});
    dragChanged = true;
}

void RoadEditor::MoveGizmo(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer)
{
    mouse = pointer;
    const auto amount = gizmoDrag->Amount(camera, scene, pointer, gizmo.settings, snapRotation);
    const bool rotating = gizmo.mode == GizmoMode::Rotate;
    if (!amount || (rotating && *amount == appliedAngle))
    {
        return;
    }
    if (dragStart->roadGroup)
    {
        TransformGroup(world, rotating ? Vector3{} : gizmoDrag->axis * *amount, gizmoDrag->axis, rotating ? *amount : 0);
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
            if (drag == Drag::DuplicateObject)
            {
                MoveObjectCopy(world, pose);
                return;
            }
        }
        else
        {
            pose.Rotate(gizmoDrag->axis, *amount);
        }
        if (mesh)
        {
            if (!SetPlacementTransform(world, state.meshSelection, pose)) return;
        }
        else
        {
            Road& road = world.roads[state.selected];
            if (pose.position != road.points.front().position || pose.rotation != road.rotation)
            {
                // Reapply the absolute drag amount to the mouse-down pose.
                road.points = originalPoints;
                road.rotation = originalRotation;
                if (!road.SetPose(pose.position, pose.rotation))
                {
                    road.Rebuild();
                    appliedAngle = 0;
                    dragChanged = false;
                    return;
                }
            }
        }
        if (rotating) appliedAngle = *amount;
        dragChanged = pose != start;
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
    appliedAngle = *amount;
    dragChanged = *amount != 0;
}

EditorEvent RoadEditor::HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event,
                                   WorldDocument& world, FreeCamera& camera)
{
    // Input can change the selection, tool, projection or scene under a stationary cursor.
    // Recheck once after the event queue is drained, even if several events arrived.
    lastHoverPoint = {-1, -1};
#ifndef _WIN32
    if (browser.MaterialWindow().Handle(event)) return EditorEvent::Handled;
#endif
    if (pendingRamp && event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE)
    {
        pendingRamp.reset();
        return EditorEvent::Handled;
    }
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
        if (!menu.IsOpen()) roadMenuTarget.reset();
        if (browser.HandleMaterialPopup(window, renderer, event, browserPanel)) return EditorEvent::Handled;
        if (!draggedAsset.empty())
        {
            if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT)
            {
                SDL_Event release = event;
                if (!SetContentViewport(renderer) || !SDL_ConvertEventToRenderCoordinates(renderer, &release))
                    return EditorEvent::Error;
                if (std::hypot(event.button.x - assetPress.x, event.button.y - assetPress.y) >= 4.0f)
                {
                    const SDL_FPoint point{release.button.x, release.button.y};
                    if (Material::ValidAssetPath(draggedAsset))
                    {
                        const SDL_FPoint local{point.x + scene.x - roadPanel.x,
                                               point.y + scene.y - roadPanel.y + roadScroll};
                        const auto properties = InspectProperties(world, state, EditingDraft());
                        const auto slot = properties.page == PropertiesPage::Road ? MaterialSlotRect(properties) : propertyMaterialSlot;
                        const auto stripSlot = MaterialSlotRect(properties, true);
                        const bool strip = properties.HasTransitionSlot() && SDL_PointInRectFloat(&local, &stripSlot);
                        if (properties.materialSlot && local.y >= roadScroll && local.y < roadScroll + roadPanel.h &&
                            (strip || SDL_PointInRectFloat(&local, &slot)))
                            AssignMaterial(world, draggedAsset, camera, {}, strip);
                        else if (point.x >= 0 && point.y >= 0 && point.x < scene.w && point.y < scene.h)
                            AssignMaterial(world, draggedAsset, camera, point);
                    }
                    else PlaceProp(world, camera, point, draggedAsset);
                }
                draggedAsset.clear();
                return EditorEvent::Handled;
            }
            if (event.type == SDL_EVENT_MOUSE_MOTION)
            {
                SDL_Event motion = event;
                if (!SetContentViewport(renderer) || !SDL_ConvertEventToRenderCoordinates(renderer, &motion))
                    return EditorEvent::Error;
                mouse = {motion.motion.x, motion.motion.y};
                if (!(motion.motion.state & SDL_BUTTON_LMASK)) draggedAsset.clear();
                return EditorEvent::Handled;
            }
            if (event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
            {
                if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN || event.key.key == SDLK_ESCAPE) draggedAsset.clear();
                return EditorEvent::Handled;
            }
            if (event.type == SDL_EVENT_MOUSE_WHEEL) return EditorEvent::Handled;
            if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) draggedAsset.clear();
        }
        std::optional<EditorCommand> command;
        if (menu.HandleViewportMenu(renderer, scene, event, command) ||
            (drag == Drag::None && !splitterGrab && !outlinePanel.Tracking() && !SDL_GetWindowRelativeMouseMode(window) &&
             menu.HandleBar(renderer, event, command)))
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
        std::optional<InspectorEdit> edit;
        if (transformPanel.Key(window, event, edit))
        {
            if (edit) ApplyInspectorEdit(world, *edit);
            return EditorEvent::Handled;
        }
        if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) transformPanel.Blur(window);
    }
#endif
    if (const auto command = menu.Command(event))
    {
        RunCommand(*command, world, camera);
        return EditorEvent::Handled;
    }
#ifdef _WIN32
    if (const auto drop = panels.TakeMaterialDrop(event))
    {
        if (drop->inspectorTarget && *drop->inspectorTarget != state.Target()) return EditorEvent::Handled;
        auto point = drop->viewportFraction;
        if (point)
        {
            if (!SetContentViewport(renderer)) return EditorEvent::Error;
            *point = {point->x * scene.w, point->y * scene.h};
        }
        AssignMaterial(world, drop->path, camera, point, drop->strip);
        return EditorEvent::Handled;
    }
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
    if (const auto edit = panels.TakeInspectorEdit(event))
    {
        ApplyInspectorEdit(world, *edit);
        return EditorEvent::Handled;
    }
    Action action;
    if (panels.Command(event, state, action))
    {
        Release(world);
        Activate(action, world);
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
#ifndef _WIN32
        if (!splitterGrab) SetSplitterCursor(false);
#endif
    }
    if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST)
    {
#ifndef _WIN32
        cameraToolbar.CancelDrag();
        if (splitterGrab)
        {
            splitterGrab.reset();
            SDL_CaptureMouse(false);
        }
        SetSplitterCursor(false);
#endif
        Release(world);
        return EditorEvent::Pass;
    }
    if ((event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_KEY_UP) &&
        (event.key.key == SDLK_LCTRL || event.key.key == SDLK_RCTRL) &&
        (drag == Drag::AnchorRoad || drag == Drag::Rotate || (DraggingGizmo() && gizmo.mode == GizmoMode::Rotate)))
    {
        snapRotation = (event.key.mod & SDL_KMOD_CTRL) != 0;
        if (drag == Drag::AnchorRoad) UpdateAnchorRoad(camera, mouse);
        else if (DraggingGizmo()) MoveGizmo(world, camera, mouse);
        else UpdatePlanDrag(world, mouse);
        return EditorEvent::Handled;
    }
    if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_Q && (event.key.mod & SDL_KMOD_CTRL) &&
        !(event.key.mod & (SDL_KMOD_ALT | SDL_KMOD_SHIFT | SDL_KMOD_GUI)) &&
        ViewportFocused() && !SDL_GetWindowRelativeMouseMode(window))
    {
        if (!event.key.repeat) SubdivideSelection(world);
        return EditorEvent::Handled;
    }
    if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_F && ViewportFocused() &&
        !SDL_GetWindowRelativeMouseMode(window) && !(event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_GUI)))
    {
        if (!event.key.repeat && drag == Drag::None)
        {
            if (state.RoadGroupSelected()) FillSelection(world);
            else ConnectEndpoint(world);
        }
        return EditorEvent::Handled;
    }
    if (ViewportFocused() && event.type == SDL_EVENT_KEY_DOWN &&
        !(event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_GUI | SDL_KMOD_SHIFT)))
    {
        if (event.key.key == SDLK_ESCAPE && state.anchorSelection && drag == Drag::None)
        {
            state.anchorSelection.reset();
            status.clear();
            return EditorEvent::Handled;
        }
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
        if ((drag == Drag::Measure || drag == Drag::AnchorRoad) && event.key.key == SDLK_ESCAPE)
        {
            Release(world);
            return EditorEvent::Handled;
        }
        if (dragStart && event.key.key == SDLK_ESCAPE)
        {
            if (drag == Drag::DuplicateObject) MoveObjectCopy(world, *dragStart->meshTransform);
            else if (dragStart->roadGroup) dragStart->roadGroup->Restore(world);
            else if (dragStart->meshTransform)
            {
                SetPlacementTransform(world, state.meshSelection, *dragStart->meshTransform);
            }
            else
            {
                Road& road = world.roads[state.selected];
                state.point = extensionSource.value_or(state.point);
                state.anchorSelection = dragStart->anchorSelection;
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
            Activate(Action::Cancel, world);
            return EditorEvent::Handled;
        }
        if (event.key.key == SDLK_R || (event.key.key == SDLK_ESCAPE && planView))
        {
            ChangeView(world, camera);
            return EditorEvent::Handled;
        }
        if (event.key.key == SDLK_RETURN && planView)
        {
            Activate(Action::Finish, world);
            return EditorEvent::Handled;
        }
        if (event.key.key == SDLK_N && planView && drag == Drag::None && ViewportFocused() &&
            !(event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_ALT | SDL_KMOD_GUI)))
        {
            if (!state.drawing) BeginDraft(world);
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

    if (pendingRamp)
    {
        const SDL_FPoint local{pointer.x - scene.x, pointer.y};
        const bool inside = local.x >= 0 && local.x < scene.w && local.y >= 0 && local.y < scene.h;
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_RIGHT)
            pendingRamp.reset();
        else if (inside && e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_LEFT)
        {
            pendingRamp->dragging = true;
            AimRamp(camera, local);
        }
        else if (inside && pendingRamp->dragging && e.type == SDL_EVENT_MOUSE_MOTION)
            AimRamp(camera, local);
        else if (e.type == SDL_EVENT_MOUSE_BUTTON_UP && e.button.button == SDL_BUTTON_LEFT && pendingRamp->dragging)
        {
            if (inside) { AimRamp(camera, local); FinishRamp(world); }
            else pendingRamp.reset();
        }
        return EditorEvent::Handled;
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
        const float windowY = pointer.y + scene.y;
        const SDL_Point position{static_cast<int>(std::floor(pointer.x)), static_cast<int>(std::floor(windowY))};
        const auto onBar = [&] { return SDL_PointInRect(&position, &splitterArea); };
        const bool leftPress = e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_LEFT;
        const bool leftRelease = e.type == SDL_EVENT_MOUSE_BUTTON_UP && e.button.button == SDL_BUTTON_LEFT;
        if (onBar() && leftPress && !outlinePanel.Tracking())
        {
            splitterGrab = windowY - splitterArea.y;
            SDL_CaptureMouse(true);
            SetSplitterCursor(true);
        }
        if (splitterGrab)
        {
            const bool heldMotion = e.type == SDL_EVENT_MOUSE_MOTION && (e.motion.state & SDL_BUTTON_LMASK);
            if (heldMotion || leftRelease)
                LayoutSidebar(windowWidth, static_cast<int>(std::lround(pointer.y - *splitterGrab)));
            if (leftRelease || (e.type == SDL_EVENT_MOUSE_MOTION && !heldMotion))
            {
                splitterGrab.reset();
                SDL_CaptureMouse(false);
                SetSplitterCursor(onBar());
            }
            mouse = {-1, -1};
            return EditorEvent::Handled;
        }
        if (e.type == SDL_EVENT_MOUSE_MOTION)
            SetSplitterCursor(onBar() && !outlinePanel.Tracking() && !e.motion.state);
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) transformPanel.Blur(window);
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
        if (onBar() || pointer.y < 0 || pointer.y >= scene.h || pointer.x < 0 || pointer.x >= windowWidth)
        {
            mouse = {-1, -1};
            return EditorEvent::Handled;
        }
        if (pointer.x < scene.x)
        {
            mouse = {-1, -1};
            auto path = browser.HandlePointer(e, pointer, browserPanel.w, browserPanel.h);
            if (!path.empty() && (!planView || Material::ValidAssetPath(path)))
            {
                draggedAsset = std::move(path);
                assetPress = {event.button.x, event.button.y};
            }
            return EditorEvent::Handled;
        }
        if (pointer.x >= roadPanel.x)
        {
            mouse = {-1, -1};
            if (e.type == SDL_EVENT_MOUSE_WHEEL && roadScrollMax > 0 && windowY >= roadPanel.y)
            {
                const float wheel = e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -e.wheel.y : e.wheel.y;
                roadScroll = std::clamp(roadScroll - static_cast<int>(wheel * 32), 0, roadScrollMax);
                return EditorEvent::Handled;
            }
            if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_LEFT)
            {
                if (windowY < roadPanel.y)
                {
                    transformPanel.Click(window, {pointer.x - transformArea.x, windowY - transformArea.y}, transformArea.w, Inspect(world, state.Target()));
                    return EditorEvent::Handled;
                }
                const SDL_FPoint local{pointer.x - roadPanel.x, windowY - roadPanel.y + roadScroll};
                const auto properties = InspectProperties(world, state, EditingDraft());
                if (properties.roadNames && SDL_PointInRectFloat(&local, &roadNamesBox))
                {
                    transformPanel.StartEdit(window, state.Target(), InspectorEdit::roadNames, *properties.roadNames);
                    return EditorEvent::Handled;
                }
                if (properties.page == PropertiesPage::Marker && SDL_PointInRectFloat(&local, &markerTextBox))
                {
                    transformPanel.StartEdit(window, state.Target(), InspectorEdit::markerText, properties.markerText);
                    return EditorEvent::Handled;
                }
                if (properties.page == PropertiesPage::Marker &&
                    (SDL_PointInRectFloat(&local, &markerColorBox) || SDL_PointInRectFloat(&local, &markerColorSwatch)))
                {
                    transformPanel.StartEdit(window, state.Target(), InspectorEdit::markerTextColor, FormatTextColor(properties.textColor));
                    return EditorEvent::Handled;
                }
                if (properties.page == PropertiesPage::Marker && SDL_PointInRectFloat(&local, &markerClearButton))
                {
                    AssignMaterial(world, {}, camera, {});
                    return EditorEvent::Handled;
                }
                for (const Button& button : Buttons(properties))
                {
                    if (button.enabled && SDL_PointInRectFloat(&local, &button.rect))
                    {
                        Activate(button.action, world);
                        break;
                    }
                }
            }
            return e.type == SDL_EVENT_MOUSE_WHEEL && !planView ? EditorEvent::Pass : EditorEvent::Handled;
        }
#endif
    }
    if (!planView && !DraggingGizmo() && drag != Drag::AnchorRoad && e.type != SDL_EVENT_MOUSE_BUTTON_DOWN)
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
        if (drag == Drag::AnchorRoad)
        {
            if (e.motion.state & SDL_BUTTON_LMASK) UpdateAnchorRoad(camera, pointer);
            else Release(world);
        }
        else if (DraggingGizmo())
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
            if (drag == Drag::AnchorRoad)
            {
                UpdateAnchorRoad(camera, pointer);
                FinishAnchorRoad(world);
            }
            else if (DraggingGizmo())
            {
                MoveGizmo(world, camera, pointer);
            }
            else if (drag == Drag::Rotate || drag == Drag::Move)
            {
                UpdatePlanDrag(world, pointer);
                if (selectOnlyOnClick) state.SelectPoint(state.selected, state.point);
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
