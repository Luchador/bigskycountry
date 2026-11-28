#pragma once

#include "application.hpp"
#include "road_action.hpp"
#include "properties_inspector.hpp"
#include "scene_outliner.hpp"
#ifdef _WIN32
#include "windows_panels.hpp"
#endif
#ifndef _WIN32
#include "browser_splitter.hpp"
#include "camera_toolbar.hpp"
#include "outliner_splitter.hpp"
#include "sdl_transform_panel.hpp"
#include "sdl_outliner.hpp"
#include "sdl_project_settings.hpp"
#endif
#include "content_browser.hpp"
#include "edit_history.hpp"
#include "road_overlay.hpp"
#include "road_names.hpp"
#include "traffic_overlay.hpp"
#include "plan_renderer.hpp"
#include "plan_selection.hpp"
#include "road_intersection.hpp"
#include "highway_ramp_tool.hpp"
#include "railroad_switch.hpp"
#include "viewport_picker.hpp"
#include "transform_gizmo.hpp"
#include "editor_menu.hpp"
#include <SDL3/SDL.h>
#include <functional>

class RoadEditor : public EditorTool
{
public:
#ifdef _WIN32
    SDL_Window* CreateRenderWindow(SDL_Window* frame) override { return panels.Create(frame, browser); }
    bool ViewportFocused() const override { return !browser.MeshWindow().Focused() && panels.ViewportFocused(); }
#else
    bool ViewportFocused() const override
    {
        return !browser.MeshWindow().Focused() && !projectSettings.Focused() && !browser.Focused() && !browser.MaterialWindow().Focused() && !transformPanel.Focused() && !outlinePanel.Focused();
    }
#endif
    bool Initialize(SDL_Window* window, MeshAssets& models) override;
    void SubmitGizmos(WorldRenderer& renderer, const WorldDocument& world, const FreeCamera& camera) override;
    bool DraggingGizmo() const override { return gizmoDrag.has_value(); }
    const Road* Selected(const WorldDocument& world) const;
    const Road* PreviewRoad() const override
    {
        return anchorRoad && dragChanged ? &*anchorRoad : nullptr;
    }
    bool GeometrySelected(SceneKey key) const override { return state.GeometrySelected(key); }
    bool WireframeEnabled() const override { return menu.WireframeEnabled(); }
    bool VisibilityStatisticsEnabled() const override { return menu.VisibilityStatisticsEnabled(); }
    bool ShadowsEnabled() const override { return menu.ShadowsEnabled(); }
    bool FogEnabled() const override { return menu.FogEnabled(); }
    bool IsPlanView() const override { return planView; }
    bool SetContentViewport(SDL_Renderer* renderer) override;
    EditorEvent HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event, WorldDocument& world, FreeCamera& camera) override;
    void Update(WorldDocument& world, const FreeCamera& camera, double elapsed) override;
    bool Render(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera) override;
    void ReleaseRenderResources() override { browser.MeshWindow().Close(); planRenderer.Clear(); planDraft.reset(); }

private:
    const TerrainBrushSettings& TerrainBrush() const;
    std::optional<Vector3> PickTerrainBrush(const WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer) const;
    bool TerrainBrushInput(const SDL_Event& event, SDL_FPoint pointer, WorldDocument& world, const FreeCamera& camera);
    void StampTerrain(WorldDocument& world, Vector3 center, double elapsed);
    void FinishTerrainStroke(WorldDocument& world, bool cancel = false);
    std::vector<std::optional<Vector3>> TerrainBrushContour(const WorldDocument& world) const;
    void SubmitTerrainBrush(WorldRenderer& renderer, const WorldDocument& world, const FreeCamera& camera) const;
    bool DrawTerrainBrush(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera) const;
    struct TerrainStroke
    {
        EditSnapshot before;
        TerrainBrushSettings settings;
        double height = 0, elapsed = 0;
        std::optional<Vector3> previous;
    };
    std::optional<TerrainStroke> terrainStroke;
    std::optional<Vector3> terrainHover;
    enum class Drag
    {
        None,
        Move,
        Rotate,
        Pan,
        Measure,
        AnchorRoad,
        Gizmo,
        DuplicateObject,
        WaterPoint,
        BoxSelect
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
        bool mixed = false;
    };

    std::vector<Button> Buttons(const PropertiesInspector& properties) const;
#endif
    RoadStyle& Style(WorldDocument& world);
    bool EditingDraft() const { return planView && state.drawing; }
    Vector3 ToWorld(float x, float y) const;
    SDL_FPoint ToScreen(Vector3 point) const;
    SDL_FPoint Handle(const Road& road, int index) const;
    bool RenderPlan(SDL_Renderer* renderer, const WorldDocument& world);
    bool DrawMeasurement(SDL_Renderer* renderer) const;
    void UpdateSelectionBox(SDL_FPoint pointer);
    void FinishSelectionBox(WorldDocument& world, const FreeCamera& camera);
    bool DrawSelectionBox(SDL_Renderer* renderer) const;
    bool DrawTransformDelta(SDL_Renderer* renderer, Vector3 position) const;
#ifndef _WIN32
    bool RenderPropertiesPanel(SDL_Renderer* renderer, const WorldDocument& world, const PropertiesInspector& properties);
#endif
    void ChangeView(WorldDocument& world, FreeCamera& camera);
    void FocusSelection(const WorldDocument& world, FreeCamera& camera);
    void AssignMaterial(WorldDocument& world, const std::string& path, const FreeCamera& camera,
                        std::optional<SDL_FPoint> viewportPoint, bool strip = false);
    void PlaceObject(WorldDocument& world, ObjectType type, const std::string& path, Vector3 position, int folder = 0);
    void PlaceProp(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer, const std::string& path);
    void OpenViewportMenu(WorldDocument& world, FreeCamera& camera);
    void BeginDraft(WorldDocument& world);
    std::optional<GizmoPlacement> SelectedGizmoPlacement(const WorldDocument& world, const FreeCamera& camera) const;
    std::optional<SDL_FPoint> GizmoPointer() const;
    bool GizmoInForeground() const;
    bool GizmoTakesClick(RoadPick hit, SDL_Keymod modifiers) const;
    void UpdateGizmoHover(const WorldDocument& world, const FreeCamera& camera, const GizmoPlacement& placement);
    void SelectAnchor(const WorldDocument& world, IntersectionAnchor anchor, bool subtract);
    bool BeginAnchorRoad(const WorldDocument& world, IntersectionAnchor anchor, SDL_FPoint pointer, bool snap);
    void UpdateAnchorRoad(const FreeCamera& camera, SDL_FPoint pointer);
    void FinishAnchorRoad(WorldDocument& world);
    void FillSelection(WorldDocument& world);
    void ConnectRoads(WorldDocument& world);
    void ConnectRamps(WorldDocument& world);
    void ConnectAnchors(WorldDocument& world);
    void ConnectEndpoint(WorldDocument& world);
    void SubdivideSelection(WorldDocument& world);
    void PickWorldRoad(WorldDocument& world, const FreeCamera& camera, SDL_Keymod modifiers);
    bool OpenMeshAtPointer(WorldDocument& world, const FreeCamera& camera);
    bool PickWaterControl(WorldDocument& world, const FreeCamera& camera, bool doubleClick, SDL_Keymod modifiers);
    void MoveWaterPoint(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer);
    void BeginWaterEditing(WorldDocument& world);
    bool FinishWaterEditing(WorldDocument& world);
    void CancelWaterEditing(WorldDocument& world);
    void BeginRoadDrag(const Road& road, Drag kind, bool shift);
    void MovePoint(WorldDocument& world, Vector3 position);
    void RotatePlanPoint(Road& road);
    void MoveGizmo(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer);
    void ApplyOutline(const OutlineCommand& cmd, WorldDocument& world);
    void ApplyInspectorEdit(WorldDocument& world, const InspectorEdit& edit);
    std::optional<SceneKey> PlanElementAt(const WorldDocument& world, Vector3 pos) const;
    SDL_FPoint PlanGroupHandle(const ObjectTransform& frame) const;
    void BeginGroupDrag(const WorldDocument& world, Drag kind);
    void TransformGroup(WorldDocument& world, Vector3 offset, Vector3 axis, double angle);
    void UpdatePlanDrag(WorldDocument& world, SDL_FPoint pointer);
    void Activate(Action action, WorldDocument& world);
    void ChangeStyle(Action action, WorldDocument& world);
    void EditRoadGroup(WorldDocument& world, const std::function<void(RoadProperties&)>& edit);
    void PressCanvas(WorldDocument& world, float x, float y, SDL_Keymod modifiers);
    void DeleteSelection(WorldDocument& world);
    void Motion(WorldDocument& world, const SDL_MouseMotionEvent& motion);
    void MoveObjectCopy(WorldDocument& world, const ObjectTransform& pose);
    void Release(WorldDocument& world);
    const char* StatusText() const
    {
        if (riverConnectionStart && status.empty()) return "Right-click the receiving river or lake: Water > Connect River Here (Esc cancels)";
        if (curvedConnectorStart && status.empty()) return "Right-click the second road: Infrastructure > Finish Curved Connector Here (Esc cancels)";
        if (state.anchorSelection && status.empty())
            return state.meshSelection.kind == SelectionKind::Anchor ? "Press F to create a road between the anchors" :
                                                                      "Press F to connect the road endpoint";
        return status.c_str();
    }
    bool UpdateWindowTitle();
    bool Save(WorldDocument& world);
    void CreateRamp(EditorCommand command, WorldDocument& world);
    void RunCommand(EditorCommand command, WorldDocument& world, FreeCamera& camera);
    void CommitTransform(WorldDocument& world, EditSnapshot before);
    void Commit(WorldDocument& world, int roadIndex, EditSnapshot before, const Road* after = nullptr);

    double scale = 4.0; // pixels per meter
    Vector3 center{};
    SDL_Rect scene{};
#ifndef _WIN32
    void LayoutSidebar(int windowWidth, std::optional<int> dragHeight = {});
    void SetSplitterCursor(bool active, bool horizontal = false);
    SDL_Rect browserPanel{}, roadPanel{}, transformArea{}, outlineArea{}, splitterArea{}, browserSplitterArea{};
    OutlinerSplitter outlinerSplit;
    BrowserSplitter browserSplit;
    std::optional<float> splitterGrab;
    bool draggingBrowser = false;
    SDL_Cursor* resizeCursor = nullptr; // Released by SDL on video shutdown.
    SDL_Cursor* browserResizeCursor = nullptr;
    SdlOutliner outlinePanel;
    SdlTransformPanel transformPanel;
    SdlProjectSettings projectSettings;
    int roadScroll = 0, roadScrollMax = 0;
    bool intersectionPaintOpen = false;
    int intersectionPaintOwner = -1;
#endif
    struct ViewportClick
    {
        SDL_FPoint windowPoint;
        ViewportPicker picker;
        SDL_FPoint motion{};
    };
    std::optional<ViewportClick> rightClick;
    Vector3 creationPoint{};
    struct RoadMenuTarget
    {
        int index;
        std::uint64_t revision;
        std::optional<RoadSplineLocation> curveSite;
        std::optional<RoadIntersectionInsertion> insertion;
        std::optional<RampPlan> exitRamp, entranceRamp;
        std::array<std::optional<RailroadSwitchPlan>, 4> switches;
        int segment = -1;
        int point = -1;
    };
    std::optional<RoadMenuTarget> roadMenuTarget;
    struct CurvedConnectorStart { int roadId; std::uint64_t revision; RoadSplineLocation at; };
    std::optional<CurvedConnectorStart> curvedConnectorStart;
    struct RiverEndTarget { std::uint64_t id; int end; };
    struct RiverMenuTarget { std::uint64_t id; int end; Vector3 position; };
    std::optional<RiverEndTarget> riverConnectionStart;
    std::optional<RiverMenuTarget> riverMenuTarget;
    struct CrossingMenuTarget
    {
        int road;
        std::uint64_t revision;
        CrossingSite site;
        int existing = -1;
    };
    std::optional<CrossingMenuTarget> crossingMenuTarget;
    struct RampHandleDrag
    {
        RampControlHit hit;
        int ramp;
        bool exit;
        RampLink original;
        double initialGore, grabOffset, reference, low, high;
        SDL_FPoint press;
        bool moved;
        EditSnapshot before;
    };
    std::optional<RampHandleDrag> rampDrag;
    void PressRampControl(WorldDocument& world, const FreeCamera& camera, RampControlHit hit);
    void MoveRampControl(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer);
    void FinishRampControl(WorldDocument& world, bool commit);
    RoadEditorState state;
    EditHistory history;
    struct WaterEditSession
    {
        int index;
        WaterDefinition original;
        EditHistory history; // Draft edits stay out of document undo until Finish.
    };
    std::optional<WaterEditSession> waterEdit;
    int riverExtendEnd = -1;
    SceneOutliner outliner;
    EditorMenu menu;
    RoadOverlay overlay;
    RoadNames roadNames;
    EditorTrafficOverlay trafficOverlay;
    PlanRenderer planRenderer;
    std::optional<Road> planDraft;
    TransformGizmo gizmo;
    std::optional<GizmoDrag> gizmoDrag;
    std::optional<int> hoveredAxis;
    SDL_FPoint lastHoverPoint{-1, -1};
    DragAngle planRotation;
    double appliedAngle = 0; // Signed radians actually applied since mouse-down.
    double appliedScale = 1; // Multiplier actually applied since mouse-down.
    bool snapRotation = false;
    std::vector<RoadPoint> originalPoints;
    Vector3 originalRotation{};
    Vector3 groupPivot{}, groupOffset{};
    std::optional<int> extensionSource; // Original endpoint index for the entire Shift-drag.
    bool selectOnlyOnClick = false; // In plan view, defer narrowing the selection until release.
#ifndef _WIN32
    CameraToolbar cameraToolbar;
    std::string draggedAsset;
    SDL_FPoint assetPress{};
#endif
    ContentBrowser browser;
#ifdef _WIN32
    WindowsPanels panels;
#endif
    bool planView = false;
    bool dragChanged = false;
    SDL_Window* frame = nullptr;
    std::string cleanTitle;
    std::uint64_t savedRevision = 0;
    bool titleShowsChanges = false;
    bool saveFailed = false;
    std::optional<EditSnapshot> dragStart;
    bool initialized = false;
    Drag drag = Drag::None;
    Vector3 measureStart{}, measureEnd{}; // World-space endpoints while Drag::Measure is active.
    Vector3 planDragStart{};
    struct SelectionBox
    {
        SDL_FPoint start, end;
        BoxSelectionMode mode;
        std::optional<SceneKey> clickTarget;
        bool moved = false;
        bool perspective = false;
    };
    std::optional<SelectionBox> selectionBox;
    SDL_FPoint mouse{};
    SDL_FPoint anchorPress{};
    std::optional<Road> anchorRoad; // Preview only until the left button is released.
    std::string status;
};
