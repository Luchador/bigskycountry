#pragma once

#include <SDL3/SDL.h>
#include <optional>
#include <array>
#include <vector>
#include "material_action.hpp"

enum class EditorCommand { Save, CookWorld, Undo, Redo, ToggleWireframe, ToggleRoadNames, ToggleTrafficLanes, ToggleFog, ToggleShadows, ToggleVisibilityStatistics, ProjectSettings, CreateRoad, CreateRailroad, CreateMarker, CreateIntersection, CreateYIntersection, CreateLake, CreateRiver,
                           CreateExitRamp, CreateEntranceRamp, CreateRailSwitchLeft, CreateRailSwitchRight, CreateRailSwitchLeftReverse, CreateRailSwitchRightReverse, ToggleRailroadCrossing, FlattenSegment, SplitRoad, CreateDeadEnd, ToggleOverpass, StartCurvedConnector, FinishCurvedConnector, CancelCurvedConnector, StartRiverConnection, FinishRiverConnection, DisconnectRiver, CancelRiverConnection, CreateSpawnPoint, CreateTownSquare, CreateCulDeSac };
struct RampMenuOptions { bool exit, entrance; };
struct CurveMenuOptions { bool start = false, finish = false, cancel = false; };
struct RiverMenuOptions { bool start = false, finish = false, disconnect = false, cancel = false; };
struct SplitMenuOptions { bool enabled, railroad; };
enum class OutlineOp;

class EditorMenu
{
public:
#ifdef _WIN32
    static std::optional<MaterialAction> OpenMaterialMenu(SDL_Window* window, int screenX, int screenY, bool onMaterial);
    static std::optional<OutlineOp> OpenFolderMenu(void* frame, int screenX, int screenY);
#endif
    bool Attach(SDL_Window* window);
    std::optional<EditorCommand> OpenViewport(SDL_FPoint windowPoint, SDL_FPoint viewportPoint, SDL_Rect scene, bool canCreateIntersection, std::optional<RampMenuOptions> ramps,
                                              std::optional<bool> flattenEnabled, bool crossingAvailable = false, bool crossingExists = false, std::optional<SplitMenuOptions> split = {}, std::optional<bool> deadEnd = {}, std::optional<bool> overpass = {}, std::optional<std::array<bool, 4>> switches = {}, CurveMenuOptions curves = {}, RiverMenuOptions rivers = {}, std::optional<bool> culDeSac = {});
#ifndef _WIN32
    static constexpr int barHeight = 24;
    bool HandleBar(SDL_Renderer* renderer, const SDL_Event& event, std::optional<EditorCommand>& command);
    bool DrawBar(SDL_Renderer* renderer, int width) const;
    bool HandleViewportMenu(SDL_Renderer* renderer, SDL_Rect scene, const SDL_Event& event, std::optional<EditorCommand>& command);
    bool DrawViewportMenu(SDL_Renderer* renderer) const;
    bool IsOpen() const { return openMenu >= 0 || popup != Popup::Closed; }
#endif
    bool WireframeEnabled() const { return wireframe; }
    void ToggleWireframe();
    bool RoadNamesEnabled() const { return roadNames; }
    void ToggleRoadNames();
    bool TrafficLanesEnabled() const { return trafficLanes; }
    void ToggleTrafficLanes();
    bool ShadowsEnabled() const { return shadows; }
    void ToggleShadows();
    bool FogEnabled() const { return fog; }
    void ToggleFog();
    bool VisibilityStatisticsEnabled() const { return visibilityStatistics; }
    void ToggleVisibilityStatistics();
    void SetHistoryEnabled(bool canUndo, bool canRedo);
    std::optional<EditorCommand> Command(const SDL_Event& event) const;

private:
    struct ViewportItem { const char* label; EditorCommand command; bool enabled = true; };
    std::array<std::vector<ViewportItem>, 3> viewportItems;
    bool roadNames = true;
    bool trafficLanes = false;
    bool fog = true;
    bool shadows = true;
    bool visibilityStatistics = false;
    bool wireframe = false, undoEnabled = false, redoEnabled = false;
    SDL_WindowID windowID = 0;
#ifdef _WIN32
    Uint32 commandEvent = 0;
    void* editMenu = nullptr;
    void* viewMenu = nullptr;
#else
    int openMenu = -1, barRow = 0;
    bool Enabled(EditorCommand command) const;
    enum class Popup { Closed, Root, Submenu };
    Popup popup = Popup::Closed;
    std::array<SDL_FRect, 3> rootItems{};
    SDL_FRect submenu{};
    int activeCategory = 0, firstItem = 0;
    void LayoutViewportSubmenu(SDL_Rect scene);
    int activeItem = 0;
#endif
};
