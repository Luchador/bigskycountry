#pragma once

#include <SDL3/SDL.h>
#include <optional>
#include "material_action.hpp"

enum class EditorCommand { Save, Undo, Redo, ToggleWireframe, ToggleRoadNames, ProjectSettings, CreateRoad, CreateRailroad, CreateMarker, CreateIntersection,
                           CreateExitRamp, CreateEntranceRamp, ToggleRailroadCrossing, FlattenSegment, SplitRoad, CreateDeadEnd, ToggleOverpass };
struct RampMenuOptions { bool exit, entrance; };
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
                                              std::optional<bool> flattenEnabled, bool crossingAvailable = false, bool crossingExists = false, std::optional<SplitMenuOptions> split = {}, std::optional<bool> deadEnd = {}, std::optional<bool> overpass = {});
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
    void SetHistoryEnabled(bool canUndo, bool canRedo);
    std::optional<EditorCommand> Command(const SDL_Event& event) const;

private:
    bool roadNames = true;
    bool wireframe = false, undoEnabled = false, redoEnabled = false;
    SDL_WindowID windowID = 0;
#ifdef _WIN32
    Uint32 commandEvent = 0;
    void* editMenu = nullptr;
    void* viewMenu = nullptr;
#else
    RampMenuOptions rampOptions{};
    bool intersectionEnabled = true;
    std::optional<bool> deadEndEnabled, overpassState;
    bool crossingAvailable = false, crossingExists = false;
    // Segment and control-point actions share the second root-menu row.
    std::optional<bool> roadActionEnabled;
    EditorCommand roadAction = EditorCommand::FlattenSegment;
    const char* roadActionLabel = "Flatten Segment";
    bool roadActionFocused = false;
    int openMenu = -1, barRow = 0;
    bool Enabled(EditorCommand command) const;
    enum class Popup { Closed, Create, Submenu, Ramp };
    Popup popup = Popup::Closed;
    SDL_FRect rootItem{}, roadActionItem{}, submenu{};
    int activeItem = 0;
#endif
};
