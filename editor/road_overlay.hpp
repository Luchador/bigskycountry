#pragma once

#include "road.hpp"
#include "transform_gizmo.hpp"

class WorldDocument;
struct RoadEditorState;

enum class RoadPickKind { Empty, Road, Segment, Point, Object, Terrain, Mesh, Intersection };

enum class RampControl { Attachment, Taper };

struct RampLaneHit
{
    int host, side;
};

struct RampControlHit
{
    RampControl control;
    int host, station, side;
};

struct RoadPick
{
    RoadPickKind kind = RoadPickKind::Empty;
    int index = -1;
    int side = -1; // Intersection approach, or -1 for the center.
};

class RoadOverlay
{
public:
    RoadOverlay();
    static std::optional<GizmoPlacement> PointPlacement(Vector3 point, const FreeCamera& camera, SDL_Rect scene);
    static Vector3 SplinePosition(const ViewportPicker& pick, SDL_Rect scene);
    void Draw(WorldRenderer& renderer, const Road& road, const RoadEditorState& selection, int roadIndex,
              const FreeCamera& camera, SDL_Rect scene) const;
    std::optional<std::pair<int, int>> PickGroupPoint(ViewportPicker& picker, const WorldDocument& world,
        const RoadEditorState& selection, const FreeCamera& camera, SDL_Rect scene) const;
    void DrawRampControls(WorldRenderer& renderer, const WorldDocument& world, const Road& selected,
                          const FreeCamera& camera, SDL_Rect scene) const;
    std::optional<RampLaneHit> PickRampLane(ViewportPicker& picker, const WorldDocument& world,
        const RoadEditorState& selection, const FreeCamera& camera, SDL_Rect scene) const;
    std::optional<RampControlHit> PickRampControl(ViewportPicker& picker, const WorldDocument& world,
                                                  const Road& selected, const FreeCamera& camera, SDL_Rect scene) const;
    static std::optional<double> LaneDistanceAtPointer(const Road& host, int side, const FreeCamera& camera,
                                                       SDL_Rect scene, SDL_FPoint pointer, double reference);
    void DrawIntersections(WorldRenderer& renderer, const WorldDocument& world, const RoadEditorState& selection,
                           const FreeCamera& camera, SDL_Rect scene) const;
    RoadPick Pick(ViewportPicker& picker, const WorldDocument& world, const Road* selected,
                  const FreeCamera& camera, SDL_Rect scene, bool preferAnchor = false, Vector3* placement = nullptr) const;

private:
    void DrawDirection(WorldRenderer& renderer, const Road& ramp, const FreeCamera& camera, SDL_Rect scene) const;
    std::array<MeshInstance, 2> IntersectionMarker(const GizmoPlacement& placement, double yaw) const;
    MeshInstance Marker(const GizmoPlacement& placement) const;
    std::shared_ptr<const Mesh> marker;
};
