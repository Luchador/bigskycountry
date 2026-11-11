#pragma once

#include "road.hpp"
#include "transform_gizmo.hpp"

class WorldDocument;
struct RoadEditorState;

enum class RoadPickKind { Empty, Road, Segment, Point, Object, Terrain, Mesh, Intersection };

struct RoadPick
{
    RoadPickKind kind = RoadPickKind::Empty;
    int index = -1;
};

class RoadOverlay
{
public:
    RoadOverlay();
    static std::optional<GizmoPlacement> PointPlacement(Vector3 point, const FreeCamera& camera, SDL_Rect scene);
    void Draw(WorldRenderer& renderer, const Road& road, const RoadEditorState& selection,
              const FreeCamera& camera, SDL_Rect scene) const;
    void DrawIntersections(WorldRenderer& renderer, const WorldDocument& world, int selected, const FreeCamera& camera, SDL_Rect scene) const;
    RoadPick Pick(ViewportPicker& picker, const WorldDocument& world, const Road* selected,
                  const FreeCamera& camera, SDL_Rect scene) const;

private:
    std::array<MeshInstance, 2> IntersectionMarker(const GizmoPlacement& placement) const;
    MeshInstance Marker(const GizmoPlacement& placement) const;
    std::shared_ptr<const Mesh> marker;
};
