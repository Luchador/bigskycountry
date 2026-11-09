#pragma once

#include "road.hpp"
#include "transform_gizmo.hpp"
#include <set>

class WorldDocument;

enum class RoadPickKind { Empty, Road, Segment, Point };

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
    static GizmoAxes PointAxes(const Road& road, int point);
    void Draw(WorldRenderer& renderer, const Road& road, int point, const std::set<int>& segments,
              const FreeCamera& camera, SDL_Rect scene) const;
    RoadPick Pick(ViewportPicker& picker, const WorldDocument& world, const Road* selected,
                  const FreeCamera& camera, SDL_Rect scene) const;

private:
    MeshInstance Marker(const GizmoPlacement& placement) const;
    std::shared_ptr<const Mesh> marker;
};
