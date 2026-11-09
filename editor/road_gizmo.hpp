#pragma once

#include "mesh.hpp"
#include "road.hpp"
#include <optional>

class FreeCamera;
class WorldDocument;
class WorldRenderer;

enum class GizmoMode { Translate, Rotate };
enum class PickKind { Empty, Road, Point, Axis };

struct GizmoPick
{
    PickKind kind = PickKind::Empty;
    int index = -1;
    Vector3 position{};
};

// The same transformed triangles drive rendering and hit testing.
class RoadGizmo
{
public:
    void Load(MeshAssets& assets);
    void Draw(WorldRenderer& renderer, const Road& road, int point, const FreeCamera& camera, SDL_Rect scene) const;
    GizmoPick Pick(const WorldDocument& world, int road, int point, const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse) const;
    GizmoMode mode = GizmoMode::Translate;

private:
    MeshInstance Marker(Vector3 position, double metersPerPixel) const;
    MeshInstance Handle(Vector3 position, double metersPerPixel, int axis) const;
    std::shared_ptr<const Mesh> marker, arrow, ring;
};

// Map pointer motion to a displacement or angle about one world axis, relative to mouse-down.
class GizmoDrag
{
public:
    GizmoDrag(GizmoMode mode, int axisIndex, Vector3 point, Vector3 hit, const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse);
    std::optional<double> Amount(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse) const;
    Vector3 axis;

private:
    GizmoMode mode;
    Vector3 origin, normal, start;
    SDL_FPoint mouseStart, screenDirection{};
    double unitsPerPixel = 0;
    bool useScreen = false;
};
