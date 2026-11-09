#pragma once

#include "mesh.hpp"
#include "transform_space.hpp"
#include <array>
#include <optional>

class FreeCamera;
class ViewportPicker;
class WorldRenderer;

enum class GizmoMode { Translate, Rotate };

using GizmoAxes = std::array<Vector3, 3>;
inline constexpr GizmoAxes worldAxes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

struct GizmoPlacement
{
    Vector3 origin;
    double metersPerPixel;
    GizmoAxes localAxes = worldAxes; // Orthonormal X, Y, Z supplied by the selected object.

    // A fixed-size gizmo centered on any visible world-space pivot.
    static std::optional<GizmoPlacement> At(Vector3 pivot, const FreeCamera& camera, SDL_Rect scene);
};

// Signed displacement in meters or rotation in radians relative to mouse-down.
class GizmoDrag
{
public:
    std::optional<double> Amount(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse) const;
    Vector3 axis;

private:
    friend class TransformGizmo;
    GizmoDrag(GizmoMode mode, const GizmoAxes& axes, int axisIndex, const GizmoPlacement& placement, const ViewportPicker& pick);
    GizmoAxes startAxes;
    GizmoMode mode;
    Vector3 origin, normal, start;
    SDL_FPoint mouseStart, screenDirection{};
    double unitsPerPixel = 0;
    bool useScreen = false;
};

// World- or local-axis handles. The caller chooses the pivot and applies the resulting drag to its selection.
class TransformGizmo
{
public:
    void Load(MeshAssets& assets);
    void Draw(WorldRenderer& renderer, const GizmoPlacement& placement, const GizmoDrag* drag = nullptr) const;
    // Starts a drag only if a handle is closer than the scene geometry already tested.
    std::optional<GizmoDrag> BeginDrag(ViewportPicker& picker, const GizmoPlacement& placement) const;

    GizmoMode mode = GizmoMode::Translate;
    GizmoSpace space = GizmoSpace::World;

private:
    const GizmoAxes& Axes(const GizmoPlacement& placement) const;
    MeshInstance Handle(const GizmoPlacement& placement, const GizmoAxes& axes, int axis) const;
    std::shared_ptr<const Mesh> arrow, ring;
};
