#pragma once

#include "mesh.hpp"
#include "transform_settings.hpp"
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
    bool yawOnly = false; // Keep intersections horizontal.
    static std::optional<GizmoPlacement> At(Vector3 pivot, const FreeCamera& camera, SDL_Rect scene);
};

// A circular handle reports bearings modulo one turn. Accumulate signed motion
// so the delta readout stays continuous through 180 degrees and multiple turns.
class DragAngle
{
public:
    explicit DragAngle(double startBearing = 0) : previous(startBearing) {}
    void Update(double bearing);
    double Amount(bool snap) const { return Snap(total, snap); }
    static double Snap(double radians, bool enabled);

private:
    double previous, total = 0;
};

// Signed displacement in meters or rotation in radians relative to mouse-down.
class GizmoDrag
{
public:
    std::optional<double> Amount(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse, const TransformSettings& settings, bool snapRotation);
    Vector3 axis;
    char AxisName() const { return axisName; }

private:
    friend class TransformGizmo;
    GizmoDrag(GizmoMode mode, const GizmoAxes& axes, int axisIndex, const GizmoPlacement& placement, const ViewportPicker& pick);
    DragAngle rotation;
    char axisName;
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
    void Draw(WorldRenderer& renderer, const GizmoPlacement& placement, const GizmoDrag* drag = nullptr,
              std::optional<int> hoveredAxis = {}) const;
    std::optional<int> PickAxis(ViewportPicker& picker, const GizmoPlacement& placement, const GizmoDrag* drag = nullptr) const;
    // Starts a drag only if a handle is closer than the scene geometry already tested.
    std::optional<GizmoDrag> BeginDrag(ViewportPicker& picker, const GizmoPlacement& placement) const;

    GizmoMode mode = GizmoMode::Translate;
    TransformSettings settings;

private:
    const GizmoAxes& Axes(const GizmoPlacement& placement) const;
    MeshInstance Handle(const GizmoPlacement& placement, const GizmoAxes& axes, int axis) const;
    std::shared_ptr<const Mesh> arrow, ring;
};
