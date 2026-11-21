#include "transform_gizmo.hpp"
#include "editor_colors.hpp"
#include "viewport_picker.hpp"
#include "free_camera.hpp"
#include "world_renderer.hpp"

#include <array>
#include <algorithm>
#include <cmath>

namespace
{
    constexpr std::array<SDL_FColor, 3> axisColors{{{1, 0.18f, 0.14f, 1}, {0.24f, 0.90f, 0.30f, 1}, {0.22f, 0.48f, 1, 1}}};
    constexpr double ringPixels = 56.0;

    std::optional<Vector3> OnPlane(Vector3 eye, Vector3 ray, Vector3 origin, Vector3 normal)
    {
        const double divisor = Math::Dot(normal, ray);
        // Near-parallel rays have no stable grab position. Keep the last valid edit.
        if (std::abs(divisor) < 0.02 * Math::Length(ray))
        {
            return {};
        }
        const double t = Math::Dot(origin - eye, normal) / divisor;
        if (t < worldNearDepth)
        {
            return {};
        }
        return eye + ray * t;
    }
}

void DragAngle::Update(double bearing)
{
    total += std::remainder(bearing - previous, 2.0 * std::numbers::pi);
    previous = bearing;
}

double DragAngle::Snap(double radians, bool enabled)
{
    constexpr double step = Math::DegreesToRadians(5.0);
    return enabled ? std::round(radians / step) * step : radians;
}

DragScale::DragScale(const FreeCamera& camera, Vector3 pivot, Vector3 axis, SDL_FPoint mouse)
    : start(mouse), direction{1, -1}
{
    if (axis == Vector3{}) return;
    // Project the local axis at the pivot, accounting for perspective. When the
    // axis points directly at the camera, use upward motion to enlarge it.
    direction = {0, -1};
    const auto view = camera.WorldToView(pivot), along = camera.WorldDirectionToView(axis);
    if (view.z < worldNearDepth) return;
    const double dx = along.x - along.z * view.x / view.z;
    const double dy = -along.y + along.z * view.y / view.z;
    const double length = std::hypot(dx, dy);
    if (length > 0.05)
        direction = {static_cast<float>(dx / length), static_cast<float>(dy / length)};
}

double DragScale::Amount(SDL_FPoint mouse) const
{
    const double pixels = (double(mouse.x) - start.x) * direction.x + (double(mouse.y) - start.y) * direction.y;
    if (!std::isfinite(pixels)) return 1;
    // 100 pixels doubles/halves the starting size. Exponential scaling never
    // crosses zero, flips an axis, or collapses a mesh during a long drag.
    return std::exp2(std::clamp(pixels / 100.0, -10.0, 10.0));
}

ObjectTransform ScaleObjectTransform(ObjectTransform start, int axis, double factor)
{
    if (axis < 0 || axis > 3 || !std::isfinite(factor) || factor <= 0) return start;
    if (axis == 0 || axis == 3) start.scale.x *= factor;
    if (axis == 1 || axis == 3) start.scale.y *= factor;
    if (axis == 2 || axis == 3) start.scale.z *= factor;
    return start;
}

std::optional<GizmoPlacement> GizmoPlacement::At(Vector3 pivot, const FreeCamera& camera, SDL_Rect scene)
{
    const double focal = ViewportFocalLength(scene);
    const double depth = camera.WorldToView(pivot).z;
    if (focal <= 0 || depth < worldNearDepth)
    {
        return {};
    }
    return GizmoPlacement{pivot, depth / focal};
}

void TransformGizmo::Load(MeshAssets& assets)
{
    const auto load = [&](const char* path)
    {
        auto model = assets.Load(path);

        if (!model)
        {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Transform gizmo %s unavailable: %s", path, SDL_GetError());
        }

        return model;
    };

    arrow = load("editor/arrow.glb");
    ring = load("editor/rotation_gizmo.glb");
    scaleArrow = load("editor/scale_gizmo.glb");
    scaleBox = load("editor/box.glb");
}

const GizmoAxes& TransformGizmo::Axes(const GizmoPlacement& placement) const
{
    // ObjectTransform stores rotation followed by local component scales. World
    // nonuniform scaling would require shear, which that transform cannot store.
    return mode == GizmoMode::Scale || settings.space == GizmoSpace::Local ? placement.localAxes : worldAxes;
}

MeshInstance TransformGizmo::Handle(const GizmoPlacement& placement, const GizmoAxes& axes, int axis) const
{
    MeshTransform transform;
    transform.origin = placement.origin;
    // Both arrow meshes point along local X. The ring's normal is local Y.
    const int from = mode == GizmoMode::Rotate ? 1 : 0;
    GizmoAxes basis = axes;

    if (axis < 3 && axis != from)
    {
        basis[from] = axes[axis];
        basis[axis] = axes[from] * -1;
    }

    // The authored ring radius is four model units.
    const double scale = ringPixels / 4 * placement.metersPerPixel;
    transform.x = basis[0] * scale;
    transform.y = basis[1] * scale;
    transform.z = basis[2] * scale;

    const auto mesh = mode == GizmoMode::Scale ? (axis == 3 ? scaleBox : scaleArrow)
                                              : (mode == GizmoMode::Translate ? arrow : ring);
    return {mesh, transform};
}

void TransformGizmo::Draw(WorldRenderer& renderer, const GizmoPlacement& placement, const GizmoDrag* drag,
                          std::optional<int> hoveredAxis) const
{
    const GizmoAxes& axes = drag ? drag->startAxes : Axes(placement);

    for (int axis = 0; axis < (mode == GizmoMode::Scale ? 4 : 3); ++axis)
    {
        if (mode == GizmoMode::Rotate && placement.yawOnly && axis != 1) continue;
        if (mode == GizmoMode::Translate && placement.horizontalTranslation && axis == 1) continue;
        const SDL_FColor color = axis == 3 ? SDL_FColor{0.9f, 0.9f, 0.9f, 1} : axisColors[axis];
        renderer.SubmitMesh(Handle(placement, axes, axis), hoveredAxis == axis ? EditorColors::amber : color);
    }
}

std::optional<int> TransformGizmo::PickAxis(ViewportPicker& picker, const GizmoPlacement& placement, const GizmoDrag* drag) const
{
    const GizmoAxes& axes = drag ? drag->startAxes : Axes(placement);
    std::optional<int> hit;

    // The three shafts start inside the box. Give its projected face priority
    // so a click at the pivot reliably selects uniform scaling.
    if (mode == GizmoMode::Scale && picker.TestMesh(Handle(placement, axes, 3))) return 3;

    for (int axis = 0; axis < 3; ++axis)
    {
        if (mode == GizmoMode::Rotate && placement.yawOnly && axis != 1) continue;
        if (mode == GizmoMode::Translate && placement.horizontalTranslation && axis == 1) continue;
        if (picker.TestMesh(Handle(placement, axes, axis)))
        {
            hit = axis;
        }
    }

    return hit;
}

std::optional<GizmoDrag> TransformGizmo::BeginDrag(ViewportPicker& picker, const GizmoPlacement& placement) const
{
    if (const auto hit = PickAxis(picker, placement))
    {
        return GizmoDrag(mode, Axes(placement), *hit, placement, picker);
    }

    return {};
}

GizmoDrag::GizmoDrag(GizmoMode tool, const GizmoAxes& axes, int axisIndex, const GizmoPlacement& placement, const ViewportPicker& pick)
    : axis(axisIndex < 3 ? axes[axisIndex] : Vector3{}), axisIndex(axisIndex), axisName("XYZA"[axisIndex]),
      startAxes(axes), mode(tool), origin(placement.origin),
      mouseStart(pick.mouse), unitsPerPixel(placement.metersPerPixel)
{
    if (mode == GizmoMode::Scale)
    {
        scaling.emplace(pick.camera, origin, axis, mouseStart);
        return;
    }
    const double metersPerPixel = placement.metersPerPixel;
    const Vector3 view = Math::Normalize(origin - pick.eye);

    normal = mode == GizmoMode::Rotate ? axis : Math::Normalize(view - axis * Math::Dot(view, axis));

    if (std::abs(Math::Dot(normal, Math::Normalize(pick.ray))) > 0.12)
    {
        if (const auto p = OnPlane(pick.eye, pick.ray, origin, normal))
        {
            start = *p - origin;

            if (mode == GizmoMode::Translate || Math::Length(start) > metersPerPixel)
            {
                return;
            }
        }
    }

    // Edge-on rotation uses motion along the projected ring tangent. An axis
    // pointing straight at the camera can be translated by dragging vertically.
    useScreen = true;
    screenDirection = {0, -1};

    if (mode == GizmoMode::Rotate)
    {
        const Vector3 hit = pick.HitPosition();
        const Vector3 radial = hit - origin;
        const Vector3 tangent = pick.camera.WorldDirectionToView(Math::Cross(axis, radial));
        const Vector3 v = pick.camera.WorldToView(hit);
        const double dx = tangent.x - tangent.z * v.x / v.z;
        const double dy = -tangent.y + tangent.z * v.y / v.z;
        const double length = std::hypot(dx, dy);

        if (length > metersPerPixel)
        {
            screenDirection = {static_cast<float>(dx / length), static_cast<float>(dy / length)};
        }

        unitsPerPixel = 1.0 / ringPixels;
    }
}

std::optional<double> GizmoDrag::Amount(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse, const TransformSettings& settings, bool snapRotation)
{
    if (scaling) return scaling->Amount(mouse);
    if (useScreen)
    {
        const double amount = ((mouse.x - mouseStart.x) * screenDirection.x + (mouse.y - mouseStart.y) * screenDirection.y) * unitsPerPixel;
        return mode == GizmoMode::Translate ? settings.SnapDistance(amount) : DragAngle::Snap(amount, snapRotation);
    }

    const auto point = OnPlane(camera.GetPosition(), ViewportRay(camera, scene, mouse), origin, normal);

    if (mode == GizmoMode::Translate)
    {
        if (!point) return {};
        return settings.SnapDistance(Math::Dot(*point - origin - start, axis));
    }

    // Keep the last valid bearing at the pivot or when the ring plane is edge-on.
    // Ctrl can still change the snap at that bearing without further pointer motion.
    if (point)
    {
        const Vector3 current = *point - origin;
        if (Math::Length(current) > unitsPerPixel)
        {
            rotation.Update(std::atan2(Math::Dot(axis, Math::Cross(start, current)), Math::Dot(start, current)));
        }
    }
    return rotation.Amount(snapRotation);
}
