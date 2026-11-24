#include "mesh_editor.hpp"
#include "viewport_picker.hpp"
#include "view_geometry.hpp"
#include <numbers>
#include "editor_colors.hpp"
#ifdef _WIN32
#include "windows_mesh_controls.hpp"
#endif
#include <algorithm>
#include <cmath>

namespace
{
    constexpr double previewRadius = 100;
    SDL_Rect Scene(SDL_FRect view) { return {0, 0, int(view.w), int(view.h)}; }
}

void MeshEditorWindow::UpdateCamera()
{
    camera.LookAt(Vector3{std::sin(yaw) * std::cos(pitch) * distance, std::sin(pitch) * distance,
                          -std::cos(yaw) * std::cos(pitch) * distance} * previewRadius, {});
}
std::size_t MeshEditorWindow::RowCount() const
{ return showCollision ? collisionDraft.shapes.size() : instance.mesh->materials->slots.size(); }
std::string MeshEditorWindow::ShapeLabel(std::size_t index) const
{
    const auto& shape = collisionDraft.shapes.at(index);
    return std::string(shape.type == CollisionShapeType::Capsule ? "Capsule " : "Sphere ") + std::to_string(index + 1);
}
void MeshEditorWindow::SelectShape(std::optional<std::size_t> index)
{
    FinishCollisionDrag(false);
    selectedShape = index && *index < collisionDraft.shapes.size() ? index : std::nullopt;
    collisionHover.reset(); dirty = true;
#ifdef _WIN32
    if (controls) controls->RefreshSlots();
#endif
}
void MeshEditorWindow::SyncCollision()
{
    collisionSnapshot = MeshCollision::Current(*instance.mesh);
    collisionDraft = *collisionSnapshot;
    if (selectedShape && *selectedShape >= collisionDraft.shapes.size()) selectedShape.reset();
    dirty = true;
#ifdef _WIN32
    if (controls) controls->Reload();
#endif
}
bool MeshEditorWindow::CommitCollision(const MeshCollisionData& next, bool history)
{
    const auto before = MeshCollision::Current(*instance.mesh);
    if (*before == next) { SyncCollision(); return true; }
    if (!MeshCollision::Save(*instance.mesh, next))
    {
        status = SDL_GetError(); SyncCollision(); return false;
    }
    if (history)
    {
        collisionUndo.push_back(*before);
        if (collisionUndo.size() > 64) collisionUndo.erase(collisionUndo.begin());
        collisionRedo.clear();
    }
    status = "Collision saved"; SyncCollision(); return true;
}
void MeshEditorWindow::SetCollisionMode(MeshCollisionMode mode)
{
    CancelPointer(); auto next = collisionDraft; next.mode = mode;
    CommitCollision(next);
}
void MeshEditorWindow::CollisionCommand(int command)
{
    CancelPointer();
    if (command == 0)
    {
        showCollision = !showCollision; scroll = 0; selectedShape.reset();
        SyncCollision(); return;
    }
    if (command >= 1 && command <= 3) { SetCollisionMode(MeshCollisionMode(command - 1)); return; }
    auto next = collisionDraft;
    if (command == 4 || command == 5 || command == 6)
    {
        if (next.shapes.size() >= MeshCollision::maxShapes)
        { status = "Maximum 128 collision shapes"; dirty = true; return; }
        CollisionShape shape;
        if (command == 6)
        {
            if (!selectedShape) return;
            shape = next.shapes[*selectedShape];
        }
        else
        {
            shape.type = command == 4 ? CollisionShapeType::Capsule : CollisionShapeType::Sphere;
            const auto& b = instance.mesh->bounds;
            const double height = std::max(.01, b.max.y - b.min.y);
            const double diameter = std::clamp(height * .08, .01, 1000.0);
            shape.transform.position = {(b.min.x + b.max.x) * .5, b.min.y + height * .25, (b.min.z + b.max.z) * .5};
            shape.transform.scale = {diameter, shape.type == CollisionShapeType::Capsule ? height * .4 : diameter, diameter};
        }
        next.shapes.push_back(shape); next.mode = MeshCollisionMode::Simple;
        showCollision = true; selectedShape = next.shapes.size() - 1;
    }
    else if (command == 7)
    {
        if (!selectedShape) return;
        next.shapes.erase(next.shapes.begin() + *selectedShape); selectedShape.reset();
    }
    else return;
    CommitCollision(next);
    if (selectedShape) scroll = std::max(0.f, float(*selectedShape + 1) * 88 - SlotClip().h);
}

bool MeshEditorWindow::CollisionKey(SDL_Keycode key, SDL_Keymod mods)
{
    if (!showCollision || (mods & SDL_KMOD_ALT)) return false;
    if (key == SDLK_ESCAPE)
    {
        if (collisionDrag) FinishCollisionDrag(false);
        else
        {
            if (panelGrab) panelWidth = panelDragStart;
            CancelPointer(); SelectShape({});
        }
        return true;
    }
    if (mods & SDL_KMOD_CTRL)
    {
        if (key == SDLK_D) { CollisionCommand(6); return true; }
        if (key == SDLK_Z || key == SDLK_Y)
        {
            CancelPointer();
            auto& source = key == SDLK_Z ? collisionUndo : collisionRedo;
            auto& target = key == SDLK_Z ? collisionRedo : collisionUndo;
            if (!source.empty())
            {
                const auto before = *collisionSnapshot, next = source.back();
                if (CommitCollision(next, false)) { source.pop_back(); target.push_back(before); }
            }
            return true;
        }
        return false;
    }
    if (key == SDLK_W || key == SDLK_E || key == SDLK_S)
    {
        CancelPointer(); collisionGizmo.mode = key == SDLK_W ? GizmoMode::Translate : key == SDLK_E ? GizmoMode::Rotate : GizmoMode::Scale;
        collisionHover.reset(); dirty = true; return true;
    }
    if (key == SDLK_DELETE) { CollisionCommand(7); return true; }
    return false;
}

std::optional<GizmoPlacement> MeshEditorWindow::CollisionGizmoPlacement() const
{
    if (!showCollision || !selectedShape) return {};
    auto pose = collisionDraft.shapes[*selectedShape].transform; pose.scale = {1, 1, 1};
    const auto frame = pose.ToMeshTransform();
    auto placement = GizmoPlacement::At(instance.transform.Point(pose.position), camera, Scene(View()));
    if (placement) placement->localAxes = {frame.x, frame.y, frame.z};
    return placement;
}
bool MeshEditorWindow::CollisionPointerDown(SDL_FPoint point)
{
    if (!showCollision) return false;
    UpdateCamera(); const auto view = View(); point.x -= view.x; point.y -= view.y;
    ViewportPicker picker(camera, Scene(view), point);
    if (const auto placement = CollisionGizmoPlacement())
        if (auto drag = collisionGizmo.BeginDrag(picker, *placement))
        {
            collisionDrag = std::move(drag); collisionDragStart = collisionDraft.shapes[*selectedShape].transform;
            SDL_CaptureMouse(true); dirty = true; return true;
        }
    const auto ray = Math::Normalize(picker.ray);
    double nearest = std::numeric_limits<double>::infinity(); std::optional<std::size_t> hit;
    for (std::size_t i = 0; i < collisionDraft.shapes.size(); ++i)
        if (const auto h = MeshCollision::Capsule(collisionDraft.shapes[i], instance.transform).Raycast(picker.eye, ray, nearest))
        { nearest = h->distance; hit = i; }
    SelectShape(hit);
    return hit.has_value();
}
void MeshEditorWindow::CollisionPointerMove(SDL_FPoint point)
{
    if (!showCollision) return;
    UpdateCamera(); const auto view = View(); point.x -= view.x; point.y -= view.y;
    if (collisionDrag && selectedShape)
    {
        const auto amount = collisionDrag->Amount(camera, Scene(view), point, collisionGizmo.settings, SDL_GetModState() & SDL_KMOD_CTRL);
        if (!amount) return;
        auto next = collisionDragStart;
        auto& shape = collisionDraft.shapes[*selectedShape];
        if (collisionGizmo.mode == GizmoMode::Translate)
            next.position = next.position + collisionDrag->axis * (*amount / instance.transform.x.x);
        else if (collisionGizmo.mode == GizmoMode::Rotate) next.Rotate(collisionDrag->axis, *amount);
        else
        {
            const int axis = collisionDrag->AxisIndex();
            if (shape.type == CollisionShapeType::Sphere || axis == 3) next.scale = next.scale * *amount;
            else if (axis == 1) next.scale.y *= *amount;
            else { next.scale.x *= *amount; next.scale.z = next.scale.x; }
        }
        auto candidate = collisionDraft; candidate.shapes[*selectedShape].transform = next;
        if (MeshCollision::Valid(candidate))
        {
            collisionDraft = std::move(candidate);
#ifdef _WIN32
            if (controls) controls->InvalidateSlots();
#endif
        }
        dirty = true;
    }
    else
    {
        std::optional<int> hovered;
        if (point.x >= 0 && point.y >= 0 && point.x < view.w && point.y < view.h)
            if (const auto placement = CollisionGizmoPlacement())
            { ViewportPicker picker(camera, Scene(view), point); hovered = collisionGizmo.PickAxis(picker, *placement); }
        if (hovered != collisionHover) { collisionHover = hovered; dirty = true; }
    }
}
void MeshEditorWindow::FinishCollisionDrag(bool commit)
{
    if (!collisionDrag) return;
    collisionDrag.reset(); SDL_CaptureMouse(false);
    if (commit) CommitCollision(collisionDraft);
    else { collisionDraft = *collisionSnapshot; dirty = true; }
}

void MeshEditorWindow::DrawCollision()
{
    if (!showCollision) return;
    preview->BeginUnlit(); preview->BeginForeground();
    const auto scene = Scene(View());
    const double focal = ViewportFocalLength(scene);
    const auto planes = ViewGeometry::SidePlanes(focal, scene);
    const auto line = [&](Vector3 a, Vector3 b, SDL_FColor color) {
        const auto stroke = ViewGeometry::ProjectStroke(camera.WorldToView(a), camera.WorldToView(b), planes, focal, scene, worldNearDepth, .8f);
        if (!stroke) return;
        std::array<Vector3, 4> corners;
        for (int j = 0; j < 4; ++j)
        {
            const auto p = stroke->corners[j];
            corners[j] = camera.GetPosition() + camera.ViewRay((p.x - scene.w * .5) / focal, (scene.h * .5 - p.y) / focal) *
                (j < 2 ? stroke->startDepth : stroke->endDepth);
        }
        preview->SubmitTriangle(corners[0], corners[1], corners[2], color);
        preview->SubmitTriangle(corners[0], corners[2], corners[3], color);
    };
    for (std::size_t i = 0; i < collisionDraft.shapes.size(); ++i)
    {
        const auto cap = MeshCollision::Capsule(collisionDraft.shapes[i], instance.transform);
        const auto color = selectedShape == i ? EditorColors::orange : SDL_FColor{.25f, .85f, .95f, 1};
        auto axis = Math::Normalize(cap.b - cap.a); if (Math::Length(axis) < .5) axis = {0, 1, 0};
        const auto right = Math::Normalize(Math::Cross(axis, std::abs(axis.x) < .9 ? Vector3{1, 0, 0} : Vector3{0, 0, 1}));
        const auto forward = Math::Cross(axis, right);
        constexpr int steps = 32;
        for (int j = 0; j < steps; ++j)
        {
            const double a = j * 2 * std::numbers::pi / steps, b = (j + 1) * 2 * std::numbers::pi / steps;
            const auto p = (right * std::cos(a) + forward * std::sin(a)) * cap.radius;
            const auto q = (right * std::cos(b) + forward * std::sin(b)) * cap.radius;
            line(cap.a + p, cap.a + q, color); line(cap.b + p, cap.b + q, color);
        }
        for (auto radial : {right, forward})
            for (int sign : {-1, 1})
                for (int j = 0; j < steps / 2; ++j)
                {
                    const auto curve = [&](int sample) {
                        const double t = sample * 2 * std::numbers::pi / steps;
                        return (sign < 0 ? cap.a : cap.b) + (radial * std::cos(t) + axis * (sign * std::sin(t))) * cap.radius;
                    };
                    line(curve(j), curve(j + 1), color);
                }
        for (auto radial : {right, right * -1, forward, forward * -1})
            line(cap.a + radial * cap.radius, cap.b + radial * cap.radius, color);
    }
    if (const auto placement = CollisionGizmoPlacement())
        collisionGizmo.Draw(*preview, *placement, collisionDrag ? &*collisionDrag : nullptr, collisionHover);
}
