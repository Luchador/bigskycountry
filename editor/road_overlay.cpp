#include "road_overlay.hpp"
#include "editor_colors.hpp"
#include "edit_history.hpp"
#include "free_camera.hpp"
#include "view_geometry.hpp"
#include "viewport_picker.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"

#include <cmath>
#include <numbers>

namespace
{
    constexpr Vector3 markerOffsetPixels{0, 6, 0};
    constexpr float overlayStrokeWidth = 1.5f;

    // Reuse the same constant-pixel-width stroke for the spline and radius guides.
    struct OverlayStroke
    {
        double focal;
        SDL_Rect viewport;
        std::array<Vector3, 4> sides;
        Vector3 eye, forward, right, up;

        OverlayStroke(const FreeCamera& camera, SDL_Rect scene, double focalLength)
            : focal(focalLength), viewport{0, 0, scene.w, scene.h},
              sides(ViewGeometry::SidePlanes(focal, viewport)), eye(camera.GetPosition()),
              forward(camera.ViewRay(0, 0)), right((camera.ViewRay(1, 0) - forward) * (1.0 / focal)),
              up((camera.ViewRay(0, 1) - forward) * (1.0 / focal)) {}

        template<typename Visit>
        void Triangles(Vector3 viewStart, Vector3 viewEnd, Visit visit) const
        {
            const auto stroke = ViewGeometry::ProjectStroke(viewStart, viewEnd, sides, focal, viewport,
                                                             worldNearDepth, overlayStrokeWidth);
            if (!stroke) return;
            std::array<Vector3, 4> corners;
            for (int j = 0; j < 4; ++j)
            {
                const SDL_FPoint p = stroke->corners[j];
                const Vector3 ray = forward + right * (p.x - viewport.w * 0.5) + up * (viewport.h * 0.5 - p.y);
                // Bias toward the eye so coincident pavement cannot hide the stroke.
                const double depth = j < 2 ? stroke->startDepth : stroke->endDepth;
                corners[j] = eye + ray * std::max(worldNearDepth, depth / 1.0001);
            }
            visit(corners[0], corners[1], corners[2]);
            visit(corners[0], corners[2], corners[3]);
        }
    };

    template<typename Visit>
    void VisitSegmentTriangles(const Road& road, const FreeCamera& camera, SDL_Rect scene, Visit visit)
    {
        const double focal = ViewportFocalLength(scene);
        if (focal <= 0) return;
        const OverlayStroke stroke(camera, scene, focal);
        const Vector3 lift = camera.WorldDirectionToView(markerOffsetPixels);
        const auto display = [&](Vector3 point)
        {
            const Vector3 v = camera.WorldToView(point);
            return v + lift * (v.z / focal); // Match the elevated centers of the point markers.
        };

        for (int i = 0; i + 1 < static_cast<int>(road.points.size()); ++i)
        {
            const auto samples = road.SegmentSamples(i);
            if (samples.empty()) continue;
            Vector3 previous = display(samples.front());
            for (Vector3 sample : samples.subspan(1))
            {
                const Vector3 next = display(sample);
                stroke.Triangles(previous, next, [&](Vector3 a, Vector3 b, Vector3 c) { visit(i, a, b, c); });
                previous = next;
            }
        }
    }

    template<typename Visit>
    void VisitPoints(const Road& road, const FreeCamera& camera, SDL_Rect scene, Visit visit)
    {
        for (int i = 0; i < static_cast<int>(road.points.size()); ++i)
        {
            if (const auto placement = RoadOverlay::PointPlacement(road.points[i].position, camera, scene))
            {
                visit(i, *placement);
            }
        }
    }
}

std::optional<GizmoPlacement> RoadOverlay::PointPlacement(Vector3 point, const FreeCamera& camera, SDL_Rect scene)
{
    auto placement = GizmoPlacement::At(point, camera, scene);
    if (placement)
    {
        placement->origin = point + markerOffsetPixels * placement->metersPerPixel;
    }
    return placement;
}

RoadOverlay::RoadOverlay()
{
    auto diamond = std::make_shared<Mesh>();

    for (Vector3 axis : std::array<Vector3, 3>{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}})
    {
        diamond->vertices.push_back({axis, axis});
        diamond->vertices.push_back({axis * -1, axis * -1});
    }

    diamond->indices = {2, 4, 0, 2, 1, 4, 2, 5, 1, 2, 0, 5, 3, 0, 4, 3, 4, 1, 3, 1, 5, 3, 5, 0};

    marker = std::move(diamond);
}

MeshInstance RoadOverlay::Marker(const GizmoPlacement& placement) const
{
    const double r = 7.0 * placement.metersPerPixel;
    return {marker, {placement.origin, {r, 0, 0}, {0, r, 0}, {0, 0, r}}};
}

void RoadOverlay::Draw(WorldRenderer& renderer, const Road& road, const RoadEditorState& selection, const FreeCamera& camera, SDL_Rect scene) const
{
    VisitSegmentTriangles(road, camera, scene, [&](int i, Vector3 a, Vector3 b, Vector3 c)
    {
        renderer.SubmitTriangle(a, b, c, selection.segments.contains(i) ? EditorColors::amber : EditorColors::blue);
    });

    VisitPoints(road, camera, scene, [&](int i, const GizmoPlacement& placement)
    {
        renderer.SubmitMesh(Marker(placement), selection.PointSelected(i) ? EditorColors::amber : EditorColors::blue);
    });
}

std::array<MeshInstance, 2> RoadOverlay::IntersectionMarker(const GizmoPlacement& placement, double yaw) const
{
    const double length = 14.0 * placement.metersPerPixel;
    const double width = 3.85 * placement.metersPerPixel;
    const auto axes = Intersection::Directions(yaw);
    return {MeshInstance{marker, {placement.origin, axes[0] * length, {0, width, 0}, axes[1] * width}},
            MeshInstance{marker, {placement.origin, axes[0] * width, {0, width, 0}, axes[1] * length}}};
}

void RoadOverlay::DrawIntersections(WorldRenderer& renderer, const WorldDocument& world, const RoadEditorState& selection,
                                    const FreeCamera& camera, SDL_Rect scene) const
{
    for (const auto& [id, item] : world.intersections)
    {
        for (int side = 0; side < IntersectionAnchor::sideCount; ++side)
        {
            if (const auto anchor = PointPlacement(item.AnchorPosition(side), camera, scene))
            {
                const bool active = selection.AnchorHighlighted({id, side});
                renderer.SubmitMesh(Marker(*anchor), active ? EditorColors::amber : EditorColors::blue);
            }
        }
        const bool selected = selection.GeometrySelected({SceneKind::Intersection, id});
        const auto placement = PointPlacement(item.position, camera, scene);
        if (!placement) continue;
        for (const auto& mesh : IntersectionMarker(*placement, item.yaw))
            renderer.SubmitMesh(mesh, selected ? EditorColors::amber : EditorColors::blue);
        if (!selected || item.cornerRadius == 0) continue;

        const OverlayStroke stroke(camera, scene, ViewportFocalLength(scene));
        auto previous = camera.WorldToView(item.position + Vector3{item.cornerRadius, 0, 0});
        for (int i = 1; i <= 64; ++i)
        {
            const double angle = (2.0 * std::numbers::pi / 64) * i;
            const Vector3 offset{item.cornerRadius * std::cos(angle), 0, item.cornerRadius * std::sin(angle)};
            const auto next = camera.WorldToView(item.position + offset);
            stroke.Triangles(previous, next, [&](Vector3 a, Vector3 b, Vector3 c)
            {
                renderer.SubmitTriangle(a, b, c, EditorColors::amber);
            });
            previous = next;
        }
    }
}

RoadPick RoadOverlay::Pick(ViewportPicker& picker, const WorldDocument& world, const Road* selected, const FreeCamera& camera, SDL_Rect scene, bool preferAnchor) const
{
    RoadPick hit;

    // Equal-depth fragments use the last drawn surface. Test in reverse draw
    // order so picking and material drops agree with the depth buffer.
    for (int i = static_cast<int>(world.Objects().size()) - 1; i >= 0; --i)
    {
        if (picker.TestMesh(world.Objects()[i]->Instance())) hit = {RoadPickKind::Object, i};
    }
    for (int i = static_cast<int>(world.Terrain().size()) - 1; i >= 0; --i)
    {
        if (picker.TestMesh(world.Terrain()[i]->Instance())) hit = {RoadPickKind::Terrain, i};
    }
    for (int i = static_cast<int>(world.meshes.size()) - 1; i >= 0; --i)
    {
        if (picker.TestMesh(world.meshes[i])) hit = {RoadPickKind::Mesh, i};
    }
    for (auto it = world.IntersectionSurfaces().rbegin(); it != world.IntersectionSurfaces().rend(); ++it)
    {
        if (picker.TestRoad(it->second.geometry)) hit = {RoadPickKind::Intersection, it->first};
    }
    for (int i = static_cast<int>(world.roads.size()) - 1; i >= 0; --i)
    {
        if (picker.TestRoad(world.roads[i])) hit = {RoadPickKind::Road, i};
    }

    for (const auto& [id, item] : world.intersections)
    {
        if (const auto placement = PointPlacement(item.position, camera, scene))
            for (const auto& mesh : IntersectionMarker(*placement, item.yaw))
                if (picker.TestMesh(mesh)) hit = {RoadPickKind::Intersection, id};
    }
    // A joined endpoint and its anchor share one marker. Shift gives the anchor the tie.
    const auto pickAnchors = [&]
    {
        for (const auto& [id, item] : world.intersections)
        {
            for (int side = 0; side < IntersectionAnchor::sideCount; ++side)
            {
                const auto placement = PointPlacement(item.AnchorPosition(side), camera, scene);
                if (placement && picker.TestMesh(Marker(*placement))) hit = {RoadPickKind::Intersection, id, side};
            }
        }
    };
    if (preferAnchor) pickAnchors();
    if (selected)
    {
        VisitSegmentTriangles(*selected, camera, scene, [&](int i, Vector3 a, Vector3 b, Vector3 c)
        {
            if (picker.TestTriangle(a, b, c))
            {
                hit = {RoadPickKind::Segment, i};
            }
        });
        VisitPoints(*selected, camera, scene, [&](int i, const GizmoPlacement& placement)
        {
            if (picker.TestMesh(Marker(placement)))
            {
                hit = {RoadPickKind::Point, i};
            }
        });
    }

    if (!preferAnchor) pickAnchors();
    return hit;
}
