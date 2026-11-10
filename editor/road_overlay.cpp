#include "road_overlay.hpp"
#include "editor_colors.hpp"
#include "edit_history.hpp"
#include "free_camera.hpp"
#include "view_geometry.hpp"
#include "viewport_picker.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"

#include <cmath>

namespace
{
    constexpr Vector3 markerOffsetPixels{0, 6, 0};
    constexpr float ROAD_SPLINE_THICKNESS = 1.5f;

    template<typename Visit>
    void VisitSegmentTriangles(const Road& road, const FreeCamera& camera, SDL_Rect scene, Visit visit)
    {
        const double focal = ViewportFocalLength(scene);

        if (focal <= 0)
        {
            return;
        }

        const SDL_Rect viewport{0, 0, scene.w, scene.h};
        const auto sides = ViewGeometry::SidePlanes(focal, viewport);
        const Vector3 eye = camera.GetPosition();
        const Vector3 forward = camera.ViewRay(0, 0);
        const Vector3 right = (camera.ViewRay(1, 0) - forward) * (1.0 / focal);
        const Vector3 up = (camera.ViewRay(0, 1) - forward) * (1.0 / focal);
        const Vector3 lift = camera.WorldDirectionToView(markerOffsetPixels);

        const auto display = [&](Vector3 point)
        {
            const Vector3 v = camera.WorldToView(point);
            return v + lift * (v.z / focal); // Match the elevated centers of the point markers.
        };

        for (int i = 0; i + 1 < static_cast<int>(road.points.size()); ++i)
        {
            const auto samples = road.SegmentSamples(i);

            if (samples.empty())
            {
                continue;
            }

            Vector3 a = display(samples.front());

            for (Vector3 sample : samples.subspan(1))
            {
                const Vector3 b = display(sample);

                const auto stroke = ViewGeometry::ProjectStroke(a, b, sides, focal, viewport, worldNearDepth, ROAD_SPLINE_THICKNESS);

                a = b;

                if (!stroke)
                {
                    continue;
                }

                std::array<Vector3, 4> corners;

                for (int j = 0; j < 4; ++j)
                {
                    const SDL_FPoint p = stroke->corners[j];
                    const Vector3 ray = forward + right * (p.x - scene.w * 0.5) + up * (scene.h * 0.5 - p.y);
                    // Bias toward the eye as for road outlines. A vertical span can
                    // still be coplanar with the pavement after the marker lift.
                    const double depth = j < 2 ? stroke->startDepth : stroke->endDepth;
                    corners[j] = eye + ray * std::max(worldNearDepth, depth / 1.0001);
                }

                visit(i, corners[0], corners[1], corners[2]);
                visit(i, corners[0], corners[2], corners[3]);
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

RoadPick RoadOverlay::Pick(ViewportPicker& picker, const WorldDocument& world, const Road* selected, const FreeCamera& camera, SDL_Rect scene) const
{
    RoadPick hit;

    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
    {
        if (picker.TestRoad(world.roads[i]))
        {
            hit = {RoadPickKind::Road, i};
        }
    }

    for (const auto& mesh : world.meshes)
    {
        if (picker.TestMesh(mesh)) hit = {};
    }
    for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
    {
        if (picker.TestMesh(world.Terrain()[i]->Instance())) hit = {RoadPickKind::Terrain, i};
    }
    for (int i = 0; i < static_cast<int>(world.Objects().size()); ++i)
    {
        if (picker.TestMesh(world.Objects()[i]->Instance())) hit = {RoadPickKind::Object, i};
    }

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

    return hit;
}
