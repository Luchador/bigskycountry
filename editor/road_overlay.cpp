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
    constexpr double strokeDepthBias = 1.0001;
    constexpr float overlayStrokeWidth = 1.5f;

    // Use a constant-pixel-width stroke for splines and intersection guides.
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
        void Triangles(Vector3 viewStart, Vector3 viewEnd, Visit visit, float halfWidth = overlayStrokeWidth,
                       float innerHalfWidth = 0.0f) const
        {
            const auto stroke = ViewGeometry::ProjectStroke(viewStart, viewEnd, sides, focal, viewport,
                                                             worldNearDepth, halfWidth);
            if (!stroke) return;
            std::array<Vector3, 4> corners;
            for (int j = 0; j < 4; ++j)
            {
                const SDL_FPoint p = stroke->corners[j];
                const Vector3 ray = forward + right * (p.x - viewport.w * 0.5) + up * (viewport.h * 0.5 - p.y);
                // Bias toward the eye so coincident pavement cannot hide the stroke.
                const double depth = j < 2 ? stroke->startDepth : stroke->endDepth;
                corners[j] = eye + ray * std::max(worldNearDepth, depth / strokeDepthBias);
            }
            const auto quad = [&](Vector3 a, Vector3 b, Vector3 c, Vector3 d)
            {
                visit(a, b, c);
                visit(a, c, d);
            };
            if (innerHalfWidth == 0.0f)
            {
                quad(corners[0], corners[1], corners[2], corners[3]);
                return;
            }
            // Leave the middle free for the editable spline, even when looking along it.
            const double rim = 0.5 * (1.0 - innerHalfWidth / halfWidth);
            const Vector3 startRim = (corners[1] - corners[0]) * rim;
            const Vector3 endRim = (corners[2] - corners[3]) * rim;
            quad(corners[0], corners[0] + startRim, corners[3] + endRim, corners[3]);
            quad(corners[1] - startRim, corners[1], corners[2], corners[2] - endRim);
        }

        void Draw(WorldRenderer& renderer, Vector3 viewStart, Vector3 viewEnd, SDL_FColor color) const
        {
            Triangles(viewStart, viewEnd, [&](Vector3 a, Vector3 b, Vector3 c) { renderer.SubmitTriangle(a, b, c, color); });
        }
    };

    template<typename Visit>
    void VisitSplineTriangles(const OverlayStroke& stroke, const FreeCamera& camera, std::span<const Vector3> samples,
                              Visit visit, float halfWidth = overlayStrokeWidth, float innerHalfWidth = 0.0f)
    {
        if (samples.size() < 2) return;
        const Vector3 lift = camera.WorldDirectionToView(markerOffsetPixels);
        const auto display = [&](Vector3 point)
        {
            const Vector3 v = camera.WorldToView(point);
            return v + lift * (v.z / stroke.focal); // Match the elevated centers of the point markers.
        };
        Vector3 previous = display(samples.front());
        for (Vector3 sample : samples.subspan(1))
        {
            const Vector3 next = display(sample);
            stroke.Triangles(previous, next, visit, halfWidth, innerHalfWidth);
            previous = next;
        }
    }

    template<typename Visit>
    void VisitSegmentTriangles(const Road& road, const OverlayStroke& stroke, const FreeCamera& camera, Visit visit)
    {
        for (int i = 0; i + 1 < static_cast<int>(road.points.size()); ++i)
        {
            VisitSplineTriangles(stroke, camera, road.SegmentSamples(i),
                                 [&](Vector3 a, Vector3 b, Vector3 c) { visit(i, a, b, c); });
        }
    }

    // Show every pair on the selected highway, and either attached end of a selected ramp.
    template<typename Visit>
    void VisitRampSeams(const WorldDocument& world, const Road& selected, Visit visit)
    {
        const int selectedIndex = static_cast<int>(&selected - world.roads.data());
        for (const auto& [key, seam] : selected.rampSeams) visit(selectedIndex, key, seam);
        if (!selected.style.oneWay || selected.points.size() < 2) return;
        for (const auto* end : {&selected.points.front(), &selected.points.back()})
        {
            if (!end->ramp) continue;
            const auto key = std::pair{end->ramp->station, end->ramp->side};
            for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
            {
                const auto& seams = world.roads[i].rampSeams;
                const auto seam = seams.find(key);
                if (seam != seams.end()) { visit(i, key, seam->second); break; }
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

Vector3 RoadOverlay::SplinePosition(const ViewportPicker& pick, SDL_Rect scene)
{
    // Invert the stroke's depth bias and its depth-scaled vertical lift.
    const double focal = ViewportFocalLength(scene);
    const Vector3 stroke = pick.eye + (pick.HitPosition() - pick.eye) * strokeDepthBias;
    const double scale = pick.camera.WorldToView(stroke).z /
                         (focal + pick.camera.WorldDirectionToView(markerOffsetPixels).z);
    return stroke - markerOffsetPixels * scale;
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

void RoadOverlay::DrawDirection(WorldRenderer& renderer, const Road& ramp, const FreeCamera& camera, SDL_Rect scene) const
{
    const auto samples = ramp.SegmentSamples(0);
    if (samples.size() < 2) return;
    const size_t i = samples.size() / 2;
    const auto mid = (samples[i - 1] + samples[i]) * 0.5;
    const auto forward = Math::Normalize(samples[i] - samples[i - 1]);
    const auto left = Math::Normalize(Math::Cross(forward, {0, 1, 0}));
    const double length = std::min(7.0, ramp.SplineLengthMeters() * 0.25);
    const double focal = ViewportFocalLength(scene);
    if (focal <= 0) return;
    const auto raised = mid + Vector3{0, 0.12, 0};
    const auto tip = camera.WorldToView(raised + forward * (length * 0.5));
    const OverlayStroke stroke(camera, scene, focal);
    for (int side : {-1, 1})
        stroke.Draw(renderer, tip, camera.WorldToView(raised - forward * (length * 0.5) + left * (length * 0.35 * side)), EditorColors::amber);
}

void RoadOverlay::Draw(WorldRenderer& renderer, const Road& road, const RoadEditorState& selection, int roadIndex, const FreeCamera& camera, SDL_Rect scene) const
{
    const double focal = ViewportFocalLength(scene);
    if (focal <= 0) return;
    const OverlayStroke stroke(camera, scene, focal);
    const int laneCount = road.style.GetLaneCount();
    for (size_t lane = 0; lane < road.laneCenterlines.size(); ++lane)
    {
        // An odd road's middle lane shares the editing spline. Leave a green rim around it.
        const bool middle = laneCount % 2 != 0 && lane == static_cast<size_t>(laneCount / 2);
        VisitSplineTriangles(stroke, camera, road.laneCenterlines[lane],
            [&](Vector3 a, Vector3 b, Vector3 c) { renderer.SubmitTriangle(a, b, c, EditorColors::laneGreen); },
            middle ? 2 * overlayStrokeWidth : overlayStrokeWidth, middle ? overlayStrokeWidth : 0.0f);
    }
    if (road.style.oneWay) DrawDirection(renderer, road, camera, scene);
    VisitSegmentTriangles(road, stroke, camera, [&](int i, Vector3 a, Vector3 b, Vector3 c)
    {
        renderer.SubmitTriangle(a, b, c, selection.segments.contains(i) ? EditorColors::amber : EditorColors::blue);
    });

    VisitPoints(road, camera, scene, [&](int i, const GizmoPlacement& placement)
    {
        const bool selected = (roadIndex == selection.selected && selection.PointSelected(i)) ||
                              selection.groupEndpoints.contains({roadIndex, i});
        renderer.SubmitMesh(Marker(placement), selected ? EditorColors::amber : EditorColors::blue);
    });
}

std::optional<std::pair<int, int>> RoadOverlay::PickGroupPoint(ViewportPicker& picker, const WorldDocument& world,
    const RoadEditorState& selection, const FreeCamera& camera, SDL_Rect scene) const
{
    std::optional<std::pair<int, int>> hit;
    for (auto key : selection.SelectedRoadRows())
        VisitPoints(world.roads[key.index], camera, scene, [&](int i, const GizmoPlacement& placement)
        {
            if (picker.TestMesh(Marker(placement))) hit.emplace(key.index, i);
        });
    return hit;
}

void RoadOverlay::DrawRampControls(WorldRenderer& renderer, const WorldDocument& world, const Road& selected,
                                   const FreeCamera& camera, SDL_Rect scene) const
{
    const double focal = ViewportFocalLength(scene);
    if (focal <= 0) return;
    const OverlayStroke stroke(camera, scene, focal);
    std::set<std::pair<int, int>> drawnLanes;
    VisitRampSeams(world, selected, [&](int i, auto key, const RampSeam& seam)
    {
        const auto& host = world.roads[i];
        const int side = key.second;
        if (&host != &selected && drawnLanes.emplace(i, side).second)
            VisitSplineTriangles(stroke, camera, host.OuterLane(side), [&](Vector3 a, Vector3 b, Vector3 c) {
                renderer.SubmitTriangle(a, b, c, EditorColors::laneGreen);
            });
        for (bool taper : {false, true})
        {
            const auto point = host.OuterLanePosition(side, taper ? seam.tipDistance : seam.goreDistance);
            if (const auto placement = PointPlacement(point, camera, scene))
                renderer.SubmitMesh(Marker(*placement), taper ? EditorColors::taperPink : EditorColors::laneGreen);
        }
    });
}

std::optional<RampControlHit> RoadOverlay::PickRampControl(ViewportPicker& picker, const WorldDocument& world,
                                                           const Road& selected, const FreeCamera& camera, SDL_Rect scene) const
{
    const double focal = ViewportFocalLength(scene);
    if (focal <= 0) return {};
    const OverlayStroke stroke(camera, scene, focal);
    std::optional<RampControlHit> hit;
    const int host = static_cast<int>(&selected - world.roads.data());
    const auto pickLane = [&](int i, int side)
    {
        VisitSplineTriangles(stroke, camera, world.roads[i].OuterLane(side), [&](Vector3 a, Vector3 b, Vector3 c) {
            if (picker.TestTriangle(a, b, c)) hit = RampControlHit{RampControl::Lane, i, 0, side};
        });
    };
    for (int side : {-1, 1}) pickLane(host, side);
    std::set<std::pair<int, int>> testedLanes;
    VisitRampSeams(world, selected, [&](int i, auto key, const RampSeam& seam)
    {
        if (i != host && testedLanes.emplace(i, key.second).second) pickLane(i, key.second);
        for (bool taper : {false, true})
        {
            const auto point = world.roads[i].OuterLanePosition(key.second, taper ? seam.tipDistance : seam.goreDistance);
            const auto placement = PointPlacement(point, camera, scene);
            if (placement && picker.TestMesh(Marker(*placement)))
                hit = RampControlHit{taper ? RampControl::Taper : RampControl::Attachment, i, key.first, key.second};
        }
    });
    return hit;
}

std::optional<double> RoadOverlay::LaneDistanceAtPointer(const Road& host, int side, const FreeCamera& camera,
                                                        SDL_Rect scene, SDL_FPoint pointer, double reference)
{
    const auto line = host.OuterLane(side);
    const auto& distances = host.outerLaneDistances[side > 0];
    const double focal = ViewportFocalLength(scene);
    if (line.size() < 2 || focal <= 0) return {};
    const auto lift = camera.WorldDirectionToView(markerOffsetPixels) * (1.0 / focal);
    const auto view = [&](Vector3 p) { const auto v = camera.WorldToView(p); return v + lift * v.z; };
    const SDL_Rect viewport{0, 0, scene.w, scene.h};
    double best = std::numeric_limits<double>::infinity();
    std::optional<double> result;
    auto previous = view(line.front());
    for (size_t i = 1; i < line.size(); ++i)
    {
        const auto next = view(line[i]);
        const auto start = previous;
        auto a = previous, b = next;
        previous = next;
        if (!ViewGeometry::ClipLine(a, b, {0, 0, 1}, worldNearDepth)) continue;
        const auto pa = ViewGeometry::ScreenPoint(a, focal, viewport), pb = ViewGeometry::ScreenPoint(b, focal, viewport);
        const double dx = pb.x - pa.x, dy = pb.y - pa.y;
        const double x = pointer.x + 0.5 - pa.x, y = pointer.y + 0.5 - pa.y;
        if (dx == 0 && dy == 0) continue;
        const double u = std::clamp((x * dx + y * dy) / (dx * dx + dy * dy), 0.0, 1.0);
        const double error = std::hypot(x - dx * u, y - dy * u);
        if (error > best + 0.25) continue;
        // Undo perspective interpolation and near-plane clipping before interpolating meters.
        const auto delta = next - start;
        const auto closest = a + (b - a) * (u * a.z / std::lerp(b.z, a.z, u));
        const double fraction = std::clamp(Math::Dot(closest - start, delta) / Math::Dot(delta, delta), 0.0, 1.0);
        const double distance = std::lerp(distances[i - 1], distances[i], fraction);
        if (result && std::abs(error - best) <= 0.25 && std::abs(distance - reference) >= std::abs(*result - reference)) continue;
        best = error;
        result = distance;
    }
    return result;
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
    const double focal = ViewportFocalLength(scene);
    if (focal <= 0 || world.intersections.empty()) return;
    const bool group = selection.RoadGroupSelected();
    const OverlayStroke stroke(camera, scene, focal);
    for (const auto& [id, item] : world.intersections)
    {
        const auto anchors = item.AnchorPositions();
        const bool selected = selection.GeometrySelected({SceneKind::Intersection, id});
        if (group && selected && !world.IntersectionSurfaces().contains(id))
        {
            // Until the junction has pavement, outline its four authoring anchors.
            auto previous = camera.WorldToView(anchors.back());
            for (Vector3 anchor : anchors)
            {
                const auto next = camera.WorldToView(anchor);
                stroke.Draw(renderer, previous, next, EditorColors::orange);
                previous = next;
            }
        }
        for (int side = 0; side < IntersectionAnchor::sideCount; ++side)
        {
            if (const auto anchor = PointPlacement(anchors[side], camera, scene))
            {
                const bool active = selection.AnchorHighlighted({id, side});
                renderer.SubmitMesh(Marker(*anchor), active ? EditorColors::amber : EditorColors::blue);
            }
        }
        const auto placement = PointPlacement(item.position, camera, scene);
        if (!placement) continue;
        for (const auto& mesh : IntersectionMarker(*placement, item.yaw))
            renderer.SubmitMesh(mesh, selected ? EditorColors::amber : EditorColors::blue);
        if (!selected || item.cornerRadius == 0) continue;

        auto previous = camera.WorldToView(item.position + Vector3{item.cornerRadius, 0, 0});
        for (int i = 1; i <= 64; ++i)
        {
            const double angle = (2.0 * std::numbers::pi / 64) * i;
            const Vector3 offset{item.cornerRadius * std::cos(angle), 0, item.cornerRadius * std::sin(angle)};
            const auto next = camera.WorldToView(item.position + offset);
            stroke.Draw(renderer, previous, next, EditorColors::amber);
            previous = next;
        }
    }
}

RoadPick RoadOverlay::Pick(ViewportPicker& picker, const WorldDocument& world, const Road* selected, const FreeCamera& camera, SDL_Rect scene, bool preferAnchor, Vector3* placement) const
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

    if (placement)
    {
        // World creation uses roads and terrain beneath the click, independently of editor handles.
        if (hit.kind == RoadPickKind::Object || hit.kind == RoadPickKind::Mesh)
        {
            ViewportPicker ground(camera, scene, picker.mouse);
            for (const Road& road : world.roads) ground.TestRoad(road);
            for (const auto& [id, surface] : world.IntersectionSurfaces()) ground.TestRoad(surface.geometry);
            for (const auto& chunk : world.Terrain()) ground.TestMesh(chunk->Instance());
            *placement = ground.PlacementPosition();
        }
        else *placement = picker.PlacementPosition();
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
    const double focal = ViewportFocalLength(scene);
    if (selected && focal > 0)
    {
        const OverlayStroke stroke(camera, scene, focal);
        VisitSegmentTriangles(*selected, stroke, camera, [&](int i, Vector3 a, Vector3 b, Vector3 c)
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
