#include "road_gizmo.hpp"
#include "free_camera.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"
#include "view_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    constexpr std::array<Vector3, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    constexpr std::array<SDL_FColor, 3> colors{{{1, 0.18f, 0.14f, 1}, {0.24f, 0.90f, 0.30f, 1}, {0.22f, 0.48f, 1, 1}}};
    constexpr SDL_FColor controlColor{0.39f, 0.90f, 0.88f, 1}, selectedColor{1, 0.76f, 0.32f, 1};
    constexpr double ringPixels = 56.0;

    double Focal(SDL_Rect scene)
    {
        return 0.5 * std::min(scene.w, scene.h);
    }

    Vector3 Ray(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse)
    {
        const double focal = Focal(scene);
        // Hit the same pixel center sampled by the world rasterizer.
        return camera.ViewRay((mouse.x + 0.5 - scene.w * 0.5) / focal, (scene.h * 0.5 - mouse.y - 0.5) / focal);
    }

    Vector3 DisplayOrigin(Vector3 point, double metersPerPixel)
    {
        return point + Vector3{0, 6 * metersPerPixel, 0};
    }

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

    struct Picker
    {
        Vector3 eye, ray;
        double depth = std::numeric_limits<double>::infinity();
        GizmoPick hit{};

        void Triangle(Vector3 a, Vector3 b, Vector3 c, PickKind kind, int index)
        {
            // Front-facing ray/triangle intersection in world coordinates.
            const Vector3 ab = b - a, ac = c - a;
            const Vector3 p = Math::Cross(ray, ac);
            const double determinant = Math::Dot(ab, p);

            if (determinant <= 0)
            {
                return;
            }

            const Vector3 fromA = eye - a;
            const double u = Math::Dot(fromA, p);
            const Vector3 q = Math::Cross(fromA, ab);
            const double v = Math::Dot(ray, q);

            if (u < 0 || v < 0 || u + v > determinant)
            {
                return;
            }

            const double t = Math::Dot(ac, q) / determinant;

            if (t >= worldNearDepth && t < depth)
            {
                depth = t;
                hit = {kind, index, eye + ray * t};
            }
        }

        void Instance(const MeshInstance& instance, PickKind kind, int index)
        {
            if (!instance.mesh)
            {
                return;
            }

            const Mesh& mesh = *instance.mesh;
            const bool mirrored = instance.transform.Determinant() < 0;

            const auto vertex = [&](std::size_t i)
            {
                return instance.transform.Point(mesh.vertices[mesh.indices[i]].position);
            };

            for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
            {
                Triangle(vertex(i), vertex(i + (mirrored ? 2 : 1)), vertex(i + (mirrored ? 1 : 2)), kind, index);
            }
        }
    };

    template<typename Visit>

    void VisitSegmentTriangles(const Road& road, const FreeCamera& camera, SDL_Rect scene, Visit visit)
    {
        const double focal = Focal(scene);

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
        const Vector3 lift = camera.WorldDirectionToView(DisplayOrigin({}, 1.0));

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
        const double focal = Focal(scene);

        if (focal <= 0)
        {
            return;
        }

        for (int i = 0; i < static_cast<int>(road.points.size()); ++i)
        {
            const Vector3 point = road.points[i].position;
            const double depth = camera.WorldToView(point).z;

            if (depth >= worldNearDepth)
            {
                visit(i, point, depth / focal);
            }
        }
    }
}

void RoadGizmo::Load(MeshAssets& assets)
{
    const auto load = [&](const char* path)
    {
        auto model = assets.Load(path);

        if (!model)
        {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Road gizmo %s unavailable: %s", path, SDL_GetError());
        }

        return model;
    };

    arrow = load("editor/arrow.glb");
    ring = load("editor/rotation_gizmo.glb");

    auto diamond = std::make_shared<Mesh>();

    for (Vector3 axis : axes)
    {
        diamond->vertices.push_back({axis, axis});
        diamond->vertices.push_back({axis * -1, axis * -1});
    }

    diamond->indices = {2, 4, 0, 2, 1, 4, 2, 5, 1, 2, 0, 5, 3, 0, 4, 3, 4, 1, 3, 1, 5, 3, 5, 0};

    marker = std::move(diamond);
}

MeshInstance RoadGizmo::Marker(Vector3 point, double metersPerPixel) const
{
    const double r = 7.0 * metersPerPixel;
    return {marker, {DisplayOrigin(point, metersPerPixel), axes[0] * r, axes[1] * r, axes[2] * r}};
}

MeshInstance RoadGizmo::Handle(Vector3 point, double metersPerPixel, int axis) const
{
    MeshTransform transform;
    transform.origin = DisplayOrigin(point, metersPerPixel);
    // The arrow points along local X. The ring's normal is local Y.
    const int from = mode == GizmoMode::Translate ? 0 : 1;
    std::array<Vector3, 3> basis = axes;
    if (axis != from)
    {
        basis[from] = axes[axis];
        basis[axis] = axes[from] * -1;
    }
    // The authored ring radius is four model units.
    const double scale = ringPixels / 4 * metersPerPixel;
    transform.x = basis[0] * scale;
    transform.y = basis[1] * scale;
    transform.z = basis[2] * scale;
    return {mode == GizmoMode::Translate ? arrow : ring, transform};
}

/**
 * Draw 3D viewport road control points and segment splines.
 */
void RoadGizmo::Draw(WorldRenderer& renderer, const Road& road, int point, const std::set<int>& segments, const FreeCamera& camera, SDL_Rect scene) const
{
    VisitSegmentTriangles(road, camera, scene, [&](int i, Vector3 a, Vector3 b, Vector3 c)
    {
        renderer.SubmitTriangle(a, b, c, segments.contains(i) ? selectedColor : controlColor);
    });

    VisitPoints(road, camera, scene, [&](int i, Vector3 position, double scale)
    {
        const bool selected = i == point;

        renderer.SubmitMesh(Marker(position, scale), selected ? selectedColor : controlColor);

        if (selected)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                renderer.SubmitMesh(Handle(position, scale, axis), colors[axis]);
            }
        }
    });
}

GizmoPick RoadGizmo::Pick(const WorldDocument& world, int road, int point, const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse) const
{
    if (Focal(scene) <= 0)
    {
        return {};
    }

    Picker picker{camera.GetPosition(), Ray(camera, scene, mouse)};

    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
    {
        const Road& candidate = world.roads[i];
        for (const auto* surface : {&candidate.triangles, &candidate.whiteStripes, &candidate.yellowStripes})
        {
            for (const auto& t : *surface)
            {
                picker.Triangle(t[0], t[1], t[2], PickKind::Road, i);
            }
        }
        for (const auto& t : candidate.edgeTriangles)
        {
            picker.Triangle(t[0].position, t[1].position, t[2].position, PickKind::Road, i);
        }
    }

    for (const MeshInstance& instance : world.meshes)
    {
        picker.Instance(instance, PickKind::Empty, -1);
    }

    if (road >= 0)
    {
        VisitSegmentTriangles(world.roads[road], camera, scene, [&](int i, Vector3 a, Vector3 b, Vector3 c)
        {
            picker.Triangle(a, b, c, PickKind::Segment, i);
        });

        VisitPoints(world.roads[road], camera, scene, [&](int i, Vector3 pos, double scale)
        {
            picker.Instance(Marker(pos, scale), PickKind::Point, i);
            if (i == point)
            {
                for (int axis = 0; axis < 3; ++axis)
                {
                    picker.Instance(Handle(pos, scale, axis), PickKind::Axis, axis);
                }
            }
        });
    }

    return picker.hit;
}

GizmoDrag::GizmoDrag(GizmoMode tool, int axisIndex, Vector3 point, Vector3 hit, const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse) : axis(axes[axisIndex]), mode(tool), mouseStart(mouse)
{
    const double metersPerPixel = camera.WorldToView(point).z / Focal(scene);
    origin = DisplayOrigin(point, metersPerPixel);
    const Vector3 eye = camera.GetPosition();
    const Vector3 ray = Ray(camera, scene, mouse);
    const Vector3 view = Math::Normalize(origin - eye);

    normal = mode == GizmoMode::Rotate ? axis : Math::Normalize(view - axis * Math::Dot(view, axis));

    if (std::abs(Math::Dot(normal, Math::Normalize(ray))) > 0.12)
    {
        if (const auto p = OnPlane(eye, ray, origin, normal))
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
    unitsPerPixel = metersPerPixel;

    if (mode == GizmoMode::Rotate)
    {
        const Vector3 radial = hit - origin;
        const Vector3 tangent = camera.WorldDirectionToView(Math::Cross(axis, radial));
        const Vector3 v = camera.WorldToView(hit);
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

std::optional<double> GizmoDrag::Amount(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse) const
{
    if (useScreen)
    {
        return ((mouse.x - mouseStart.x) * screenDirection.x + (mouse.y - mouseStart.y) * screenDirection.y) * unitsPerPixel;
    }

    const auto point = OnPlane(camera.GetPosition(), Ray(camera, scene, mouse), origin, normal);

    if (!point)
    {
        return {};
    }

    const Vector3 current = *point - origin;

    if (mode == GizmoMode::Translate)
    {
        return Math::Dot(current - start, axis);
    }

    return std::atan2(Math::Dot(axis, Math::Cross(start, current)), Math::Dot(start, current));
}
