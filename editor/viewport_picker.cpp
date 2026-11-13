#include "viewport_picker.hpp"
#include "free_camera.hpp"
#include "road.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"

#include <algorithm>

double ViewportFocalLength(SDL_Rect scene)
{
    return 0.5 * std::min(scene.w, scene.h);
}

Vector3 ViewportRay(const FreeCamera& camera, SDL_Rect scene, SDL_FPoint mouse)
{
    const double focal = ViewportFocalLength(scene);

    if (focal <= 0)
    {
        return {};
    }

    return camera.ViewRay((mouse.x + 0.5 - scene.w * 0.5) / focal, (scene.h * 0.5 - mouse.y - 0.5) / focal);
}

ViewportPicker::ViewportPicker(const FreeCamera& view, SDL_Rect scene, SDL_FPoint pointer) : camera(view), mouse(pointer), eye(view.GetPosition()), ray(ViewportRay(view, scene, pointer))
{
}

void ViewportPicker::TestPlacementSurfaces(const WorldDocument& world)
{
    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
        if (!world.hierarchy.EditorHidden({SceneKind::Road, i})) TestRoad(world.roads[i]);
    for (const auto& [id, junction] : world.IntersectionSurfaces())
        if (!world.hierarchy.EditorHidden({SceneKind::Intersection, id})) TestRoad(junction.geometry);
    for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
        if (!world.hierarchy.EditorHidden({SceneKind::Terrain, i})) TestMesh(world.Terrain()[i]->Instance());
}

bool ViewportPicker::TestTriangle(Vector3 a, Vector3 b, Vector3 c)
{
    const Vector3 ab = b - a, ac = c - a;
    const Vector3 p = Math::Cross(ray, ac);
    const double determinant = Math::Dot(ab, p);

    if (determinant <= 0)
    {
        return false;
    }

    const Vector3 fromA = eye - a;
    const double u = Math::Dot(fromA, p);
    const Vector3 q = Math::Cross(fromA, ab);
    const double v = Math::Dot(ray, q);

    if (u < 0 || v < 0 || u + v > determinant)
    {
        return false;
    }

    const double t = Math::Dot(ac, q) / determinant;

    if (t >= worldNearDepth && t < depth)
    {
        depth = t;
        return true;
    }

    return false;
}

Vector3 ViewportPicker::PlacementPosition(double modelBaseY) const
{
    auto surface = depth < std::numeric_limits<double>::infinity()
        ? std::optional{HitPosition()} : camera.PickGround(ray, worldNearDepth);
    if (surface)
    {
        surface->y -= modelBaseY;
        return *surface;
    }
    constexpr double fallbackDistance = 20.0;
    return eye + Math::Normalize(ray) * fallbackDistance;
}

bool ViewportPicker::TestRoad(const RoadGeometry& road)
{
    const double previous = depth;
    for (const auto* surface : {&road.whiteStripes, &road.yellowStripes})
    {
        for (const auto& t : *surface)
        {
            TestTriangle(t[0], t[1], t[2]);
        }
    }
    const auto testSurface = [&](const auto& surface)
    {
        for (const auto& t : surface) TestTriangle(t[0].position, t[1].position, t[2].position);
    };
    testSurface(road.pavementTriangles);
    testSurface(road.edgeTriangles);
    for (const auto& run : road.railroadTies)
    {
        const Vector3 offset = run.Center() - eye;
        const double along = Math::Dot(offset, ray) / Math::Dot(ray, ray);
        const auto delta = offset - ray * std::clamp(along, 0.0, depth);
        const double radius = run.Radius();
        if (Math::Dot(delta, delta) > radius * radius) continue;
        for (int i = 0; i < run.count; ++i) TestMesh({RailroadTieMesh(), run.At(i)});
    }
    for (const auto& [material, strip] : road.transitionStrips) testSurface(strip);
    return depth < previous;
}

bool ViewportPicker::TestMesh(const MeshInstance& instance)
{
    if (!instance.mesh)
    {
        return false;
    }

    const Mesh& mesh = *instance.mesh;
    const bool mirrored = instance.transform.Determinant() < 0;

    const auto vertex = [&](std::size_t i)
    {
        return instance.transform.Point(mesh.vertices[mesh.indices[i]].position);
    };

    const double previous = depth;

    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
    {
        TestTriangle(vertex(i), vertex(i + (mirrored ? 2 : 1)), vertex(i + (mirrored ? 1 : 2)));
    }

    return depth < previous;
}
