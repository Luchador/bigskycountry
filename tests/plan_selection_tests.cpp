#include "../editor/plan_selection.hpp"
#include "world_document.hpp"
#include "prop.hpp"
#include <iostream>
#include <stdexcept>

namespace
{
void Check(bool ok, const char* message)
{ if (!ok) throw std::runtime_error(message); }

std::shared_ptr<Mesh> Triangle(Vector3 a, Vector3 b, Vector3 c)
{
    auto mesh = std::make_shared<Mesh>();
    for (auto p : {a, b, c}) mesh->vertices.push_back({p, {0,1,0}});
    mesh->indices = {0,1,2}; mesh->UpdateBounds(); return mesh;
}

void Rectangles()
{
    const PlanSelectionRect box({2, 100, 2}, {-2, -100, -2});
    Check(box.Contains(Vector3{0, 100000, 0}) && box.Contains(Vector3{2,0,2}), "Direction/elevation/edge inclusion failed");
    Check(!box.Contains(Vector3{3,0,0}), "Outside point included");
    Check(box.Intersects({-10,0,0}, {10,0,0}), "Crossing segment missed");
    Check(!box.Intersects({-10,0,3}, {10,0,3}), "Parallel outside segment included");
    Check(box.Intersects({-100,0,-100}, {100,0,-100}, {0,0,100}), "Box inside a large triangle missed");
    Check(box.Intersects({-100,0,0}, {100,0,0}, {100,0,1}), "Triangle crossing without contained vertices missed");
    Check(!box.Intersects({-10,0,1}, {-10,0,10}, {1,0,10}), "Empty region inside triangle bounds was selected");
    Check(!box.Intersects({10,0,10}, {10,20,10}, {10,40,10}), "Distant vertical triangle selected");
    Check(box.Intersects({0,0,0}, {0,20,0}, {0,40,0}), "Vertical projected triangle missed");

    auto source = Triangle({0,0,0}, {10,0,0}, {0,0,10});
    MeshInstance mesh{source, ObjectTransform{{500000,500,500000}, {0, Math::DegreesToRadians(90), 0}, {-2,1,3}}.ToMeshTransform()};
    const auto bounds = PlanMeshBounds(mesh);
    Check(PlanMeshIntersects(mesh, {bounds.min, bounds.max}), "Mirrored/scaled/rotated mesh missed");
    const auto vertex = mesh.transform.Point({2,0,2});
    Check(PlanMeshIntersects(mesh, {vertex - Vector3{.1,0,.1}, vertex + Vector3{.1,0,.1}}), "Mesh transform lost at large world coordinates");
    mesh = {source, {}};
    Check(!PlanMeshIntersects(mesh, {{8,0,8}, {9,0,9}}), "Mesh bounds false positive");
}

void World()
{
    WorldDocument world(AssetRoot::Project);
    Road road; road.id = world.AllocateRoadId();
    road.points = {{{-100, 80, 0}}, {{100, 80, 0}}}; road.Rebuild(); world.roads.push_back(road);
    road.id = world.AllocateRoadId(); road.style.railroad = RailroadSettings{};
    road.points = {{{-100, -20, 30}}, {{100, -20, 30}}}; road.Rebuild(); world.roads.push_back(road);
    const int junction = world.CreateIntersection({40,0,40});
    Check(junction != 0, "Create unconnected intersection");
    auto model = Triangle({0,0,0}, {10,0,0}, {0,0,10});
    auto prop = std::make_unique<Prop>(); prop->SetMesh(model);
    Check(prop->SetTransform(ObjectTransform{{-40,0,-40}}) && world.InsertObject(0, std::move(prop), 0), "Insert object");
    world.meshes.push_back({model, ObjectTransform{{40,0,-40}}.ToMeshTransform()});
    auto* terrain = world.PlaceTerrain("terrain/test_grid.glb", ObjectTransform{{-40,0,40}});
    Check(terrain != nullptr, "Place imported terrain"); terrain->SetMesh(model);
    Check(world.PlaceLake(LakeDefinition::Circle(12), ObjectTransform{{0,0,-40}}) != nullptr, "Place lake");

    const std::set<SceneKey> expected{{SceneKind::Road,0}, {SceneKind::Road,1}, {SceneKind::Intersection,junction},
        {SceneKind::Object,0}, {SceneKind::Mesh,0}, {SceneKind::Terrain,0}, {SceneKind::Terrain,1}};
    Check(SelectPlanBox(world, {{-120,0,-120}, {120,0,120}}) == expected, "Box omitted a scene kind or included folders");
    Check(SelectPlanBox(world, {{-.1,0,-.1}, {.1,0,.1}}) == std::set<SceneKey>{{SceneKind::Road,0}}, "Small box failed to cross road pavement");
    Check(SelectPlanBox(world, {{-.1,0,29.9}, {.1,0,30.1}}) == std::set<SceneKey>{{SceneKind::Road,1}}, "Railroad embankment missed");
    Check(SelectPlanBox(world, {{-1,0,-41}, {1,0,-39}}) == std::set<SceneKey>{{SceneKind::Terrain,1}}, "Lake surface missed");
    Check(SelectPlanBox(world, {{-1,0,65}, {1,0,70}}).empty(), "Empty area selected geometry");
    const auto anchor = world.intersections.at(junction).AnchorPosition(0);
    Check(SelectPlanBox(world, {anchor - Vector3{.1,0,.1}, anchor + Vector3{.1,0,.1}}).contains({SceneKind::Intersection,junction}),
          "Unconnected intersection anchor missed");

    const int parent = world.hierarchy.Create(0, "Hidden"), child = world.hierarchy.Create(parent, "Nested");
    Check(world.hierarchy.Move({{SceneKind::Road,0}, {SceneKind::Object,0}, {SceneKind::Terrain,1}}, child), "Move hidden fixtures");
    Check(world.hierarchy.SetChildrenHidden(parent, true), "Hide folder");
    auto visible = expected;
    for (auto key : {SceneKey{SceneKind::Road,0}, SceneKey{SceneKind::Object,0}, SceneKey{SceneKind::Terrain,1}}) visible.erase(key);
    Check(SelectPlanBox(world, {{120,0,120}, {-120,0,-120}}) == visible, "Reverse box selected hidden descendants");
    Check(world.hierarchy.SetChildrenHidden(child, false), "Override hidden parent");
    Check(SelectPlanBox(world, {{-120,0,120}, {120,0,-120}}) == expected, "Explicitly unhidden descendants omitted");

    const std::set<SceneKey> old{{SceneKind::Object,0}, {SceneKind::Road,0}}, hits{{SceneKind::Road,0}, {SceneKind::Terrain,0}};
    Check(CombineBoxSelection(old, hits, BoxSelectionMode::Replace) == hits, "Replace mode failed");
    Check(CombineBoxSelection(old, hits, BoxSelectionMode::Add).size() == 3, "Shift add duplicated/dropped selection");
    Check(CombineBoxSelection(old, hits, BoxSelectionMode::Subtract) == std::set<SceneKey>{{SceneKind::Object,0}}, "Ctrl subtract failed");
    Check(CombineBoxSelection(old, {}, BoxSelectionMode::Replace).empty(), "Empty box did not clear selection");
    Check(CombineBoxSelection(old, {}, BoxSelectionMode::Add) == old, "Empty add changed selection");
}
}

int main()
{
    try { Rectangles(); World(); std::cout << "Plan selection tests passed\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << ": " << SDL_GetError() << '\n'; return 1; }
    return 0;
}
