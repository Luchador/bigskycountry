#include "../editor/viewport_selection.hpp"
#include "../editor/edit_history.hpp"
#include "../editor/selection_bounds.hpp"
#include "prop.hpp"
#include "view_geometry.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void Check(bool ok, const char* message)
{ if (!ok) throw std::runtime_error(message); }
bool Near(Vector3 a, Vector3 b) { return Math::Length(a-b) < 1e-7; }
constexpr SDL_Rect viewport{150, 54, 800, 600};
std::shared_ptr<Mesh> Triangle(Vector3 a, Vector3 b, Vector3 c)
{
    auto mesh = std::make_shared<Mesh>();
    for (auto p : {a,b,c}) mesh->vertices.push_back({p,{0,1,0}});
    mesh->indices = {0,1,2}; mesh->UpdateBounds(); return mesh;
}
SDL_FPoint Screen(const CameraView& camera, Vector3 p)
{ return ViewGeometry::ScreenPoint(camera.WorldToView(p), 300, {0,0,800,600}); }

void PerspectiveGeometry()
{
    CameraView camera;
    const ViewportSelectionBox box(camera, viewport, {430,330}, {370,270});
    Check(box.Contains(Vector3{0,0,10}) && box.Contains(Vector3{1,1,10}), "Point/edge/reversed rectangle failed");
    Check(!box.Contains(Vector3{0,0,-10}) && !box.Contains(Vector3{0,0,.05}), "Behind/near-plane points selected");
    Check(!box.Contains(Vector3{2,0,10}), "Outside point selected");
    Check(box.Intersects({-20,-20,10}, {20,-20,10}, {0,20,10}), "Box inside triangle missed");
    Check(box.Intersects({-20,0,10}, {20,0,10}, {20,.5,10}), "Crossing triangle without contained vertices missed");
    Check(!box.Intersects({-10,1,10}, {-10,10,10}, {1,10,10}), "Triangle bounds false positive");
    Check(!box.Intersects({-20,-20,-10}, {20,-20,-10}, {0,20,-10}), "Behind-camera triangle selected");
    Check(box.Intersects({0,0,-1}, {-2,-2,10}, {2,2,10}), "Triangle crossing near plane missed");
    Check(!box.Intersects({100,100,-1}, {100,100,10}, {110,110,10}), "Outside triangle crossing near plane selected");
    Bounds3 behind; behind.Include({-20,-20,-20}); behind.Include({20,20,-1});
    Check(!box.Intersects(behind), "Behind-camera bounds selected");

    auto mesh = Triangle({-1,-1,0}, {1,-1,0}, {0,1,0});
    MeshInstance instance{mesh, ObjectTransform{{500000,30,-300000},{.1,.4,.2},{-2,1,3}}.ToMeshTransform()};
    camera.LookAt({500000,30,-300020}, instance.transform.origin);
    const auto p = Screen(camera, instance.transform.Point({0,0,0}));
    const ViewportSelectionBox precise(camera, viewport, {p.x-1,p.y-1}, {p.x+1,p.y+1});
    Check(precise.Intersects(instance), "Rotated/scaled/mirrored mesh lost precision at world coordinates");
    Check(precise.Intersects(MeshInstance{nullptr, instance.transform}), "Unloaded mesh origin missed");
}

void WorldSelection()
{
    WorldDocument world(AssetRoot::Project);
    Road road; road.id = world.AllocateRoadId();
    road.points = {{{-100, 0, 0}}, {{100, 0, 0}}}; road.Rebuild(); world.roads.push_back(road);
    road.id = world.AllocateRoadId(); road.style.railroad = RailroadSettings{};
    road.points = {{{-100, 0, 30}}, {{100, 0, 30}}}; road.Rebuild(); world.roads.push_back(road);
    const int junction = world.CreateIntersection({40,0,40});
    auto model = Triangle({0,0,0}, {10,0,0}, {0,0,10});
    auto prop = std::make_unique<Prop>(); prop->SetMesh(model);
    Check(prop->SetTransform(ObjectTransform{{-40,0,-40}}) && world.InsertObject(0,std::move(prop),0), "Insert object");
    world.meshes.push_back({model,ObjectTransform{{40,0,-40}}.ToMeshTransform()});
    auto* terrain = world.PlaceTerrain("terrain/test_grid.glb",ObjectTransform{{-40,0,40}});
    Check(terrain != nullptr,"Place terrain"); terrain->SetMesh(model);
    Check(world.PlaceLake(LakeDefinition::Circle(12),ObjectTransform{{0,0,-40}}) != nullptr,"Place lake");
    Check(world.PlaceRiver(RiverDefinition::Straight(10,20),ObjectTransform{{40,0,70}}) != nullptr,"Place river");
    // Two overlapping objects at different depths must both be included.
    prop = std::make_unique<Prop>(); prop->SetMesh(model);
    Check(prop->SetTransform(ObjectTransform{{-40,-10,-40}}) && world.InsertObject(1,std::move(prop),0),"Insert occluded object");
    CameraView camera; camera.LookAt({0,200,-.001},{0,0,0});
    const ViewportSelectionBox full(camera,viewport,{600,500},{200,100});
    const std::set<SceneKey> expected{{SceneKind::Road,0},{SceneKind::Road,1},{SceneKind::Intersection,junction},
        {SceneKind::Object,0},{SceneKind::Object,1},{SceneKind::Mesh,0},{SceneKind::Terrain,0},{SceneKind::Terrain,1},{SceneKind::Terrain,2}};
    Check(SelectViewportBox(world,full)==expected,"World box omitted scene kind or depth-overlapped object");
    const auto p = Screen(camera,{0,0,0});
    Check(SelectViewportBox(world,{camera,viewport,{p.x-1,p.y-1},{p.x+1,p.y+1}})==std::set<SceneKey>{{SceneKind::Road,0}},
          "Small box crossing road pavement failed");
    const int folder = world.hierarchy.Create(0,"Hidden");
    Check(world.hierarchy.Move({{SceneKind::Road,0},{SceneKind::Object,0},{SceneKind::Terrain,1}},folder) &&
          world.hierarchy.SetChildrenHidden(folder,true),"Hide fixtures");
    auto visible = expected;
    for (auto key : {SceneKey{SceneKind::Road,0},SceneKey{SceneKind::Object,0},SceneKey{SceneKind::Terrain,1}}) visible.erase(key);
    Check(SelectViewportBox(world,full)==visible,"Hidden folder contents selected");
    camera.LookAt({0,200,0},{0,400,0});
    Check(SelectViewportBox(world,{camera,viewport,{0,0},{800,600}}).empty(),"Objects behind camera selected");
}

void WaterGroups()
{
    WorldDocument world(AssetRoot::Project);
    auto river = RiverDefinition::Straight();
    Check(river.InsertPoint(0,0,.5)>=0 && river.InsertPoint(1,0,.5)>=0,"Add bank controls");
    Check(river.MovePoint(1,{-12.5,4,100}),"Set initial slope");
    auto* chunk = world.PlaceRiver(river,ObjectTransform{{500000,20,-300000},{0,.3,0}});
    Check(chunk && chunk->BeginWaterEdit(),"Begin group editing");
    const auto oldMesh = chunk->Instance().mesh, oldWater = chunk->Water().mesh;
    const auto original = *chunk->WaterSource();
    RoadEditorState state;
    state.PickWaterPoint(0,1,false,false); state.PickWaterPoint(0,4,true,false);
    Check(state.meshSelection.SelectedPoints()==std::set<int>{1,4},"Shift did not add control point on other bank");
    state.PickWaterPoint(0,1,false,false);
    Check(state.meshSelection.SelectedPoints()==std::set<int>{1,4},"Click on selected control lost group");
    state.PickWaterPoint(0,1,false,true);
    Check(state.meshSelection.SelectedPoints()==std::set<int>{4},"Ctrl did not remove point");
    state.PickWaterPoint(0,1,true,false);
    auto fields = Inspect(world,state.Target());
    Check(fields.CanEdit(1),"Group elevation disabled");
    Check(Near(fields.value.position,chunk->Instance().transform.Point({0,2,100})),"Group pivot is not center of controls");
    const auto beforeSelection=state.meshSelection;
    EditSnapshot before(state); before.water=EditSnapshot::WaterEdit{0,original};
    auto pose=fields.value; pose.position.y+=7;
    Check(ApplyTransform(world,fields,1,pose),"Group inspector elevation failed");
    Check(chunk->River()->banks[0][1].position.y==11 && chunk->River()->banks[1][1].position.y==7,"Group height differences lost");
    for (int i : {0,2,3,5}) Check(WaterPoint(*chunk->WaterSource(),i)==WaterPoint(original,i),"Unselected bank point moved");
    Check(chunk->Instance().mesh==oldMesh && chunk->Water().mesh==oldWater,"Group edit rebuilt meshes during draft");
    const auto bounds = SelectionBounds(world,state);
    Check(std::abs(bounds.min.y-27)<1e-7,"Focus bounds omitted lower point");
    Check(bounds.max.y==31,"Focus bounds omitted upper point");
    const auto edited=*chunk->WaterSource();
    EditSnapshot after(state); after.water=EditSnapshot::WaterEdit{0,edited};
    EditHistory history; Check(history.Record(-1,before,after),"Record group edit");
    state.SelectRows({});
    Check(history.Undo(world,state)==EditHistory::Result::MapChanged && *chunk->WaterSource()==original && state.meshSelection==beforeSelection,
          "Undo lost group source/selection");
    Check(history.Redo(world,state)==EditHistory::Result::MapChanged && *chunk->WaterSource()==edited && state.meshSelection==beforeSelection,
          "Redo lost group source/selection");
    Check(chunk->SetWater(original),"Restore group fixture");
    WaterDefinition moved=original;
    Check(TranslateWaterPoints(moved,{0,3,6},{0,9,0}),"Move end handle and matching bank points");
    Check(WaterPoint(moved,0).y==9 && WaterPoint(moved,3).y==9,"Overlapping handle moved bank points twice");
    Check(Near(*WaterSelectionCenter(original,{0,6}),{0,0,0}),"End handle pivot counted bank point twice");
    const auto saved=moved;
    Check(!TranslateWaterPoints(moved,{0,999},{0,5,0}) && moved==saved,"Invalid selection partially changed river");
    Check(!TranslateWaterPoints(moved,{0},{0,std::numeric_limits<double>::infinity(),0}) && moved==saved,"Nonfinite group edit mutated river");
    moved=original;
    Check(TranslateWaterPoints(moved,{0,1,2,3,4,5,6,7},{1,2,3}),"Translate complete river");
    for (int bank=0;bank<2;++bank) for (int i=0;i<3;++i)
    {
        const auto& a=std::get<RiverDefinition>(original).banks[bank][i];
        const auto& b=std::get<RiverDefinition>(moved).banks[bank][i];
        Check(b.incoming==a.incoming && b.outgoing==a.outgoing && Near(b.position,a.position+Vector3{1,2,3}),"Rigid group move changed curve handles");
    }
    moved=original;
    Check(!RemoveWaterPoints(moved,{0,1,4}) && moved==original,"Invalid group deletion partially deleted points");
    Check(RemoveWaterPoints(moved,{1,4}) && std::get<RiverDefinition>(moved).banks[0].size()==2 &&
          std::get<RiverDefinition>(moved).banks[1].size()==2,"Multi-bank deletion mishandled flattened indices");
    Check(!RemoveWaterPoints(moved,{4}),"Deleted green end handle");

    // The box queries the draft, including its local elevations and placement yaw.
    moved=original;
    Check(TranslateWaterPoints(moved,{1,4},{0,10,0}) && chunk->SetWater(moved),"Raise draft bank pair");
    CameraView camera; const auto center=Inspect(world,state.Target()).value.position;
    camera.LookAt(center+Vector3{0,150,-.001},center);
    const auto p=Screen(camera,chunk->Instance().transform.Point(WaterPoint(moved,1)));
    const auto q=Screen(camera,chunk->Instance().transform.Point(WaterPoint(moved,4)));
    const ViewportSelectionBox bankBox(camera,viewport,{std::min(p.x,q.x)-3,std::min(p.y,q.y)-3},
                                                     {std::max(p.x,q.x)+3,std::max(p.y,q.y)+3});
    Check(SelectWaterBox(*chunk,bankBox)==std::set<int>{1,4},"Water box did not use transformed draft control points");
    state.SelectWaterPoints(0,SelectWaterBox(*chunk,bankBox));
    Check(state.SelectedRows()==std::set<SceneKey>{{SceneKind::Terrain,0}} && chunk->EditingWater(),"Point box exited edit mode");
    state.SelectWaterPoints(0,{});
    Check(state.meshSelection==TransformTarget{SelectionKind::Terrain,0} && chunk->EditingWater(),"Empty box lost water editing target");
    Check(chunk->FinishWaterEdit() && chunk->Instance().mesh!=oldMesh,"Finish group edit did not rebuild bed");
    Check(SelectWaterBox(*chunk,bankBox).empty(),"Selected water points outside edit mode");

    auto* lake=world.PlaceLake(LakeDefinition::Circle(40)); Check(lake && lake->BeginWaterEdit(),"Begin lake editing");
    state.SelectWaterPoints(1,{0,1}); fields=Inspect(world,state.Target());
    Check(!fields.CanEdit(1) && fields.CanEdit(0),"Lake group changed elevation rules");
    const auto lakeBefore=*lake->WaterSource();
    pose=fields.value; pose.position.x+=2;
    Check(ApplyTransform(world,fields,0,pose),"Lake group translation");
    for (int i=0;i<4;++i)
        Check(Near(WaterPoint(*lake->WaterSource(),i),WaterPoint(lakeBefore,i)+Vector3{i<2 ? 2.0 : 0,0,0}),"Lake group moved wrong controls");
    moved=lakeBefore;
    Check(!RemoveWaterPoints(moved,{0,1}) && moved==lakeBefore,"Lake minimum count deletion was not atomic");
    Check(TranslateWaterPoints(moved,{0,1,2,3},{2,10,3}),"Whole lake translation");
    for (int i=0;i<4;++i)
    {
        const auto& a=std::get<LakeDefinition>(lakeBefore).points[i]; const auto& b=std::get<LakeDefinition>(moved).points[i];
        Check(b.incoming==a.incoming && b.outgoing==a.outgoing && Near(b.position,a.position+Vector3{2,0,3}),"Whole lake distorted shape or left flat plane");
    }
    camera.LookAt({0,100,-.001},{0,0,0});
    Check(SelectWaterBox(*lake,{camera,viewport,{200,100},{600,500}})==std::set<int>{0,1,2,3},"Lake box omitted controls");
}
}
int main()
{
    try { PerspectiveGeometry(); WorldSelection(); WaterGroups(); std::cout << "Viewport selection tests passed\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << ": " << SDL_GetError() << '\n'; return 1; }
}
