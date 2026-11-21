#include "../editor/transform_gizmo.hpp"
#include "../editor/viewport_picker.hpp"
#include "../editor/edit_history.hpp"
#include "free_camera.hpp"
#include "marker.hpp"
#include "prop.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
    void Check(bool ok, const char* message)
    { if (!ok) throw std::runtime_error(std::string(message) + ": " + SDL_GetError()); }
    bool Near(double a, double b) { return std::abs(a-b) < 1e-6; }

    SDL_FPoint Screen(const FreeCamera& camera, SDL_Rect viewport, Vector3 p)
    {
        const auto view = camera.WorldToView(p);
        const double focal = ViewportFocalLength(viewport);
        return {float(viewport.w*.5 + view.x/view.z*focal - .5), float(viewport.h*.5 - view.y/view.z*focal - .5)};
    }

    void Motion()
    {
        FreeCamera camera; camera.LookAt({0,0,-10}, {});
        const SDL_FPoint start{400,300};
        const DragScale uniform(camera, {}, {}, start);
        Check(uniform.Amount(start) == 1, "Mouse-down changed scale");
        Check(Near(uniform.Amount({500,300}), 2) && Near(uniform.Amount({400,200}), 2), "Right/up must enlarge uniformly");
        Check(Near(uniform.Amount({300,300}), .5) && Near(uniform.Amount({400,400}), .5), "Left/down must shrink uniformly");
        Check(Near(uniform.Amount({450,250}), 2), "Uniform diagonal drag combined axes incorrectly");
        Check(uniform.Amount({500,400}) == 1, "Opposing uniform motion should cancel");
        Check(uniform.Amount({-1e30f,1e30f}) > 0 && std::isfinite(uniform.Amount({1e30f,-1e30f})), "Long drag collapsed/overflowed scale");
        Check(uniform.Amount({std::numeric_limits<float>::infinity(),0}) == 1, "Non-finite pointer accepted");
        const DragScale x(camera, {}, {1,0,0}, start), y(camera, {}, {0,1,0}, start), z(camera, {}, {0,0,1}, start);
        Check(Near(x.Amount({500,300}),2) && x.Amount({400,200}) == 1, "X drag did not follow screen axis");
        Check(Near(y.Amount({400,200}),2) && y.Amount({500,300}) == 1, "Y drag did not follow screen axis");
        Check(Near(z.Amount({400,200}),2), "Camera-aligned axis needs stable vertical fallback");

        // Rotated objects, off-center pivots and different camera distances.
        const Vector3 pivot{1234,56,-789};
        const auto basis = ObjectTransform{{},{.3,.7,-.2}}.ToMeshTransform();
        for (double distance : {10.0,1000.0})
        {
            camera.LookAt(pivot + Vector3{-distance,.6*distance,-distance}, pivot);
            for (auto axis : {basis.x,basis.y,basis.z})
            {
                const Vector3 origin = pivot + camera.Right()*distance*.3;
                const SDL_Rect viewport{0,0,800,600};
                const auto a = Screen(camera,viewport,origin), b = Screen(camera,viewport,origin+axis*.01);
                const double length = std::hypot(b.x-a.x,b.y-a.y);
                const SDL_FPoint end{a.x+float(100*(b.x-a.x)/length),a.y+float(100*(b.y-a.y)/length)};
                const auto factor = DragScale(camera,origin,axis,a).Amount(end);
                Check(std::abs(factor-2) < .002, "Local axis direction or distance-independent sensitivity incorrect");
            }
            Check(Near(DragScale(camera,pivot,{},start).Amount({500,300}),2), "Uniform scale depends on camera distance");
        }
    }

    void TransformsAndHistory()
    {
        const ObjectTransform start{{100,200,300},{.2,.6,-.1},{2,3,4}};
        for (int axis = 0; axis < 4; ++axis)
        {
            const auto pose = ScaleObjectTransform(start,axis,2);
            Check(pose.position == start.position && pose.rotation == start.rotation, "Scaling moved/rotated object");
            Check(pose.scale.x == (axis == 0 || axis == 3 ? 4 : 2) &&
                  pose.scale.y == (axis == 1 || axis == 3 ? 6 : 3) &&
                  pose.scale.z == (axis == 2 || axis == 3 ? 8 : 4), "Scale changed wrong local component");
        }
        auto mirrored = start; mirrored.scale.x = -2;
        Check(ScaleObjectTransform(mirrored,3,.5).scale == Vector3{-1,1.5,2}, "Uniform scaling lost proportions or mirror sign");
        for (double factor : {0.0,-1.0,std::numeric_limits<double>::infinity()})
            Check(ScaleObjectTransform(start,0,factor) == start, "Invalid scale multiplier accepted");
        Check(ScaleObjectTransform(start,0,1) == start, "Returning to mouse-down did not restore exact original pose");

        WorldDocument world(AssetRoot::Project);
        auto prop = std::make_unique<Prop>(); prop->SetTransform(start);
        Check(world.InsertObject(0,std::move(prop),0), "Insert prop fixture");
        auto marker = std::make_unique<Marker>(); marker->SetTransform(start);
        Check(world.InsertObject(1,std::move(marker),0), "Insert marker fixture");
        for (int i = 0; i < 2; ++i)
        {
            RoadEditorState state; state.SelectRows({{SceneKind::Object,i}});
            EditSnapshot before(state); before.meshTransform = Inspect(world,state.Target()).value;
            const auto pose = ScaleObjectTransform(*before.meshTransform,3,2);
            Check(SetPlacementTransform(world,state.Target(),pose), "Scale object/marker through editor placement path");
            Check(Inspect(world,state.Target()).value.scale == Vector3{4,6,8}, "Inspector did not see new scale");
            EditSnapshot after(state); after.meshTransform = Inspect(world,state.Target()).value;
            EditHistory history; Check(history.Record(-1,before,after), "Record scale gesture");
            Check(history.Undo(world,state) == EditHistory::Result::MapChanged && world.Objects()[i]->GetTransform() == start, "Undo scale");
            Check(history.Redo(world,state) == EditHistory::Result::MapChanged && world.Objects()[i]->GetTransform() == pose, "Redo scale");
            Check(SetPlacementTransform(world,state.Target(),start) && world.Objects()[i]->GetTransform() == start, "Cancel scale");
        }
    }

    // Optional asset integration test: pass --meshes when the user's two gizmo
    // GLBs are installed in assets/models/editor. Core tests need no extra assets.
    void MeshPicking()
    {
        MeshAssets assets(AssetRoot::Project);
        const auto arrow = assets.Load("editor/scale_gizmo.glb"), box = assets.Load("editor/box.glb");
        Check(arrow && box, "Missing supplied scale gizmo meshes");
        TransformGizmo gizmo; gizmo.Load(assets); gizmo.mode = GizmoMode::Scale;
        FreeCamera camera; const SDL_Rect viewport{0,0,800,600};
        const Vector3 pivot{100,20,-50};
        const auto frame = ObjectTransform{{},{.2,.4,.1}}.ToMeshTransform();
        for (double distance : {10.0,1000.0}) for (auto space : {GizmoSpace::World,GizmoSpace::Local})
        {
            camera.LookAt(pivot + Vector3{-distance,.7*distance,-distance},pivot);
            auto placement = *GizmoPlacement::At(pivot,camera,viewport);
            placement.localAxes = {frame.x,frame.y,frame.z}; gizmo.settings.space = space;
            const auto center = Screen(camera,viewport,pivot);
            ViewportPicker centerPick(camera,viewport,center);
            Check(gizmo.PickAxis(centerPick,placement) == 3, "Center box lost picking priority");
            ViewportPicker centerDragPick(camera,viewport,center);
            auto uniform = gizmo.BeginDrag(centerDragPick,placement);
            Check(uniform && uniform->AxisIndex() == 3 && Near(*uniform->Amount(camera,viewport,{center.x+100,center.y},gizmo.settings,false),2), "Center box did not start uniform drag");
            const double modelScale = 14*placement.metersPerPixel;
            const double tip = arrow->bounds.max.x - (arrow->bounds.max.y-arrow->bounds.min.y)*.5;
            for (int i = 0; i < 3; ++i)
            {
                const auto position = Screen(camera,viewport,pivot + placement.localAxes[i]*(tip*modelScale));
                ViewportPicker picker(camera,viewport,position);
                auto drag = gizmo.BeginDrag(picker,placement);
                Check(drag && drag->AxisIndex() == i, "Axis mesh orientation/picking mismatch");
                const double dx = position.x-center.x, dy = position.y-center.y, length = std::hypot(dx,dy);
                const auto factor = drag->Amount(camera,viewport,{position.x+float(100*dx/length),position.y+float(100*dy/length)},gizmo.settings,false);
                Check(factor && Near(*factor,2), "Picked axis scaled against its visual direction");
            }
            ViewportPicker miss(camera,viewport,{0,0});
            Check(!gizmo.BeginDrag(miss,placement), "Empty viewport began scale gesture");
        }
    }
}

int main(int argc, char**)
{
    try
    {
        Motion(); TransformsAndHistory(); if (argc > 1) MeshPicking();
        std::cout << "Scale gizmo tests passed\n"; return 0;
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
