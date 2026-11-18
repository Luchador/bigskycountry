#include "../editor/edit_history.hpp"
#include "../editor/transform_settings.hpp"
#include "geographic_coordinates.hpp"
#include "prop.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
void Check(bool condition, const char* message)
{ if (!condition) throw std::runtime_error(message); }
bool Near(double a, double b) { return std::abs(a-b)<1e-8; }
bool Near(Vector3 a, Vector3 b) { return Math::Length(a-b)<1e-8; }

void Arithmetic()
{
    const std::pair<const char*, double> cases[] = {
        {"125+10",135}, {"90-5",85}, {"100 + 25 - 5",120}, {"-100-25+5",-120},
        {"-5",-5}, {"+5",5}, {"10--5",15}, {"10 + -5",5}, {"10-+5",5},
        {".5 + .25 - 1.",-.25}, {"1e-3+2e+2",200.001}, {"-2E-2 - -1E+2",99.98},
        {" \t- 2.5 + 5\r\n",2.5}, {"123456.789+0.01",123456.799}, {"1e308-1e308",0}
    };
    for (const auto& [text, expected] : cases)
    {
        const auto actual=ParseTransformExpression(text);
        Check(actual && Near(*actual,expected),text);
    }
    for (const char* text : {"", " ", "-", "1+", "1-", "1e", "1e-", "1e-+2", "1 2", "1+*2", "1+++2", "---2",
                            "1*2", "2/0", "(1+2)", "1,2", "0x10", "5m", "nan", "inf", "1+nan", "-inf+1",
                            "1e309", "1e308+1e308", "1e-9999", "1+2garbage", "+=5"})
        Check(!ParseTransformExpression(text),text);
    Check(!ParseTransformExpression(std::string_view("1\0+2",5)),"Embedded NUL accepted");
    Check(!ParseTransformValue("1+2") && ParseTransformValue("-1e-2")==-.01,"Literal-only parsing changed outside transform inputs");

    const TransformFields fields{{SelectionKind::Object,0},ObjectTransform{{10,20.123456789,30},{.1,.2,.3},{1,2,3}}};
    for (int field=0;field<6;++field)
    {
        const auto result=fields.Parse(field,"100 + 25 - 5"); Check(result.has_value(),"Editable transform rejected expression");
        const TransformFields updated{fields.target,*result};
        Check(Near(updated.DisplayValue(field),120),"Expression has incorrect units");
        for (int other=0;other<9;++other)
            if (other!=field) Check(updated.DisplayValue(other)==fields.DisplayValue(other),"Expression rounded another axis");
    }
    Check(fields.Parse(0,"-5")->position.x==-5,"Negative absolute coordinate became relative subtraction");
    Check(Near(fields.Parse(4,"90-5")->rotation.y,Math::DegreesToRadians(85)),"Rotation expression did not use degrees");
    Check(!fields.Parse(0,"1+broken") && !fields.Parse(6,"1+2"),"Invalid or scale expression was accepted");
    const auto geographic=fields.Parse(TransformFields::geographicLocation,"40+.25, -98.5-1");
    const auto expected=GeographicCoordinates{40.25,-99.5}.ToWorld(fields.value.position.y);
    Check(geographic && expected && Near(geographic->position,*expected),"Geographic expressions changed projection/elevation");
    Check(fields.Parse(TransformFields::geographicLocation,"35.0851266266639, -106.65282545913769").has_value(),"Google Maps paste regression");
    Check(!fields.Parse(TransformFields::geographicLocation,"89+2, -98.5") &&
          !fields.Parse(TransformFields::geographicLocation,"40, -170-20"),"Geographic expression bypassed bounds validation");
    TransformFields lake{{SelectionKind::WaterPoint,0,0},{}}; lake.levelOnly=true;
    Check(!lake.Parse(1,"20+5") && lake.Parse(0,"20+5").has_value(),"Expression bypassed disabled axis");
}

void Snapping()
{
    TransformSettings settings;
    settings.ChangeStep(-100);
    const std::array<double,6> steps{.01,.1,1,10,100,1000};
    const Vector3 source{51526.853,571.125,-4761.191};
    for (double step : steps)
    {
        Check(settings.StepMeters()==step,"Wrong grid step");
        settings.snapEnabled=false;
        const Vector3 offset{2.4*step,0,-1.6*step};
        Check(settings.SnapPlanarOffset(offset)==offset && settings.SnapDistance(-1.6*step)==-1.6*step,"Snap-off changed displacement");
        settings.snapEnabled=true;
        const auto snapped=settings.SnapPlanarOffset(offset);
        Check(Near(snapped,{2*step,0,-2*step}),"Positive/negative planar increments incorrect");
        Check(settings.SnapPlanarOffset({.49*step,0,-.49*step})==Vector3{},"Sub-step extension did not collapse to source");
        Check(Near(settings.SnapPlanarOffset({-.6*step,0,.6*step}),{-step,0,step}),"Reverse extension did not snap");
        Check(Near(settings.SnapDistance(-1.6*step),-2*step),"Gizmo distance snap regression");
        const auto end=source+snapped;
        Check(end.y==source.y && Near(end-source,snapped),"Snapping moved an off-grid source or changed height");
        const double heading=Math::DegreesToRadians(35);
        const auto constrained=settings.SnapPlanarOffset({2.4*step,0,0},heading);
        Check(Near(Math::Length(constrained),2*step) && Near(std::atan2(constrained.z,constrained.x),heading),
              "Distance snap broke constrained anchor heading");
        settings.snapEnabled=false;
        Check(Near(Math::Length(settings.SnapPlanarOffset({2.4*step,0,0},heading)),2.4*step),"Angle-only snapping quantized length");
        settings.snapEnabled=true;
        // Exercise the same generated-road and railroad span rules with every step,
        // including centimeter extensions and non-grid-aligned coordinates.
        for (bool railroad : {false,true}) for (bool start : {false,true})
        {
            Road road; road.style.curved=false;
            if (railroad) road.style.railroad=RailroadSettings{};
            road.points={{{source.x-20,source.y,source.z}}, {source}};
            const auto origin=road.points[start ? 0 : 1].position;
            const auto tip=origin+settings.SnapPlanarOffset({(start ? -1.1 : 1.1)*step,0,0});
            const double spacing=std::min(Road::minPointSpacing,settings.StepMeters()*.5);
            Check(ValidRoadSpan(road,origin,tip,spacing),"Small snapped road/railroad extension rejected");
            Check(!ValidRoadSpan(road,origin,origin,spacing),"Zero-length extension accepted");
            road.points.insert(start ? road.points.begin() : road.points.end(),RoadPoint{tip});
            road.Rebuild();
            Check(!road.pavementTriangles.empty() && Near(Math::Length(tip-origin),step),"Snapped extension lost generated geometry");
        }
        settings.ChangeStep(1);
    }
    settings.ChangeStep(-3); // 1 m again.
    Check(settings.StepMeters()==1 && settings.SnapPlanarOffset({14.4,0,0}).x==14,"Changing step did not update snapped preview");
}

void AppliedHistory()
{
    WorldDocument world(AssetRoot::Project);
    auto prop=std::make_unique<Prop>();
    Check(prop->SetTransform(ObjectTransform{{125,20,30}}) && world.InsertObject(0,std::move(prop),0),"Place transform fixture");
    RoadEditorState state; state.SelectRows({{SceneKind::Object,0}});
    auto fields=Inspect(world,state.Target());
    EditSnapshot before(state); before.meshTransform=fields.value;
    const auto desired=fields.Parse(0,"125+10");
    Check(desired && ApplyTransform(world,fields,0,*desired),"Apply location expression");
    fields=Inspect(world,state.Target());
    const auto turned=fields.Parse(4,"90-5");
    Check(turned && ApplyTransform(world,fields,4,*turned),"Apply rotation expression");
    EditSnapshot after(state); after.meshTransform=Inspect(world,state.Target()).value;
    EditHistory history; Check(history.Record(-1,before,after),"Record transform expression edit");
    Check(history.Undo(world,state)==EditHistory::Result::MapChanged && Inspect(world,state.Target()).value==*before.meshTransform,"Undo arithmetic edit");
    Check(history.Redo(world,state)==EditHistory::Result::MapChanged && Inspect(world,state.Target()).value==*after.meshTransform,"Redo arithmetic edit");
}
}
int main()
{
    try { Arithmetic(); Snapping(); AppliedHistory(); std::cout<<"Transform input tests passed\n"; }
    catch (const std::exception& error) { std::cerr<<error.what()<<": "<<SDL_GetError()<<'\n'; return 1; }
}
