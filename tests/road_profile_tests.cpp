#include "road.hpp"
#include "road_io.hpp"
#include "../editor/properties_inspector.hpp"
#include "../editor/edit_history.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace
{
void Check(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}
Road Example(bool reverse = false)
{
    Road road;
    road.style.SetLaneCount(reverse ? 4 : 5);
    road.style.SetDivided(reverse);
    road.style.SetDividedGap(5);
    for (int i = 0; i <= 4; ++i) road.points.push_back({{i * 100.0, 0, 0}});
    if (reverse) Check(road.SetSegmentDivision({2, 3}, {false, 5, RoadMedian::Grass}), "Disable division before adding turn lane");
    Check(road.SetSegmentLaneCounts({2, 3}, reverse ? 5 : 4), "Set segment lanes");
    if (!reverse) Check(road.SetSegmentDivision({2, 3}, {true, 5, RoadMedian::Grass}), "Enable selected segments");
    road.Rebuild();
    return road;
}
bool Contains(const Road::Triangle& triangle, double x, double z)
{
    const Vector3 p{x, 0, z};
    const auto sign = [&](int i, int j) {
        const auto a = triangle[j] - triangle[i], b = p - triangle[i];
        return a.x * b.z - a.z * b.x;
    };
    const auto a = sign(0, 1), b = sign(1, 2), c = sign(2, 0);
    const auto area = Math::Cross(triangle[1] - triangle[0], triangle[2] - triangle[0]).y;
    if (std::abs(area) < 1e-12) return false;
    return (a >= -1e-8 && b >= -1e-8 && c >= -1e-8) || (a <= 1e-8 && b <= 1e-8 && c <= 1e-8);
}
bool Painted(const std::vector<Road::Triangle>& triangles, double x, double z)
{
    for (const auto& t : triangles) if (Contains(t, x, z)) return true;
    return false;
}
double Mix(const Road& road, std::size_t segment, double t, bool median)
{
    const auto b = road.SegmentSurfaceBlend(segment, t);
    const auto a = road.SurfaceStyle(b.from), c = road.SurfaceStyle(b.to);
    const auto width = [&](const RoadStyle& s) {
        return median ? s.HalfMedianWidth() : s.HalfSeparation() + s.GetLaneCount() * s.TravelLaneWidth() * 0.5;
    };
    return std::lerp(width(a), width(c), b.mix);
}
void Geometry()
{
    for (bool reverse : {false, true}) for (auto edge : {RoadEdge::None, RoadEdge::Transition, RoadEdge::Curb})
    {
        auto road = Example(reverse);
        road.style.edge = edge;
        road.Rebuild();
        // With a 5 m divide, the median replaces exactly one turn lane. The two
        // lanes each way stay 5 m wide, and their dashed divider stays at +/-7.5 m.
        for (int i = 1; i < 400; ++i)
        {
            const double x = i + 0.37;
            const auto segment = static_cast<std::size_t>(x / 100);
            Check(std::abs(Mix(road, segment, std::fmod(x, 100) / 100, false) - 12.5) < 1e-9,
                  "Turn-lane/median conversion pinched the through lanes");
            const bool dash = std::fmod(x, RoadStyle::dashPeriod) < 3;
            for (double side : {-1.0, 1.0})
                Check(Painted(road.whiteStripes, x, side * 7.5) == dash, "White dash lost continuity or phase");
        }
        for (double x = 5; x < 400; x += 11.23) for (double z = -12.4; z < 12.5; z += 0.71)
        {
            bool covered = false;
            for (const auto& t : road.pavementTriangles)
                covered |= Contains({t[0].position, t[1].position, t[2].position}, x, z);
            for (const auto& t : road.edgeTriangles)
                covered |= Contains({t[0].position, t[1].position, t[2].position}, x, z);
            Check(covered, "Hole between pavement and changing median");
        }
        for (double x = 1; x < 400; x += 3.17)
        {
            const auto segment = static_cast<std::size_t>(x / 100);
            const double median = Mix(road, segment, std::fmod(x, 100) / 100, true);
            if (median > 0.1)
                Check(!Painted(road.yellowStripes, x, median * 0.5) && !Painted(road.whiteStripes, x, median * 0.5),
                      "Paint appeared on the median");
        }
        Check(road.EndpointStyle(true).IsDivided() == reverse && road.EndpointStyle(false).IsDivided() != reverse,
              "Intersection mouths did not inherit endpoint division");
    }
    auto ordinary = Example();
    ordinary.SetDivision({false, 5, RoadMedian::Concrete});
    ordinary.Rebuild();
    for (const auto& t : ordinary.whiteStripes)
    {
        const auto center = (t[0] + t[1] + t[2]) * (1.0 / 3);
        Check(!(center.x > 101 && center.x < 199 && std::abs(center.z) < 8.0),
              "Ordinary lane-count taper regained dashed lines");
    }
    // A division-only edit must also trigger sampling on a straight spline.
    Road division;
    division.style.SetLaneCount(4);
    division.style.curved = false;
    for (int i = 0; i <= 3; ++i) division.points.push_back({{100.0 * i, 0, 0}});
    division.SetSegmentDivision({2}, {true, 8, RoadMedian::Concrete});
    division.Rebuild();
    Check(division.SegmentSamples(1).size() > 20 && Mix(division, 1, 0.5, true) > 0,
          "Division-only taper was not subdivided");
}
void Persistence()
{
    auto road = Example();
    std::stringstream data;
    data << std::setprecision(17);
    RoadIO::WriteRoad(data, road);
    Check(bool(data), "Save mixed divided road");
    auto loaded = RoadIO::ReadRoad(data);
    Check(bool(data) && loaded.points == road.points && loaded.style == road.style, "Division failed round trip");
    loaded.Rebuild();
    Check(loaded.whiteStripes == road.whiteStripes, "Reload changed paint continuity");
    auto before = road.Section(0, 1), after = road.Section(1, 4);
    before.Rebuild(); after.Rebuild();
    for (double t : {0.0, 0.25, 0.5, 0.75, 1.0})
    {
        Check(std::abs(Mix(after, 0, t, false) - Mix(road, 1, t, false)) < 1e-9, "Split lost pavement context");
        Check(std::abs(Mix(after, 0, t, true) - Mix(road, 1, t, true)) < 1e-9, "Split lost median context");
    }
    Check(loaded.SubdivideSegments({2}) && loaded.points[2].segmentDivision == loaded.points[3].segmentDivision,
          "Subdivision lost division settings");
    auto cut = road.Section(0, 2);
    std::stringstream split;
    split << std::setprecision(17);
    RoadIO::WriteRoad(split, cut);
    auto restored = RoadIO::ReadRoad(split);
    Check(bool(split) && restored.divisionNeighbors == cut.divisionNeighbors &&
          restored.divisionNeighbors[1].has_value(), "Split context did not survive save/load");
    Check(!road.SetSegmentDivision({0}, {true, 5, RoadMedian::Grass}), "Odd-lane segment accepted division");
    Check(!road.SetSegmentLaneCounts({2}, 5), "Divided segment accepted odd lanes");
    Check(!road.SetSegmentDivision({1}, {true, 1, RoadMedian::Grass}), "Accepted separation below 2 m");
    auto invalid = road;
    invalid.points[2].segmentLanes = 5;
    std::stringstream bad;
    RoadIO::WriteRoad(bad, invalid);
    Check(!bad, "Invalid divided road was serialized");

    // Version 23 omitted division neighbors and the optional field at each point.
    Road legacy;
    legacy.points = {{{0, 0, 0}}, {{50, 0, 0}}};
    std::stringstream modern;
    modern << std::setprecision(17);
    RoadIO::WriteRoad(modern, legacy);
    std::stringstream old;
    std::string line;
    for (int i = 0; std::getline(modern, line); ++i)
    {
        if (i == 1) line.resize(line.size() - 4); // Two absent neighbor overrides.
        if (i >= 5) line.resize(line.size() - 2); // One absent point override.
        old << line << '\n';
    }
    const auto imported = RoadIO::ReadRoad(old, 23);
    Check(bool(old) && imported.points == legacy.points && !imported.HasDivisionOverrides(), "Legacy road compatibility");
}
void InspectorAndHistory()
{
    WorldDocument world;
    world.roads.push_back(Example());
    RoadEditorState state;
    state.selected = 0;
    state.segments = {2};
    auto p = InspectProperties(world, state, false);
    Check(p.segmentSelection && p.canDivide && p.AllDivided() && p.minLanes == 4 && p.minGap == 5,
          "Even-lane segment could not use division in an odd-lane road");
    state.segments = {0, 2};
    p = InspectProperties(world, state, false);
    Check(!p.canDivide && p.anyDivided && p.Mixed(RoadProperty::Divided), "Mixed segment division state");
    state.segments = {2};
    EditSnapshot before(state, &world.roads[0]);
    p = InspectProperties(world, state, false);
    auto local = world.roads[0].SegmentStyle(2);
    p.ApplyRoadStyle(local, RoadAction::Divide);
    Check(world.roads[0].SetSegmentDivision(state.segments, local.Division()), "Inspector division toggle");
    world.roads[0].Rebuild();
    Check(!world.roads[0].SegmentStyle(2).IsDivided() && world.roads[0].SegmentStyle(3).IsDivided() &&
          world.roads[0].style.GetLaneCount() == 5, "Segment edit affected unrelated segments or default");
    EditHistory history;
    Check(history.Record(0, std::move(before), EditSnapshot(state, &world.roads[0])), "Record division edit");
    Check(history.Undo(world, state) == EditHistory::Result::MapChanged && world.roads[0].SegmentStyle(2).IsDivided(),
          "Undo did not restore segment division");
    Check(history.Redo(world, state) == EditHistory::Result::MapChanged && !world.roads[0].SegmentStyle(2).IsDivided(),
          "Redo did not restore segment division");
    const RoadProperties properties(world.roads[0]);
    world.roads[0].SetDivision({false, 4, RoadMedian::Concrete});
    properties.Apply(world.roads[0]);
    Check(RoadProperties(world.roads[0]) == properties, "Group property restoration lost segment division");
    p = InspectProperties(world, state, false);
    Check(p.StepLanes(true) == 5, "Undivided segment could not regain its turn lane");
    world.roads[0].SetLaneCount(5);
    std::stringstream saved;
    RoadIO::WriteRoad(saved, world.roads[0]);
    Check(bool(saved) && !world.roads[0].SegmentStyle(3).IsDivided(), "Whole-road odd lane edit left invalid division");
}
}
int main()
{
    try { Geometry(); Persistence(); InspectorAndHistory(); std::cout << "Road profile tests passed\n"; }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
