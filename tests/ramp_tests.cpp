#include "highway_ramp.hpp"
#include "world_document.hpp"
#include "road_io.hpp"
#include "terrain_fill.hpp"
#include "../editor/highway_ramp_tool.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace
{
void Check(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}
bool Near(Vector3 a, Vector3 b, double epsilon = 1e-7) { return Math::Length(a - b) < epsilon; }
bool Contains(const Road::Triangle& t, Vector3 p)
{
    const auto cross = [](Vector3 a, Vector3 b) { return a.x * b.z - a.z * b.x; };
    const auto a = cross(t[1] - t[0], p - t[0]);
    const auto b = cross(t[2] - t[1], p - t[1]);
    const auto c = cross(t[0] - t[2], p - t[2]);
    return (a > 1e-9 && b > 1e-9 && c > 1e-9) || (a < -1e-9 && b < -1e-9 && c < -1e-9);
}
template<class Surface> bool Covered(const Surface& surface, Vector3 p)
{
    for (const auto& t : surface) if (Contains({t[0].position, t[1].position, t[2].position}, p)) return true;
    return false;
}
Road Highway(bool curved, bool oneWay, RoadEdge edge)
{
    Road road;
    road.id = 1;
    road.style.SetLaneCount(4);
    road.style.SetDivided(!oneWay);
    road.style.oneWay = oneWay;
    road.style.edge = edge;
    road.style.curved = true;
    road.points = {{{-150, 0, 0}}, {{0, 0, 0}}, {{150, 0, 0}}};
    if (curved) { road.points.front().position = {-150, -3, 15}; road.points.back().position = {150, 5, 25}; }
    road.Rebuild();
    return road;
}
void Joins()
{
    for (bool curved : {false, true}) for (bool oneWay : {false, true})
    for (int side : {-1, 1}) for (bool exit : {false, true})
    for (auto edge : {RoadEdge::None, RoadEdge::Transition, RoadEdge::Curb})
    {
        auto host = Highway(curved, oneWay, edge);
        RoadStyle style;
        style.oneWay = true; style.SetLaneCount(1);
        const auto original = PlanRamp(host, {1, side, 0}, 1, style, exit, 60);
        Check(original.has_value(), "Plan baseline ramp");
        host.style.leftShoulder = host.style.rightShoulder = true;
        host.Rebuild();
        const auto plan = PlanRamp(host, {1, side, 0}, 1, style, exit, 60);
        Check(plan.has_value(), "Plan ramp through shoulder");
        Check(Near(original->seam.center, plan->seam.center), "Shoulder shifted ramp position");
        Check(Near(original->seam.forward, plan->seam.forward), "Shoulder changed ramp departure tangent");
        host.points = plan->hostPoints;
        WorldDocument world;
        world.roads.push_back(host);
        auto ramp = NewRamp(host.style, *plan, exit);
        ramp.id = 2;
        world.roads.push_back(std::move(ramp));
        world.RefreshRampConnections(true);
        auto& highway = world.roads[0];
        auto& branch = world.roads[1];
        const auto& joined = exit ? branch.points.front() : branch.points.back();
        Check(Near(joined.position, original->seam.center), "Refreshing shoulder cut moved ramp");
        Check(!highway.rampShoulderCuts.empty(), "Shoulder opening missing");
        // No shoulder, curb, or blending strip may cut across the branch pavement.
        for (const auto& t : branch.pavementTriangles)
        {
            const auto p = (t[0].position + t[1].position + t[2].position) * (1.0 / 3);
            Check(!Covered(highway.pavementTriangles, p), "Highway shoulder overlaps ramp pavement");
            Check(!Covered(highway.edgeTriangles, p), "Highway curb crosses ramp");
            for (const auto& [material, surface] : highway.transitionStrips)
                Check(!Covered(surface, p), "Highway transition strip crosses ramp");
        }
        if (!curved)
        {
            const double travel = highway.style.HalfSeparation() + 10;
            const int away = highway.style.TravelDirection(side) * (exit ? 1 : -1);
            for (double x : {0.17, 1.63, 5.29, 12.43, 25.71, 50.13})
                Check(Covered(highway.pavementTriangles, {x * away, 0, side * (travel - 0.37)}), "Shoulder cut removed through lane");
            Check(Covered(highway.pavementTriangles, {100.13 * away, 0, side * (travel + 4.37)}), "Distant shoulder not restored");
        }
        const auto points = branch.points;
        const auto count = highway.pavementTriangles.size();
        const auto revision = highway.revision;
        for (int i = 0; i < 5; ++i) world.RefreshRampConnections(true);
        Check(branch.points == points && highway.pavementTriangles.size() == count && highway.revision == revision,
              "Unchanged ramp regenerated or accumulated shoulder geometry");
        // Segment overrides must behave just like whole-highway shoulder toggles.
        highway.SetSegmentShoulders({0}, side > 0, false);
        highway.Rebuild();
        world.RefreshRampConnections(true);
        Check(Near(joined.position, original->seam.center), "Per-segment shoulder shifted ramp");
        // Moving the branch updates the opening; deleting it restores the shoulder.
        auto& free = exit ? branch.points.back() : branch.points.front();
        free.position.z += side * 7;
        branch.Rebuild();
        world.RefreshRampConnections(true);
        Check(highway.rampShoulderCuts.front().outline == branch.outline, "Ramp edit left stale shoulder opening");
        world.roads.pop_back();
        world.RefreshRampConnections(true);
        Check(highway.rampShoulderCuts.empty() && highway.rampLanes.empty(), "Deleting ramp left shoulder cut behind");
    }
}

void TerrainAndReload()
{
    WorldDocument world;
    auto host = Highway(false, false, RoadEdge::Transition);
    host.style.leftShoulder = host.style.rightShoulder = true;
    world.roads.push_back(host);
    RoadStyle style; style.oneWay = true; style.SetLaneCount(1);
    for (bool exit : {true, false})
    {
        auto& highway = world.roads[0];
        const auto site = PickRampSite(highway, {exit ? -70.0 : 70.0, 0, 0}, -1);
        Check(site.has_value(), "Find ramp site for terrain enclosure");
        const auto plan = PlanRamp(highway, *site, exit ? 1 : 2, style, exit, 60);
        Check(plan.has_value(), "Plan terrain enclosure ramp");
        highway.points = plan->hostPoints;
        auto ramp = NewRamp(highway.style, *plan, exit);
        ramp.id = exit ? 2 : 3;
        (exit ? ramp.points.back() : ramp.points.front()).connectorSocket = exit ? 3 : 4;
        world.roads.push_back(std::move(ramp));
        world.RefreshRampConnections(true);
    }
    Road connector;
    connector.id = 4; connector.style = style;
    connector.points = {world.roads[1].points.back(), world.roads[2].points.front()};
    for (auto& point : connector.points) { point.connectorTarget = point.connectorSocket; point.connectorSocket = 0; }
    connector.Rebuild();
    world.roads.push_back(std::move(connector));
    world.RefreshRampConnections(true);
    const auto fill = [](const WorldDocument& w)
    {
        std::vector<const RoadGeometry*> perimeter;
        for (const auto& road : w.roads) perimeter.push_back(&road);
        return BuildTerrainFill(perimeter, {});
    };
    const auto terrain = fill(world);
    Check(terrain && !terrain->indices.empty(), "Shoulder openings broke enclosed terrain fill");
    std::stringstream data;
    data.precision(17);
    for (const auto& road : world.roads) RoadIO::WriteRoad(data, road);
    Check(bool(data), "Save ramps with shoulder openings");
    WorldDocument loaded;
    for (size_t i = 0; i < world.roads.size(); ++i)
    {
        auto road = RoadIO::ReadRoad(data);
        Check(road.rampShoulderCuts.empty(), "Generated shoulder cut was serialized");
        road.Rebuild();
        loaded.roads.push_back(std::move(road));
    }
    Check(bool(data), "Reload ramps with shoulder openings");
    loaded.RefreshRampConnections(true);
    Check(loaded.roads[0].outline == world.roads[0].outline, "Reload changed shoulder perimeter");
    const auto restored = fill(loaded);
    Check(restored && restored->indices.size() == terrain->indices.size(), "Reload changed terrain enclosure");
}
}
int main()
{
    try { Joins(); TerrainAndReload(); std::cout << "Ramp shoulder tests passed\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
