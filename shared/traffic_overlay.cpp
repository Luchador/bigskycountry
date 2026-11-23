#include "traffic_overlay.hpp"
#include "camera_view.hpp"
#include "view_geometry.hpp"
#include "spatial_index.hpp"
#include "world_document.hpp"
#include <algorithm>
#include <cmath>

bool DrawTrafficLanes(SDL_Renderer* renderer, const TrafficNetwork& network, const CameraView& camera,
                      SDL_Rect viewport, bool plan, Vector3 center, double scale, std::uint64_t selected,
                      const std::set<int>& hiddenRoads, const TrafficLane* route)
{
    // SDL's viewport already supplies its origin.
    viewport.x = viewport.y = 0;
    const double focal = std::min(viewport.w, viewport.h) * 0.5;
    const auto sides = ViewGeometry::SidePlanes(focal, viewport);
    const auto line = [&](Vector3 a, Vector3 b) {
        SDL_FPoint p, q;
        if (plan)
        {
            p = {float(viewport.w * 0.5 + (a.x - center.x) * scale), float(viewport.h * 0.5 - (a.z - center.z) * scale)};
            q = {float(viewport.w * 0.5 + (b.x - center.x) * scale), float(viewport.h * 0.5 - (b.z - center.z) * scale)};
        }
        else
        {
            a = camera.WorldToView(a + Vector3{0, 0.15, 0}); b = camera.WorldToView(b + Vector3{0, 0.15, 0});
            if (!ViewGeometry::ClipLine(a, b, {0,0,1}, 0.1)) return true;
            for (auto side : sides) if (!ViewGeometry::ClipLine(a, b, side, 0)) return true;
            p = ViewGeometry::ScreenPoint(a, focal, viewport); q = ViewGeometry::ScreenPoint(b, focal, viewport);
        }
        return SDL_RenderLine(renderer, p.x, p.y, q.x, q.y);
    };
    for (const auto& lane : network.lanes)
    {
        if (hiddenRoads.contains(lane.roadId)) continue;
        if (plan)
        {
            const double x = viewport.w * 0.5 / std::max(0.001, scale) + lane.width;
            const double z = viewport.h * 0.5 / std::max(0.001, scale) + lane.width;
            if (lane.bounds.max.x < center.x - x || lane.bounds.min.x > center.x + x ||
                lane.bounds.max.z < center.z - z || lane.bounds.min.z > center.z + z) continue;
        }
        else if (BoundsDistanceSquared(lane.bounds, camera.GetPosition()) > 1000 * 1000) continue;
        if (lane.id == selected) SDL_SetRenderDrawColor(renderer, 255, 185, 40, 255);
        else if (lane.direction > 0) SDL_SetRenderDrawColor(renderer, 70, 240, 145, 255);
        else SDL_SetRenderDrawColor(renderer, 95, 185, 255, 255);
        for (std::size_t i = 1; i < lane.points.size(); ++i)
            if (!line(lane.points[i - 1], lane.points[i])) return false;
        const double spacing = plan ? std::max(20.0, 65.0 / std::max(0.001, scale)) : 20.0;
        for (double d = std::min(5.0, lane.Length() * 0.5); d < lane.Length(); d += spacing)
        {
            const auto at = lane.At(d), forward = lane.Tangent(d);
            const auto right = Math::Normalize(Math::Cross({0,1,0}, forward));
            if (!line(at, at - forward * 2 + right) || !line(at, at - forward * 2 - right)) return false;
        }
        SDL_SetRenderDrawColor(renderer, 255, 105, 85, 255);
        const auto end = lane.At(lane.Length());
        const auto right = Math::Normalize(Math::Cross({0,1,0}, lane.Tangent(lane.Length()))) * (lane.width * 0.45);
        if (!line(end - right, end + right)) return false;
    }
    for (const auto& link : network.links)
    {
        const auto* from=network.FindLane(link.from); const auto* to=network.FindLane(link.to);
        if (!from || !to || hiddenRoads.contains(from->roadId) || hiddenRoads.contains(to->roadId)) continue;
        if (!plan && BoundsDistanceSquared(link.path.bounds,camera.GetPosition()) > 1000*1000) continue;
        if (link.from == selected) SDL_SetRenderDrawColor(renderer,255,185,40,255);
        else if (link.kind == TrafficLinkKind::Merge) SDL_SetRenderDrawColor(renderer,255,145,70,255);
        else SDL_SetRenderDrawColor(renderer,195,125,245,255);
        for (std::size_t i=1; i<link.path.points.size(); ++i)
            if (!line(link.path.points[i-1],link.path.points[i])) return false;
    }
    if (route)
    {
        SDL_SetRenderDrawColor(renderer,255,235,150,255);
        for (std::size_t i=1; i<route->points.size(); ++i)
            if (!line(route->points[i-1],route->points[i])) return false;
    }
    for (const auto& move : network.movements)
    {
        const auto* from = network.FindLane(move.from); const auto* to = network.FindLane(move.to);
        if (!from || !to || hiddenRoads.contains(from->roadId) || hiddenRoads.contains(to->roadId)) continue;
        if (plan)
        {
            const double x = viewport.w*0.5/std::max(0.001,scale), z = viewport.h*0.5/std::max(0.001,scale);
            if (move.path.bounds.max.x < center.x-x || move.path.bounds.min.x > center.x+x ||
                move.path.bounds.max.z < center.z-z || move.path.bounds.min.z > center.z+z) continue;
        }
        else if (BoundsDistanceSquared(move.path.bounds,camera.GetPosition()) > 1000*1000) continue;
        if (move.from == selected) SDL_SetRenderDrawColor(renderer,255,185,40,255);
        else SDL_SetRenderDrawColor(renderer,180,135,230,255);
        for (std::size_t i = 1; i < move.path.points.size(); ++i)
            if (!line(move.path.points[i-1],move.path.points[i])) return false;
        const auto start = move.path.At(0);
        const auto side = Math::Normalize(Math::Cross({0,1,0},move.path.Tangent(0))) * 1.5;
        if (move.control == TrafficControl::Stop) SDL_SetRenderDrawColor(renderer,255,100,85,255);
        else if (move.control == TrafficControl::Yield) SDL_SetRenderDrawColor(renderer,255,210,65,255);
        else SDL_SetRenderDrawColor(renderer,95,235,150,255);
        if (!line(start-side,start+side)) return false;
    }
    return true;
}
bool EditorTrafficOverlay::Draw(SDL_Renderer* renderer, const WorldDocument& world, const CameraView& camera,
                                SDL_Rect viewport, bool plan, Vector3 center, double scale)
{
    std::vector<std::pair<int, std::uint64_t>> current;
    std::set<int> hidden;
    for (std::size_t i = 0; i < world.roads.size(); ++i)
    {
        const auto& road = world.roads[i]; current.emplace_back(road.id, road.revision);
        if (world.hierarchy.EditorHidden({SceneKind::Road, static_cast<int>(i)})) hidden.insert(road.id);
    }
    std::vector<int> currentControls;
    for (const auto& [id,junction] : world.intersections)
    {
        currentControls.push_back(id);
        for (auto control : junction.trafficControls) currentControls.push_back(static_cast<int>(control));
    }
    if (current != revisions || currentControls != controls)
    { network = TrafficNetwork::Build(world); revisions = std::move(current); controls = std::move(currentControls); }
    return DrawTrafficLanes(renderer, network, camera, viewport, plan, center, scale, 0, hidden);
}
