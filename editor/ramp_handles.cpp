#include "road_editor.hpp"
#include "free_camera.hpp"

#include <algorithm>
#include <cmath>

void RoadEditor::PressRampControl(WorldDocument& world, const FreeCamera& camera, RampControlHit hit)
{
    const Road& host = world.roads[hit.host];
    const auto& seam = host.rampSeams.at({hit.station, hit.side});
    const bool taper = hit.control == RampControl::Taper;
    const double start = taper ? seam.tipDistance : seam.goreDistance;
    const auto grab = RoadOverlay::LaneDistanceAtPointer(host, hit.side, camera, scene, mouse, start);
    if (!grab) return;

    // Bound a drag by the road ends and the occupied intervals on this carriageway.
    double low = 0.0, high = host.outerLaneDistances[hit.side > 0].back();
    for (const auto& [key, other] : host.rampSeams)
    {
        if (key.second != hit.side || key.first == hit.station) continue;
        const auto [first, last] = std::minmax(other.goreDistance, other.tipDistance);
        if (other.goreDistance < seam.goreDistance) low = std::max({low, last, other.goreDistance + Road::minPointSpacing});
        else high = std::min({high, first, other.goreDistance - Road::minPointSpacing});
    }
    const double run = seam.tipDistance - seam.goreDistance;
    if (taper)
    {
        if (run < 0) high = std::min(high, seam.goreDistance - Road::minPointSpacing);
        else low = std::max(low, seam.goreDistance + Road::minPointSpacing);
    }
    else
    {
        low -= std::min(0.0, run);
        high -= std::max(0.0, run);
    }
    if (low > high) return;
    const int index = seam.rampIndex;
    if (index < 0 || index >= static_cast<int>(world.roads.size())) return;
    const auto& ramp = world.roads[index];
    if (!ramp.style.oneWay || ramp.points.size() < 2) return;
    for (bool exit : {true, false})
    {
        const auto& link = (exit ? ramp.points.front() : ramp.points.back()).ramp;
        if (!link || link->station != hit.station || link->side != hit.side) continue;
        rampDrag.emplace(RampHandleDrag{hit, index, exit, *link, seam.goreDistance, start - *grab, *grab,
                                        low, high, mouse, false, EditSnapshot(state, &ramp)});
        SDL_CaptureMouse(true);
        return;
    }
}

void RoadEditor::MoveRampControl(WorldDocument& world, const FreeCamera& camera, SDL_FPoint pointer)
{
    auto& drag = *rampDrag;
    const bool atPress = std::hypot(pointer.x - drag.press.x, pointer.y - drag.press.y) < 1.0;
    if (!drag.moved && atPress) return;
    drag.moved = true;
    const auto& host = world.roads[drag.hit.host];
    const auto segment = host.rampSeams.find({drag.hit.station, drag.hit.side});
    if (segment == host.rampSeams.end()) return;
    const auto distance = RoadOverlay::LaneDistanceAtPointer(host, drag.hit.side, camera, scene, pointer, drag.reference);
    if (!distance) return;
    drag.reference = *distance;
    const double value = std::clamp(*distance + drag.grabOffset, drag.low, drag.high);
    auto& ramp = world.roads[drag.ramp];
    auto& link = *(drag.exit ? ramp.points.front() : ramp.points.back()).ramp;
    auto next = drag.original;
    if (!atPress)
    {
        if (drag.hit.control == RampControl::Taper) next.length = std::abs(value - drag.initialGore);
        else
        {
            const auto& seam = segment->second;
            next.offset = value - seam.stationDistance;
            next.length = seam.taperLength;
        }
    }
    if (std::abs(next.offset - link.offset) < 1e-7 && std::abs(next.length - link.length) < 1e-7) return;
    const auto previous = link;
    link = next;
    world.RefreshRampConnections(true);
    const auto& end = drag.exit ? ramp.points.front() : ramp.points.back();
    const auto& neighbor = ramp.points[drag.exit ? 1 : ramp.points.size() - 2];
    if (!ValidRoadSpan(ramp, end.position, neighbor.position))
    {
        link = previous;
        world.RefreshRampConnections(true);
    }
    dragChanged = link != drag.original;
}

void RoadEditor::FinishRampControl(WorldDocument& world, bool commit)
{
    auto drag = std::move(*rampDrag);
    rampDrag.reset();
    dragChanged = false;
    SDL_CaptureMouse(false);
    auto& ramp = world.roads[drag.ramp];
    auto& link = *(drag.exit ? ramp.points.front() : ramp.points.back()).ramp;
    if (link == drag.original) return;
    if (!commit)
    {
        link = drag.original;
        world.RefreshRampConnections(true);
        return;
    }
    if (history.Record(drag.ramp, std::move(drag.before), EditSnapshot(state, &ramp))) Save(world);
}
