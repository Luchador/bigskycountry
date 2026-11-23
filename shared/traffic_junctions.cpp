#include "traffic_junctions.hpp"
#include "game_world.hpp"
#include "car.hpp"
#include <algorithm>
#include <cmath>
#include <set>

bool TrafficJunctions::Conflicts(const TrafficMovement& a, const TrafficMovement& b) const
{
    if (a.junction != b.junction) return false;
    return a.id == b.id || std::binary_search(a.conflicts.begin(), a.conflicts.end(), std::size_t(&b-network.movements.data()));
}
std::size_t TrafficJunctions::Reservations() const
{
    return std::count_if(requests.begin(), requests.end(), [](const auto& item) { return item.second.reserved; });
}
bool TrafficJunctions::ExitClear(const Request& r) const
{
    const auto* lane = network.FindLane(r.move->to);
    if (!lane || lane->Length() < 10) return false;
    Bounds3 bounds;
    for (double d = 0; d <= 12; d += 2) bounds.Include(lane->At(d));
    bounds.min = bounds.min-Vector3{3,1,3}; bounds.max = bounds.max+Vector3{3,3,3};
    for (const auto* pawn : world.NearbyPawns(bounds))
    {
        if (pawn == r.driver->Controlled()) continue;
        const auto p = lane->Project(pawn->GetTransform().position, 0, 16);
        if (std::abs(p.position.y-pawn->GetTransform().position.y) > 2) continue;
        const auto tangent = lane->Tangent(p.distance), side = Math::Normalize(Math::Cross({0,1,0}, tangent));
        const auto frame = pawn->GetTransform().ToMeshTransform();
        const bool car = dynamic_cast<const Car*>(pawn) != nullptr;
        const double halfLength = car ? 2.2*std::abs(Math::Dot(frame.z,tangent))+std::abs(Math::Dot(frame.x,tangent)) : 0.35;
        const double halfWidth = car ? 2.2*std::abs(Math::Dot(frame.z,side))+std::abs(Math::Dot(frame.x,side)) : 0.35;
        const auto delta = frame.origin-p.position;
        const double along = p.distance+Math::Dot(delta,tangent);
        if (std::abs(Math::Dot(delta,side)) < 1.2+halfWidth && along+halfLength > 0 && along-halfLength < 7.8) return false;
    }
    for (double d = 0; d <= 10; d += 2)
        if ((r.driver->collisionReady && !r.driver->collisionReady(lane->At(d))) ||
            world.collision.SphereBlocked(lane->At(d)+Vector3{0,1,0},0.85)) return false;
    return true;
}
bool TrafficJunctions::Intruder(const Request& r) const
{
    auto bounds = r.move->path.bounds;
    bounds.min = bounds.min-Vector3{3,1,3}; bounds.max = bounds.max+Vector3{3,3,3};
    for (const auto* pawn : world.NearbyPawns(bounds, true))
    {
        if (pawn == r.driver->Controlled()) continue;
        // Managed requests obey reservations. The player and unmanaged/parked
        // vehicles never do, so their actual and predicted footprints take priority.
        if (const auto it = requests.find(pawn->Handle()); it != requests.end() &&
            it->second.move->junction == r.move->junction && !it->second.driver->suspended &&
            it->second.driver->Lane().Project(pawn->GetTransform().position, std::max(0.0,it->second.driver->Progress()-5),
                it->second.driver->Progress()+5).error < 1.4) continue;
        for (double t = 0; t <= PawnIndex::predictionSeconds; t += 0.25)
        {
            const auto position = pawn->GetTransform().position + pawn->Velocity()*t;
            const auto p = r.move->path.Project(position);
            const auto forward = r.move->path.Tangent(p.distance);
            const double along = p.distance+Math::Dot(position-p.position,forward);
            if (along < -2.5 || along > r.move->path.Length()+2.5) continue;
            if (p.error < (dynamic_cast<const Car*>(pawn) ? 2.8 : 1.6)) return true;
        }
    }
    return false;
}
void TrafficJunctions::Update(double seconds, const std::vector<std::unique_ptr<TrafficDriver>>& drivers)
{
    clock += seconds;
    std::set<PawnHandle> live;
    for (const auto& driver : drivers)
    {
        driver->PrepareRoute();
        const auto* car = dynamic_cast<const Car*>(driver->Controlled());
        const auto* movement = driver->Movement();
        if (!car || !movement) continue;
        live.insert(car->Handle());
        auto& r = requests[car->Handle()];
        if (r.move != movement) r = {};
        r.driver = driver.get(); r.move = movement;
        r.distance = driver->Entry()-driver->Progress(); r.speed = std::max(0.0,car->Speed());
        r.entered = r.entered || r.distance <= 2.2; // Front bumper has crossed the mouth.
        if (r.distance <= 4.5 && r.speed < 0.2 && !driver->suspended)
        {
            r.stopped += seconds;
            if (r.arrived < 0) r.arrived = clock;
        }
        else r.stopped = 0;
        r.eligible = !driver->suspended && r.distance < std::max(12.0,r.speed*2.0) &&
            (movement->control != TrafficControl::Stop || r.reserved || r.stopped >= 0.6);
        // Keep a granted turn until the rear bumper clears the outgoing mouth.
        // Permission can be withdrawn before entry if its exit becomes blocked.
        driver->JunctionPermission(r.reserved, movement->control == TrafficControl::Stop && r.stopped < 0.6 ? "Stop control" : "Waiting for junction");
    }
    std::erase_if(requests, [&](const auto& item) { return !live.contains(item.first); });
    for (auto& [handle,r] : requests)
    {
        if (r.reserved && r.entered) continue;
        r.exitClear = r.eligible && ExitClear(r);
        if (!r.exitClear || Intruder(r))
        {
            r.reserved = false;
            r.driver->JunctionPermission(false, !r.eligible ? r.move->control == TrafficControl::Stop ? "Stop control" : "Approaching junction" :
                !r.exitClear ? "Blocked exit / waiting for collision" : "Yielding to player / obstruction");
        }
    }
    std::vector<std::pair<PawnHandle, Request*>> order;
    for (auto& [handle,r] : requests) if (r.eligible && r.exitClear && !r.reserved) order.emplace_back(handle,&r);
    std::sort(order.begin(), order.end(), [](const auto& a, const auto& b) {
        const auto& x = *a.second; const auto& y = *b.second;
        if (x.move->control != y.move->control) return x.move->control > y.move->control;
        // Arrival buckets make simultaneous arrivals deterministic without a
        // non-transitive epsilon comparator. Within a tie, through/right first.
        const double tx = x.arrived < 0 ? 1e30 : std::floor(x.arrived/0.2), ty = y.arrived < 0 ? 1e30 : std::floor(y.arrived/0.2);
        if (tx != ty) return tx < ty;
        if ((x.move->turn == TrafficTurn::Left) != (y.move->turn == TrafficTurn::Left)) return x.move->turn != TrafficTurn::Left;
        return a.first < b.first;
    });
    // Equal-time stop arrivals yield to the car on their right. Resolve a full
    // four-way cycle by handle, so symmetric simultaneous arrivals cannot deadlock.
    std::vector<std::pair<PawnHandle, Request*>> sorted;
    while (!order.empty())
    {
        auto chosen = order.begin();
        if (chosen->second->move->control == TrafficControl::Stop)
        {
            const double bucket = std::floor(chosen->second->arrived/0.2);
            for (auto i = order.begin(); i != order.end(); ++i)
            {
                if (i->second->move->control != TrafficControl::Stop || std::floor(i->second->arrived/0.2) != bucket) break;
                bool yields = false;
                for (const auto& [otherHandle,other] : order)
                {
                    if (other == i->second || other->move->control != TrafficControl::Stop ||
                        std::floor(other->arrived/0.2) != bucket || !Conflicts(*i->second->move,*other->move)) continue;
                    const auto a = i->second->move->path.Tangent(0), b = other->move->path.Tangent(0);
                    if (Math::Cross(a,b).y < -0.5 || (Math::Dot(a,b) < -0.5 &&
                        i->second->move->turn == TrafficTurn::Left && other->move->turn != TrafficTurn::Left)) yields = true;
                }
                if (!yields) { chosen = i; break; }
            }
        }
        sorted.push_back(*chosen); order.erase(chosen);
    }
    for (const auto& [handle, request] : sorted)
    {
        auto& r = *request;
        if (Intruder(r)) continue;
        bool wait = false;
        for (const auto& [otherHandle, other] : requests)
        {
            if (otherHandle == handle || !Conflicts(*r.move,*other.move)) continue;
            if (other.reserved || other.entered) { wait = true; break; }
            // Priority traffic is considered while approaching, not only after
            // stopping. Never hold an intersection for a distant/blocked car.
            if (other.move->control > r.move->control && !other.driver->suspended &&
                other.distance < std::max(15.0,other.speed*6.0) && ExitClear(other)) { wait = true; break; }
            // Left turns yield to oncoming through/right traffic of equal rank.
            if (other.move->control == r.move->control && r.move->control != TrafficControl::Stop &&
                r.move->turn == TrafficTurn::Left && other.move->turn != TrafficTurn::Left &&
                Math::Dot(r.move->path.Tangent(0),other.move->path.Tangent(0)) < -0.5 &&
                other.distance < std::max(12.0,other.speed*5.0) && ExitClear(other)) { wait = true; break; }
        }
        if (wait) { r.driver->JunctionPermission(false,"Yielding to conflicting traffic"); continue; }
        r.reserved = true; r.driver->JunctionPermission(true,"Junction reserved");
    }
}
