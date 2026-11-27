#include "traffic_driver.hpp"
#include "game_world.hpp"
#include "car.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace
{
    void Append(TrafficLane& lane, Vector3 p, double width)
    {
        if (lane.points.empty() || Math::Length(lane.points.back()-p) > 1e-8)
        { lane.points.push_back(p); lane.widths.push_back(width); }
    }
    void Slice(TrafficLane& lane, const TrafficLane& path, double from, double to)
    {
        Append(lane,path.At(from),path.WidthAt(from));
        auto i = std::upper_bound(path.distances.begin(),path.distances.end(),from);
        for (; i != path.distances.end() && *i < to; ++i) Append(lane,path.points[i-path.distances.begin()],path.WidthAt(*i));
        Append(lane,path.At(to),path.WidthAt(to));
    }
}

void TrafficDriver::JoinRoute(const TrafficLane& path, const TrafficLane& to, double start, double end)
{
    maneuver = path; destination = &to; destinationStation = end;
    laneWaitDistance = -1;
    entry = start; exit = start+path.Length();
    lane = *source; lane.points.clear(); lane.widths.clear();
    Slice(lane,*source,0,start);
    Slice(lane,path,0,path.Length());
    Slice(lane,to,end,to.Length());
    lane.speedLimit = std::min(source->speedLimit,to.speedLimit);
    lane.Measure();
}

void TrafficDriver::PlanRoute()
{
    if (goalLane || !network || !Controlled()) return;
    const double horizon = 400;
    // Decide an exit before reaching its taper. Different test cars make stable
    // local route choices; no frame-by-frame random decisions or last-second dives.
    std::optional<std::size_t> nextExit;
    double nearest = horizon;
    for (auto index : network->Family(*source))
    {
        const auto& candidate = network->lanes[index];
        for (auto connection : network->Connections(candidate.id))
        {
            const auto& link = network->links[connection];
            if (link.kind != TrafficLinkKind::Exit || skippedExits.contains(connection)) continue;
            const auto p = source->Project(link.path.points.front(),progress,progress+horizon);
            const double ahead = p.distance-progress;
            if (ahead < 15 || ahead >= nearest || std::abs(p.position.y-link.path.points.front().y) > 2) continue;
            nearest = ahead; nextExit = connection;
        }
    }
    if (nextExit)
    {
        const auto& link = network->links[*nextExit];
        if ((Controlled()->Handle()+visits)%4 == 0)
        { goalLink = nextExit; goalLane = link.from; return; }
        // Skip all lanes of this exit, not just its first available ramp lane.
        const auto* branch = network->FindLane(link.to);
        for (std::size_t i=0; i<network->links.size(); ++i)
            if (network->links[i].kind == TrafficLinkKind::Exit &&
                network->FindLane(network->links[i].to)->roadId == branch->roadId) skippedExits.insert(i);
    }
    if (source->Length()-progress > horizon) return;
    // Mandatory continuation/entrance links may be on another lane of a ramp.
    double nearestLane = 1e30;
    for (auto index : network->Family(*source))
    {
        const auto& candidate = network->lanes[index];
        for (auto connection : network->Connections(candidate.id))
        {
            const auto& link = network->links[connection];
            if (link.kind == TrafficLinkKind::Exit) continue;
            const auto p = candidate.Project(source->At(progress));
            if (p.error > 50 || p.distance >= link.fromDistance-5 ||
                Math::Dot(source->Tangent(progress),candidate.Tangent(p.distance)) < 0.9) continue;
            const double rank = std::abs(candidate.ordinal-source->ordinal);
            if (rank >= nearestLane) continue;
            nearestLane = rank; goalLane = candidate.id; goalLink = connection;
        }
    }
    if (goalLane) return;
    // Choose the turn first, then get to its legal approach lane. Duplicated
    // through movements from parallel lanes represent one route choice.
    std::map<std::pair<int,int>,std::size_t> turns;
    for (auto index : network->Family(*source))
    {
        const auto& candidate = network->lanes[index];
        const auto p = candidate.Project(source->At(progress));
        if (p.error > 50 || candidate.Length()-p.distance < 10 ||
            Math::Dot(source->Tangent(progress),candidate.Tangent(p.distance)) < 0.9) continue;
        for (auto choice : network->Outgoing(candidate.id))
        {
            const auto& move = network->movements[choice];
            const auto key = std::pair{move.junction,move.toSide};
            const auto i = turns.find(key);
            if (i == turns.end() || std::abs(candidate.ordinal-source->ordinal) <
                std::abs(network->FindLane(network->movements[i->second].from)->ordinal-source->ordinal)) turns[key] = choice;
        }
    }
    if (!turns.empty())
    {
        auto chosen = turns.begin();
        std::advance(chosen,static_cast<std::ptrdiff_t>((Controlled()->Handle()+visits)%turns.size()));
        goalMovement = chosen->second; goalLane = network->movements[*goalMovement].from;
    }
}

bool TrafficDriver::ClaimsMerge() const
{
    return changing || (highwayLink && network->links[*highwayLink].kind == TrafficLinkKind::Merge && mergeGranted);
}
bool TrafficDriver::PathReady(const TrafficLane& path) const
{
    for (double d=0; d<=path.Length(); d+=2)
    {
        const auto p=path.At(d);
        if ((collisionReady && !collisionReady(p)) || world.collision.SphereBlocked(p+Vector3{0,1,0},0.85)) return false;
    }
    return !collisionReady || collisionReady(path.points.back());
}

bool TrafficDriver::GapAvailable(const TrafficLane& path, const TrafficLane& to, double toDistance) const
{
    const auto* car = dynamic_cast<const Car*>(Controlled());
    if (!car || !PathReady(path)) return false;
    const double speed = std::max(0.0,car->Speed());
    const double duration = std::clamp(path.Length()/std::max(4.0,speed),1.0,8.0);
    const auto current = to.Project(car->GetTransform().position,
        std::max(0.0,toDistance-path.Length()-30),std::min(to.Length(),toDistance+20));
    auto bounds = path.bounds;
    for (double d=std::max(0.0,current.distance-180); d< std::min(to.Length(),toDistance+140); d+=8) bounds.Include(to.At(d));
    bounds.Include(to.At(std::min(to.Length(),toDistance+140)));
    bounds.min=bounds.min-Vector3{4,2,4}; bounds.max=bounds.max+Vector3{4,3,4};
    for (const auto* other : world.NearbyPawns(bounds,true))
    {
        if (other == car || !other->hasCollision) continue;
        const auto* driver=dynamic_cast<const TrafficDriver*>(other->Controller());
        const auto* otherCar=dynamic_cast<const Car*>(other);
        const bool frozen=other->Controller() && !other->Controller()->SimulationReady();
        const auto velocity=frozen ? Vector3{} : other->Velocity();
        const auto pose=other->GetTransform().ToMeshTransform();
        const auto p=to.Project(pose.origin,std::max(0.0,current.distance-180),toDistance+150);
        const auto forward=to.Tangent(p.distance), side=Math::Normalize(Math::Cross({0,1,0},forward));
        const double width=otherCar ? std::abs(Math::Dot(pose.x,side))+2.2*std::abs(Math::Dot(pose.z,side)) : 0.35;
        const double halfLength=otherCar ? 2.2*std::abs(Math::Dot(pose.z,forward))+std::abs(Math::Dot(pose.x,forward)) : 0.35;
        if (std::abs(pose.origin.y-p.position.y) < 2.0 && std::abs(Math::Dot(pose.origin-p.position,side)) < width+1.2)
        {
            const double station=p.distance+Math::Dot(pose.origin-p.position,forward);
            const double otherSpeed=std::max(0.0,Math::Dot(velocity,forward));
            const double now=station-current.distance, later=station+otherSpeed*duration-toDistance;
            const double extent=halfLength+2.2;
            const double front=extent+following.standstillGap+speed*1.0;
            const double rear=extent+following.standstillGap+otherSpeed*1.0+std::max(0.0,otherSpeed-speed)*2.0;
            // Preserve ordering throughout the maneuver. A fast approaching rear
            // car must not be forced into severe braking by our lane change.
            if ((now >= 0 && (now < front || later < front)) || (now < 0 && (-now < rear || -later < rear))) return false;
        }
        if (driver && driver->ClaimsMerge())
        {
            // Accepted paths reserve the swept corridor. All decisions use the
            // same pre-physics poses and stable pawn update order, preventing
            // simultaneous swaps or two cars claiming the same gap.
            for (double d=driver->ManeuverStart(); d<=driver->ManeuverEnd(); d+=3)
                if (path.Project(driver->Lane().At(d)).error < 2.8) return false;
        }
        // Account for cars/players crossing the maneuver, not just those already
        // centered in the target lane. Use planned AI paths instead of projecting
        // a turning car's velocity indefinitely along a straight line.
        for (int i=0; i<=16; ++i)
        {
            const double t=duration*i/16, at=path.Length()*i/16;
            auto q=pose.origin+velocity*t;
            if (driver && !frozen) q=driver->DrivingPoint(driver->Progress()+std::max(0.0,otherCar ? otherCar->Speed() : 0.0)*t);
            const auto own=path.At(at), tangent=path.Tangent(at), right=Math::Normalize(Math::Cross({0,1,0},tangent));
            const auto delta=q-own;
            if (std::abs(delta.y)<2 && std::abs(Math::Dot(delta,right))<width+1.2 &&
                std::abs(Math::Dot(delta,tangent))<halfLength+2.2+following.standstillGap) return false;
        }
    }
    return true;
}

bool TrafficDriver::TryLaneChange(const TrafficLane& to, double length)
{
    TrafficLane path; double end=0;
    if (!network->LaneChange(*source,progress+1,to,length,path,end) || !GapAvailable(path,to,end)) return false;
    movement.reset(); highwayLink.reset(); granted=false;
    JoinRoute(path,to,progress+1,end); changing=true; changeCooldown=6;
    highwayReason="Changing lane";
    return true;
}

void TrafficDriver::HighwayUpdate(double seconds)
{
    const auto* car=dynamic_cast<const Car*>(Controlled());
    if (!network || suspended || !car) return;
    decisionDelay-=seconds; changeCooldown=std::max(0.0,changeCooldown-seconds); mergeDelay-=seconds;
    if (changing) { highwayReason="Changing lane"; return; }
    if (highwayLink)
    {
        const auto& link=network->links[*highwayLink];
        highwayReason=link.kind==TrafficLinkKind::Exit ? "Taking exit" : link.kind==TrafficLinkKind::Merge ? "Ramp merge" : "Continuing road";
        if (source->roadId == destination->roadId && source->direction != destination->direction)
            highwayReason="Turning around";
        if (link.kind==TrafficLinkKind::Merge && (!mergeGranted || progress<entry-2.2) && entry-progress<100 && mergeDelay<=0)
        {
            mergeGranted=GapAvailable(maneuver,*destination,destinationStation);
            mergeDelay=0.2;
        }
        return;
    }
    if (destination || decisionDelay>0) return;
    highwayReason=nullptr; laneWaitDistance=-1;
    decisionDelay=0.35+(Controlled()->Handle()%5)*0.03;
    const double speed=std::max(0.0,car->Speed());
    const double remaining=source->Length()-progress;
    double length=std::min(std::clamp(speed*3.5,24.0,90.0),remaining-5);
    if (!goalLane)
    {
        int fewestSteps=100;
        for (auto index : network->Family(*source))
        {
            const auto& to=network->lanes[index];
            const int steps=std::abs(to.ordinal-source->ordinal);
            if (!steps || steps>=fewestSteps) continue;
            const auto p=to.Project(source->At(progress));
            if (to.Length()-p.distance<remaining+20 || p.error>steps*7 ||
                Math::Dot(to.Tangent(p.distance),source->Tangent(progress))<0.94) continue;
            if (remaining>std::max(120.0,steps*std::clamp(speed*3.5,24.0,90.0)+60)) continue;
            goalLane=to.id; fewestSteps=steps;
        }
    }
    if (goalLane && goalLane!=source->id)
    {
        const auto* goal=network->FindLane(goalLane);
        highwayReason=goalLink || goalMovement ? "Preparing route lane" : "Preparing lane reduction";
        if (goal && !goalMovement && (!goalLink || network->links[*goalLink].kind!=TrafficLinkKind::Exit))
        {
            const int steps=std::max(1,std::abs(goal->ordinal-source->ordinal));
            length=std::min(length,(remaining-12)/steps);
            laneWaitDistance=std::max(0.0,source->Length()-(24*steps+16));
        }
        if (goalMovement && remaining < 30)
        {
            // A blocked preparation must not cause an illegal turn from the wrong
            // lane. Take an available legal movement, or stop if none exists.
            const auto& choices=network->Outgoing(source->id);
            if (!choices.empty())
            {
                goalMovement=choices[(Controlled()->Handle()+visits)%choices.size()];
                goalLane=source->id; PrepareRoute();
            }
            return;
        }
        if (goalLink && network->links[*goalLink].kind==TrafficLinkKind::Exit)
        {
            const double deadline=source->Project(network->links[*goalLink].path.points.front()).distance;
            if (deadline-progress<length+12)
            {
                // Miss an exit rather than cutting across lanes at the gore.
                skippedExits.insert(*goalLink); goalLane=0; goalLink.reset();
                return;
            }
        }
        if (goal)
            for (auto index : network->Family(*source))
            {
                const auto& to=network->lanes[index];
                if (std::abs(to.ordinal-goal->ordinal)>=std::abs(source->ordinal-goal->ordinal)) continue;
                if (TryLaneChange(to,length)) return;
            }
        // Still obey the current lane end/stop line when a required lane is blocked.
        return;
    }
    if (remaining<120) return;
    if (changeCooldown>0 || goalLane) return;
    const bool pass=leader && targetSpeed<source->speedLimit-2;
    for (auto index : network->Family(*source))
    {
        const auto& to=network->lanes[index];
        const auto p=to.Project(source->At(progress));
        if (to.Length()-p.distance<150) continue;
        const auto right=Math::Normalize(Math::Cross({0,1,0},source->Tangent(progress)));
        const double lateral=Math::Dot(p.position-source->At(progress),right);
        if ((pass && lateral>=0) || (!pass && lateral<=0)) continue;
        if (TryLaneChange(to,length)) { highwayReason=pass ? "Passing slower traffic" : "Keeping right"; return; }
    }
}

void TrafficDriver::CollisionSources(std::vector<Vector3>& points) const
{
    for (double d=0; d<=120; d+=24) points.push_back(lane.At(progress+d));
    if (!network || changing) return;
    // Prefetch potential adjacent travel before a gap is accepted; readiness is
    // still checked along the entire actual maneuver before committing to it.
    for (auto index : network->Family(*source))
    {
        const auto& other=network->lanes[index];
        if (std::abs(other.ordinal-source->ordinal)!=1) continue;
        const auto p=other.Project(lane.At(progress));
        if (p.error>8) continue;
        for (double d=0; d<=120; d+=24) points.push_back(other.At(p.distance+d));
    }
}
