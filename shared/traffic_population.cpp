#include "traffic_population.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace
{
    // Clip a line to a sphere. Using actual line/sphere intersections also finds
    // nearby portions of very long lanes whose endpoints are both far away.
    std::optional<std::pair<double,double>> InsideSphere(Vector3 a, Vector3 delta, Vector3 center, double radius)
    {
        const auto offset = a-center;
        const double length2 = Math::Dot(delta,delta);
        if (length2 < 1e-16) return {};
        const double along = Math::Dot(offset,delta);
        const double discriminant = along*along-length2*(Math::Dot(offset,offset)-radius*radius);
        if (discriminant <= 0) return {};
        const double root = std::sqrt(discriminant);
        const double first = std::max(0.0,(-along-root)/length2);
        const double last = std::min(1.0,(-along+root)/length2);
        if (last <= first) return {};
        return std::pair{first,last};
    }
    constexpr double cameraMargin = 8;
}

TrafficPopulation::TrafficPopulation(const TrafficNetwork& network, std::uint32_t seed)
    : network(network), random(seed)
{
    index.BeginSync();
    for (std::size_t i=0; i<network.lanes.size(); ++i)
    {
        const auto& lane = network.lanes[i];
        // Index bounded stretches instead of continental road AABBs. A population
        // update only examines the local portions, even on a road many km long.
        for (double first=8; first<lane.Length()-runwayMeters; first+=256)
        {
            const double last = std::min(first+256,lane.Length()-runwayMeters);
            Bounds3 bounds;
            bounds.Include(lane.At(first)); bounds.Include(lane.At(last));
            auto at = std::upper_bound(lane.distances.begin(),lane.distances.end(),first);
            for (; at!=lane.distances.end() && *at<last; ++at)
                bounds.Include(lane.points[at-lane.distances.begin()]);
            index.Upsert({SceneKind::Road,static_cast<int>(spans.size())},bounds);
            spans.push_back({i,first,last});
        }
    }
    index.EndSync();
}

bool TrafficPopulation::InSpawnArea(Vector3 point, Vector3 player, const CameraView& view)
{
    const double distance2 = Math::Dot(point-player,point-player);
    const auto cameraOffset = point-view.GetPosition();
    // Nearby spawns stay behind the camera. Distant forward spawns keep a fast
    // player encountering traffic on straight roads instead of leaving it behind.
    return distance2 >= spawnNearMeters*spawnNearMeters && distance2 <= spawnFarMeters*spawnFarMeters &&
        (Math::Dot(cameraOffset,cameraOffset) >= visibleSpawnMeters*visibleSpawnMeters ||
         Math::Dot(cameraOffset,view.Forward()) < -cameraMargin);
}

std::optional<TrafficPopulation::Site> TrafficPopulation::Pick(Vector3 player, const CameraView& view)
{
    std::map<int,std::vector<Span>> roads;
    for (auto key : index.Query(player,spawnFarMeters))
    {
        const auto& span = spans[key.index];
        const auto& lane = network.lanes[span.lane];
        double first = span.first;
        auto next = std::upper_bound(lane.distances.begin(),lane.distances.end(),first);
        while (first < span.last && next != lane.distances.end())
        {
            const double last = std::min(span.last,*next++);
            const auto a = lane.At(first), delta = lane.At(last)-a;
            const auto outer = InsideSphere(a,delta,player,spawnFarMeters);
            if (outer)
            {
                const auto add = [&](double lo, double hi, bool hide) {
                    // Visibility filtering happens before the road lottery.
                    if (hide)
                    {
                        const double atCamera = Math::Dot(a-view.GetPosition(),view.Forward())+cameraMargin;
                        const double slope = Math::Dot(delta,view.Forward());
                        if (std::abs(slope)<1e-12) { if (atCamera>=0) return; }
                        else if (slope>0) hi=std::min(hi,-atCamera/slope);
                        else lo=std::max(lo,-atCamera/slope);
                    }
                    if ((hi-lo)*(last-first)<0.1) return;
                    roads[lane.roadId].push_back({span.lane,first+(last-first)*lo,first+(last-first)*hi});
                };
                const auto visible = InsideSphere(a,delta,view.GetPosition(),visibleSpawnMeters);
                const auto allow = [&](double lo, double hi) {
                    if (!visible) { add(lo,hi,false); return; }
                    add(lo,std::min(hi,visible->first),false);
                    add(std::max(lo,visible->first),std::min(hi,visible->second),true);
                    add(std::max(lo,visible->second),hi,false);
                };
                if (const auto inner = InsideSphere(a,delta,player,spawnNearMeters))
                {
                    allow(outer->first,std::min(outer->second,inner->first));
                    allow(std::max(outer->first,inner->second),outer->second);
                }
                else allow(outer->first,outer->second);
            }
            first = last;
        }
    }
    if (roads.empty()) return {};
    // One ticket per road, independent of lane count, length, or subdivision.
    auto road = roads.begin();
    std::advance(road,static_cast<std::ptrdiff_t>(std::uniform_int_distribution<std::size_t>(0,roads.size()-1)(random)));
    double length = 0;
    for (const auto& range : road->second) length += range.last-range.first;
    double position = std::uniform_real_distribution<double>(0,length)(random);
    for (const auto& range : road->second)
    {
        const double count = range.last-range.first;
        if (position <= count) return Site{range.lane,range.first+position};
        position -= count;
    }
    return Site{road->second.back().lane,road->second.back().last};
}
