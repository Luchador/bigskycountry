#include "highway_ramp_tool.hpp"
#include "road_curve.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

std::optional<RampSite> PickRampSite(const Road& host, Vector3 hit, int side)
{
    if (host.style.railroad || host.style.GetLaneCount() < 2) return {};
    double best = std::numeric_limits<double>::infinity();
    std::optional<RampSite> site;
    for (int i = 0; i + 1 < static_cast<int>(host.points.size()); ++i)
    {
        const auto& a = host.points[i];
        const auto& b = host.points[i + 1];
        const auto inspect = [&](Vector3 prev, Vector3 next, double t0, double t1)
        {
            const auto delta = next - prev;
            const double squared = Math::Dot(delta, delta);
            if (squared == 0) return;
            const double u = std::clamp(Math::Dot(hit - prev, delta) / squared, 0.0, 1.0);
            const auto offset = hit - (prev + delta * u);
            const double d = Math::Dot(offset, offset);
            if (d >= best) return;
            best = d;
            site = RampSite{i, side, std::lerp(t0, t1, u)};
        };
        if (!host.style.curved) { inspect(a.position, b.position, 0, 1); continue; }
        Vector3 prev = a.position;
        double prevT = 0;
        RoadSpline::SampleCubic(RoadSpline::SegmentCurve(a, b, host.Tangent(i), host.Tangent(i + 1)), 0.02, 0, 0, 1,
            [&](Vector3 next, Vector3, double t) { inspect(prev, next, prevT, t); prev = next; prevT = t; });
    }
    return site;
}

std::optional<RampPlan> PlanRamp(const Road& host, RampSite site, int newStation, const RoadStyle& style,
                                 bool exit, double length)
{
    if (host.style.railroad || host.style.GetLaneCount() < 2 || !style.oneWay || !newStation || site.segment < 0 ||
        site.segment + 1 >= static_cast<int>(host.points.size()) || (site.side != -1 && site.side != 1)) return {};
    int station = newStation;
    if (site.t == 0 || site.t == 1)
    {
        const auto& p = host.points[site.segment + (site.t == 1)];
        if (p.anchor) return {};
        if (p.rampStation) station = p.rampStation;
    }
    RampLink link{station, site.side, length};
    if (!link.Valid()) return {};
    // Test against a copy of the authored spline. Its geometry is regenerated anyway.
    Road staged = host.NewSection();
    staged.points = host.points;
    staged.rampLanes = host.rampLanes;
    staged.SetMouths({host.EndpointMouth(true), host.EndpointMouth(false)});
    for (const auto& l : staged.rampLanes)
        if (l.station == station && l.side == site.side) return {};
    staged.rampLanes.push_back(HighwayRampLane(style, link, exit, host.style.oneWay && site.side > 0));
    if (!staged.InsertStation(site.segment, site.t, station)) return {};
    const auto seam = staged.rampSeams.find({station, site.side});
    if (seam == staged.rampSeams.end() || seam->second.taperLength + 1e-6 < length) return {};
    for (const auto& [key, previous] : host.rampSeams)
    {
        const auto current = staged.rampSeams.find(key);
        // Two joins on the same side cannot occupy the same section, even when their tapers point away.
        if (key.second == site.side && std::abs(previous.goreDistance - seam->second.goreDistance) < Road::minPointSpacing) return {};
        if (current == staged.rampSeams.end() || current->second.taperLength + 1e-6 < previous.taperLength) return {};
    }
    return RampPlan{std::move(staged.points), link, seam->second};
}

Road NewRamp(const RoadStyle& highway, const RampPlan& plan, bool exit)
{
    Road ramp;
    ramp.style.oneWay = true;
    ramp.style.SetLaneCount(1);
    ramp.style.material = highway.material;
    ramp.points.resize(2);
    auto& joined = exit ? ramp.points.front() : ramp.points.back();
    auto& free = exit ? ramp.points.back() : ramp.points.front();
    joined.ramp = plan.link;
    AlignRampEnd(joined, plan.seam);
    const Vector3 left = Math::Normalize(plan.seam.frame.left);
    const Vector3 along = Math::Normalize(plan.seam.forward - left * Math::Dot(plan.seam.forward, left));
    free.position = joined.position + along * (exit ? 35.0 : -35.0) + plan.seam.frame.left * (plan.seam.outwardSide * 12.0);
    ramp.rampCaps[exit ? 0 : 1] = plan.seam.frame;
    ramp.Rebuild();
    return ramp;
}
