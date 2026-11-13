#pragma once

#include "highway_ramp.hpp"

struct RampSite
{
    int segment, side;
    double t;
};

// Pick a carriageway on a divided road using the world-space surface hit.
std::optional<RampSite> PickRampSite(const Road& host, Vector3 hit);

struct RampPlan
{
    std::vector<RoadPoint> hostPoints;
    RampLink link;
    RampSeam seam;
};
std::optional<RampPlan> PlanRamp(const Road& host, RampSite site, int newStation, const RoadStyle& rampStyle,
                                 bool exit, double length, std::optional<RampLink> replacing = {});
Road NewRamp(const RoadStyle& highway, const RampPlan& plan, bool exit);
std::optional<double> RampTaperLength(const Road& host, const RampLink& link, bool exit, Vector3 click);
