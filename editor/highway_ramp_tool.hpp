#pragma once

#include "highway_ramp.hpp"

struct RampSite
{
    int segment, side;
    double t;
};

// Locate a point picked on an outer lane guide along the highway spline.
std::optional<RampSite> PickRampSite(const Road& host, Vector3 hit, int side);

struct RampPlan
{
    std::vector<RoadPoint> hostPoints;
    RampLink link;
    RampSeam seam;
};
std::optional<RampPlan> PlanRamp(const Road& host, RampSite site, int newStation, const RoadStyle& rampStyle,
                                 bool exit, double length);
Road NewRamp(const RoadStyle& highway, const RampPlan& plan, bool exit);
