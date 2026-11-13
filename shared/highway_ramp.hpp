#pragma once

#include "road.hpp"

RampLane HighwayRampLane(const RoadStyle& style, const RampLink& link, bool exit);
void AlignRampEnd(RoadPoint& end, const RampSeam& seam);
