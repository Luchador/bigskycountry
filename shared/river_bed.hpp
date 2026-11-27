#pragma once

#include "lake.hpp"
#include <array>
#include <span>

// Bank-relative sampling and depth, independent of the river's overall length
// or its orientation in the world. Junctions use the exposed union shoreline.
class RiverBedProfile
{
public:
    explicit RiverBedProfile(std::span<const LakeSample> rim);
    std::vector<Vector3> InteriorSamples() const;
    double Depth(Vector3 point, double maximum, double shoreDistance) const;

private:
    std::array<std::vector<Vector3>, 2> banks;
};
