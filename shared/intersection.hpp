#pragma once

#include "road.hpp"

// Authoring data for a flat junction. Its center is at world elevation position.y.
struct Intersection
{
    Vector3 position{};
    double cornerRadius = 6.0; // meters
    std::string material = RoadStyle::defaultMaterial;
    bool operator==(const Intersection&) const = default;
};
