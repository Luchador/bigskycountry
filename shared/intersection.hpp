#pragma once

#include "road.hpp"

// Authoring data for a flat junction. Its center is at world elevation position.y.
struct Intersection
{
    Vector3 position{};
    double cornerRadius = 6.0; // meters
    std::string material = RoadStyle::defaultMaterial;
    static constexpr double anchorDistance = 10.0; // Approach centerlines, in meters from the junction center.
    Vector3 AnchorPosition(int side) const { return position + IntersectionAnchor::directions.at(side) * anchorDistance; }

    bool operator==(const Intersection&) const = default;
};
