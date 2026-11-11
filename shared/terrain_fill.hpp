#pragma once

#include "mesh.hpp"
#include "road.hpp"

#include <span>

// Footprints and links are in world space. Links join the anchors of an intersection
// which has no rendered pavement yet. Returns an empty mesh pointer for an open boundary.
std::shared_ptr<const Mesh> BuildTerrainFill(std::span<const RoadGeometry* const> footprints,
                                             std::span<const std::array<Vector3, 2>> links);
