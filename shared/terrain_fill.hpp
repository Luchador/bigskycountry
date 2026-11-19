#pragma once

#include "mesh.hpp"
#include "road.hpp"

#include <span>

enum class TerrainFillScope { AllPockets, SelectedBoundary };

// Footprints and links are in world space. Links join the anchors of an intersection
// which has no rendered pavement yet. Exclusions are closed shoreline polygons;
// terrain meets their exact rim vertices and never covers their interiors.
// Junction seams permit terrain skirts between the two elevations at a rail/rail
// footprint corner. They are supplied only for authored, at-grade connections.
// SelectedBoundary skips incidental pockets bounded by a strict subset of the
// footprints enclosing another valid pocket. AllPockets is for geometry helpers
// (such as bridge ground patches) that must fill every enclosed area.
// Returns an empty mesh pointer for an open boundary.
std::shared_ptr<const Mesh> BuildTerrainFill(std::span<const RoadGeometry* const> footprints,
                                             std::span<const std::array<Vector3, 2>> links,
                                             std::span<const std::vector<Vector3>> exclusions = {},
                                             std::span<const std::array<Vector3, 2>> junctionSeams = {},
                                             TerrainFillScope scope = TerrainFillScope::AllPockets);
