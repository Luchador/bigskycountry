#pragma once

#include "mesh.hpp"

class WorldDocument;
struct RoadEditorState;

// Visible selection in world space, including transformed mesh bounds.
Bounds3 SelectionBounds(const WorldDocument& world, const RoadEditorState& state);
