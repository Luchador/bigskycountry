#pragma once

#include <SDL3/SDL_surface.h>

enum class GizmoSpace { World, Local };

constexpr GizmoSpace OtherSpace(GizmoSpace space)
{
    return space == GizmoSpace::World ? GizmoSpace::Local : GizmoSpace::World;
}

// The caller owns the returned surface.
SDL_Surface* LoadTransformSpaceIcon(GizmoSpace space);
