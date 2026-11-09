#pragma once

#include "road.hpp"
#include <SDL3/SDL.h>

class FreeCamera;

inline constexpr double roadNearDepth = 0.1;
enum class RoadLayer { Pavement, Stripes, Outline };

bool DrawPlanRoad(SDL_Renderer* renderer, const Road& road, Vector3 center, double scale, SDL_Rect viewport, RoadLayer layer);
