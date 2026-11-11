#pragma once

#include "road.hpp"
#include <SDL3/SDL.h>

class FreeCamera;

enum class RoadLayer { Pavement, Stripes, Outline };

bool DrawPlanRoad(SDL_Renderer* renderer, const RoadGeometry& road, Vector3 center, double scale, SDL_Rect viewport, RoadLayer layer);
