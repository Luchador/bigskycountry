#pragma once

#include "road.hpp"
#include <SDL3/SDL.h>

class FreeCamera;

enum class RoadLayer { Pavement, Stripes };

bool DrawWorldRoads(SDL_Renderer* renderer, const FreeCamera& camera, const RoadMap& map);
bool DrawPlanRoad(SDL_Renderer* renderer, const Road& road, Vector3 center, double scale, SDL_Rect viewport, RoadLayer layer);
