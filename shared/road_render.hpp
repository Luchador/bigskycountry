#pragma once

#include "road.hpp"
#include <SDL3/SDL.h>

class FreeCamera;

inline constexpr double roadNearDepth = 0.1;
enum class RoadLayer { Pavement, Stripes, Outline };

bool DrawWorldRoads(SDL_Renderer* renderer, const FreeCamera& camera, const std::vector<Road>& roads);
bool DrawWorldRoadOutline(SDL_Renderer* renderer, const FreeCamera& camera, const Road& road, SDL_Rect viewport);
bool DrawPlanRoad(SDL_Renderer* renderer, const Road& road, Vector3 center, double scale, SDL_Rect viewport, RoadLayer layer);
