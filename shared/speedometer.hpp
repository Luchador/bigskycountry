#pragma once

#include <SDL3/SDL.h>

// Game HUD, anchored to the bottom-right of the current viewport.
bool DrawSpeedometer(SDL_Renderer* renderer, double metersPerSecond);
