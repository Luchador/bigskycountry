#pragma once

#include "traffic_network.hpp"
#include <SDL3/SDL.h>
#include <set>

class CameraView;
class WorldDocument;

bool DrawTrafficLanes(SDL_Renderer* renderer, const TrafficNetwork& network, const CameraView& camera,
                      SDL_Rect viewport, bool plan = false, Vector3 center = {}, double scale = 1,
                      std::uint64_t selected = 0, const std::set<int>& hiddenRoads = {}, const TrafficLane* route = nullptr);

// Geometry changes invalidate this debug view; camera movement does not rebuild paths.
class EditorTrafficOverlay
{
public:
    bool Draw(SDL_Renderer* renderer, const WorldDocument& world, const CameraView& camera,
              SDL_Rect viewport, bool plan, Vector3 center, double scale);
private:
    TrafficNetwork network;
    std::vector<std::pair<int, std::uint64_t>> revisions;
    std::vector<int> controls;
};
