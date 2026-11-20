#pragma once

#include "math.hpp"
#include <SDL3/SDL.h>
#include <memory>
#include <span>

class CameraView;

namespace StarSky
{
    // Equatorial unit direction, visual magnitude, display RGB, SDR brightness.
    // The catalog is prepared offline; neither CSV parsing nor color conversion
    // is performed during gameplay. See tools/cook_stars.py and licenses/HYG.txt.
    struct Star { float x, y, z, magnitude, r, g, b, brightness; };
    std::span<const Star> Catalog();
    struct Frame
    {
        Vector3 east, up, north;
        Vector3 ToWorld(Vector3 equatorial) const;
    };
    Frame At(double timeOfDayMinutes);
    float DaylightFade(double sunHeight);
    float HorizonFade(double starHeight);
}

class Stars
{
public:
    Stars();
    ~Stars();
    // One instance per renderer, destroyed before that renderer (like Sky).
    bool Draw(SDL_Renderer* renderer, const CameraView& camera, double minutes,
              double sunHeight, SDL_Rect viewport);
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
