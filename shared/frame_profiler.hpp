#pragma once

#include <cstddef>
#include <cstdint>

struct SDL_Renderer;

// Counts and main-thread time for a physics tick or a complete render frame.
struct SimulationProfile
{
    double controllersMs = 0, physicsMs = 0;
    unsigned ticks = 0, maxSubsteps = 0;
    std::size_t pawnSteps = 0;
    std::uint64_t limitingPawn = 0;
    double limitingSpeed = 0, limitingSpin = 0;
    void Add(const SimulationProfile& other);
};

struct FrameProfile
{
    double totalMs = 0, streamMs = 0, updateMs = 0, sceneMs = 0;
    double trafficMs = 0, presentMs = 0, limiterMs = 0;
    SimulationProfile simulation;
};

class FrameProfiler
{
public:
    void Reset();
    void Record(const FrameProfile& frame);
    bool Draw(SDL_Renderer* renderer) const;
private:
    FrameProfile sum, display;
    unsigned frames = 0, displayFrames = 0, peakTicks = 0, displayPeakTicks = 0;
    double peakMs = 0, displayPeakMs = 0;
};
