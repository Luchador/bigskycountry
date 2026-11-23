#include "frame_profiler.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>

#ifndef BIGSKY_BUILD_CONFIG
#define BIGSKY_BUILD_CONFIG ""
#endif

void SimulationProfile::Add(const SimulationProfile& other)
{
    controllersMs += other.controllersMs;
    physicsMs += other.physicsMs;
    ticks += other.ticks;
    pawnSteps += other.pawnSteps;
    if (other.maxSubsteps > maxSubsteps)
    {
        maxSubsteps = other.maxSubsteps;
        limitingPawn = other.limitingPawn;
        limitingSpeed = other.limitingSpeed;
        limitingSpin = other.limitingSpin;
    }
}

void FrameProfiler::Reset()
{
    *this = {};
}

void FrameProfiler::Record(const FrameProfile& frame)
{
    if (!std::isfinite(frame.totalMs) || frame.totalMs <= 0) return;
    sum.totalMs += frame.totalMs;
    sum.streamMs += frame.streamMs;
    sum.updateMs += frame.updateMs;
    sum.sceneMs += frame.sceneMs;
    sum.trafficMs += frame.trafficMs;
    sum.presentMs += frame.presentMs;
    sum.limiterMs += frame.limiterMs;
    sum.simulation.Add(frame.simulation);
    ++frames;
    peakMs = std::max(peakMs, frame.totalMs);
    peakTicks = std::max(peakTicks, frame.simulation.ticks);
    if (sum.totalMs < 500) return;
    display = sum;
    displayFrames = frames;
    displayPeakMs = peakMs;
    displayPeakTicks = peakTicks;
    sum = {}; frames = 0; peakMs = 0; peakTicks = 0;
}

bool FrameProfiler::Draw(SDL_Renderer* renderer) const
{
    SDL_Rect viewport{};
    if (!SDL_GetRenderViewport(renderer, &viewport)) return false;
    const SDL_FRect box{std::max(8.f, float(viewport.w) - 448.f), 64.f,
                        std::max(0.f, std::min(440.f, float(viewport.w) - 16.f)), 312.f};
    if (!SDL_SetRenderDrawColor(renderer, 22,24,28,255) || !SDL_RenderFillRect(renderer, &box) ||
        !SDL_SetRenderDrawColor(renderer, 230,232,235,255)) return false;
    float y = box.y + 8;
    const auto line = [&](const char* format, auto... values) {
        const bool ok = SDL_RenderDebugTextFormat(renderer, box.x + 8, y, format, values...);
        y += 18;
        return ok;
    };
    const char* config = BIGSKY_BUILD_CONFIG;
    if (!*config) config = "Unspecified";
    if (!line("Frame profile (F3) | Build: %s", config)) return false;
    if (!displayFrames) return line("Collecting timings...");
    const auto average = [&](double value) { return value / displayFrames; };
    const auto& sim = display.simulation;
    const double otherSimulation = std::max(0.0, display.updateMs - sim.controllersMs - sim.physicsMs);
    const double otherFrame = std::max(0.0, display.totalMs - display.streamMs - display.updateMs -
        display.sceneMs - display.trafficMs - display.presentMs - display.limiterMs);
    return line("Frame: %.1f ms avg | %.1f ms peak", average(display.totalMs), displayPeakMs) &&
        line("Streaming/preparation:  %8.2f ms", average(display.streamMs)) &&
        line("AI/controllers:         %8.2f ms", average(sim.controllersMs)) &&
        line("Physics:                %8.2f ms", average(sim.physicsMs)) &&
        line("Other simulation:       %8.2f ms", average(otherSimulation)) &&
        line("Scene/submit:           %8.2f ms", average(display.sceneMs)) &&
        line("Traffic overlay:        %8.2f ms", average(display.trafficMs)) &&
        line("HUD/input/other:        %8.2f ms", average(otherFrame)) &&
        line("Present/vsync:          %8.2f ms", average(display.presentMs)) &&
        line("Frame limiter:          %8.2f ms", average(display.limiterMs)) &&
        line("Ticks/frame: %.1f avg | %u peak", average(sim.ticks), displayPeakTicks) &&
        line("Substeps/tick peak: %u | pawn steps/frame: %.0f", sim.maxSubsteps, average(double(sim.pawnSteps))) &&
        line("Limiting pawn: %llu | %.1f km/h | %.1f rad/s", static_cast<unsigned long long>(sim.limitingPawn),
             sim.limitingSpeed * 3.6, sim.limitingSpin) &&
        line("Main-thread time; GPU waits may be included.") &&
        line("F8: toggle traffic debug drawing.");
}
