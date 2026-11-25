#pragma once

#include "asset_paths.hpp"
#include "streaming_settings.hpp"

class ProjectSettings
{
public:
    double RailroadDetailDistance() const { return railroadDetailDistance; }
    double ShadowDistance() const { return shadowDistance; }
    static constexpr double maxShadowDistance = 10000;
    double ViewDistance() const { return viewDistance; }
    static constexpr double minViewDistance = 100, maxViewDistance = 128000;
    bool Load(AssetRoot location);
    bool SaveDistances(double railroadMeters, double viewMeters);
    StreamingSettings Streaming() const; // Effective budgets, including local config.ini overrides.
    const StreamingSettings& StreamingDefaults() const { return streaming; }
    bool SaveOptions(double railroadMeters, double viewMeters, const StreamingSettings& budgets)
        { return SaveOptions(railroadMeters, viewMeters, budgets, shadowDistance); }
    bool SaveOptions(double railroadMeters, double viewMeters, const StreamingSettings& budgets, double shadowMeters);

private:
    static constexpr double defaultShadowDistance = 2000;
    double shadowDistance = defaultShadowDistance;
    StreamingSettings streaming;
    static constexpr double defaultRailroadDetailDistance = 300.0;
    double railroadDetailDistance = defaultRailroadDetailDistance;
    static constexpr double defaultViewDistance = 8000.0;
    double viewDistance = defaultViewDistance;
};

extern ProjectSettings g_ProjectSettings;
