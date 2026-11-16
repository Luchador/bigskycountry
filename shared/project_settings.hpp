#pragma once

#include "asset_paths.hpp"

class ProjectSettings
{
public:
    double RailroadDetailDistance() const { return railroadDetailDistance; }
    double ViewDistance() const { return viewDistance; }
    static constexpr double minViewDistance = 100, maxViewDistance = 128000;
    bool Load(AssetRoot location);
    bool SaveDistances(double railroadMeters, double viewMeters);

private:
    static constexpr double defaultRailroadDetailDistance = 300.0;
    double railroadDetailDistance = defaultRailroadDetailDistance;
    static constexpr double defaultViewDistance = 8000.0;
    double viewDistance = defaultViewDistance;
};

extern ProjectSettings g_ProjectSettings;
