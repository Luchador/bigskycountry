#pragma once

#include "asset_paths.hpp"

class ProjectSettings
{
public:
    double RailroadDetailDistance() const { return railroadDetailDistance; }
    bool Load(AssetRoot location);
    bool SaveRailroadDetailDistance(double meters);

private:
    static constexpr double defaultRailroadDetailDistance = 300.0;
    double railroadDetailDistance = defaultRailroadDetailDistance;
};

extern ProjectSettings g_ProjectSettings;
