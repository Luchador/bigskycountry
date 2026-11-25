#pragma once

#include "streaming_settings.hpp"
#include <string>

// Machine settings shared by both executables, separate from world/project data.
struct RuntimeConfig
{
    int msaa = 4;
    bool ssao = true;
    int ssaoSamples = 16; // Hemisphere samples per half-resolution pixel, 1..64.
    // 0 inherits the project budget; otherwise 64..16384 MiB for this machine.
    int streamGeometryMiB = 0, streamAssetsMiB = 0, streamCollisionMiB = 0;
    StreamingSettings ApplyStreamingOverrides(StreamingSettings settings) const;
    static RuntimeConfig Load(const std::string& path);
    // Read config.ini beside the executable once, including for preview windows.
    static const RuntimeConfig& Get();
};
