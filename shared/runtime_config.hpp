#pragma once

#include <string>

// Machine settings shared by both executables, separate from world/project data.
struct RuntimeConfig
{
    int msaa = 4;
    bool ssao = true;
    int ssaoSamples = 16; // Hemisphere samples per half-resolution pixel, 1..64.
    static RuntimeConfig Load(const std::string& path);
    // Read config.ini beside the executable once, including for preview windows.
    static const RuntimeConfig& Get();
};
