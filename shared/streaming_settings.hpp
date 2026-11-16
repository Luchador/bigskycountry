#pragma once
#include <cstddef>

struct StreamingSettings
{
    static constexpr std::size_t MiB = 1024 * 1024;
    std::size_t geometryBytes = 512 * MiB;
    std::size_t assetBytes = 512 * MiB; // Half shared models, half shared textures.
    std::size_t collisionBytes = 256 * MiB;
    double preloadMeters = 512;
    double unloadMarginMeters = 768;
    double collisionMeters = 256;
    // Collision essential to live pawns can exceed a budget. Report that pressure,
    // rather than dropping their support surface or endlessly retrying an oversized chunk.
};
