#pragma once
#include <array>

// Shallow inland water. Colors are display (sRGB) values; lengths are meters.
struct WaterMaterial
{
    std::array<float, 3> color{.08f, .26f, .22f};
    float roughness = .16f;
    float waveStrength = .18f;
    float waveScale = 3;
    float waveSpeed = .6f;
    float clarity = 4; // Distance at which half the background light remains.
    float refraction = .15f;
    float shoreWidth = .35f;
    float foam = .2f;
    bool operator==(const WaterMaterial&) const = default;
};

struct WaterProperty
{
    const char* key;
    const char* label;
    float WaterMaterial::* member;
    float minimum, maximum;
};
inline constexpr std::array waterProperties{
    WaterProperty{"roughness", "Roughness", &WaterMaterial::roughness, .02f, 1},
    WaterProperty{"waveStrength", "Ripple strength", &WaterMaterial::waveStrength, 0, 1},
    WaterProperty{"waveScale", "Wave scale (m)", &WaterMaterial::waveScale, .1f, 30},
    WaterProperty{"waveSpeed", "Wave speed (m/s)", &WaterMaterial::waveSpeed, 0, 5},
    WaterProperty{"clarity", "Clarity depth (m)", &WaterMaterial::clarity, .1f, 50},
    WaterProperty{"refraction", "Refraction (m)", &WaterMaterial::refraction, 0, 1},
    WaterProperty{"shoreWidth", "Shore blend (m)", &WaterMaterial::shoreWidth, .01f, 3},
    WaterProperty{"foam", "Shore foam", &WaterMaterial::foam, 0, 1}
};
