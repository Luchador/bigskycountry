#pragma once

#include <array>
#include <string>

enum class MaterialType { Standard, AlphaCutout, Terrain, Unlit, Transition, Count };
struct MaterialTypeInfo
{
    const char* key;
    const char* label;
};
inline constexpr std::array<MaterialTypeInfo, static_cast<std::size_t>(MaterialType::Count)> materialTypes{{
    {"standard", "Standard"},
    {"alphaCutout", "Alpha cutout"},
    {"terrain", "Terrain"},
    {"unlit", "Unlit"},
    {"transition", "Transition"}
}};

enum class MaterialSlot { BaseColor, Normal, ORM, Emissive, Opacity, Translucency, OuterBaseColor, OuterNormal, OuterORM, Count };
inline constexpr std::size_t materialSlotCount = static_cast<std::size_t>(MaterialSlot::Count);
inline constexpr std::array<const char*, materialSlotCount> materialSlotLabels{
    "Base color", "Normal", "ORM", "Emissive", "Opacity", "Translucency",
    "Outer base color", "Outer normal", "Outer ORM"
};

struct Material
{
    MaterialType type = MaterialType::Standard;
    std::array<float, 3> emissiveTint{1, 1, 1}; // Display (sRGB) picker values.
    float translucency = 1;
    bool Uses(MaterialSlot slot) const
    {
        if (type == MaterialType::Transition)
            return slot <= MaterialSlot::ORM || (slot >= MaterialSlot::OuterBaseColor && slot < MaterialSlot::Count);
        return type == MaterialType::Unlit ? slot == MaterialSlot::Emissive || slot == MaterialSlot::Translucency
                                           : slot < MaterialSlot::Translucency;
    }
    const char* SlotLabel(std::size_t index) const
    {
        if (type == MaterialType::Transition && index <= static_cast<std::size_t>(MaterialSlot::ORM))
        {
            constexpr std::array inner{"Inner base color", "Inner normal", "Inner ORM"};
            return inner[index];
        }
        return materialSlotLabels.at(index);
    }
    std::array<std::string, materialSlotCount> textures{}; // Relative to assets/textures. Empty means unassigned.
    static bool ValidTexturePath(const std::string& path);
    static bool ValidAssetPath(const std::string& path);
    // Main-thread cache generations for saved material definitions and refreshed source images.
    static unsigned Revision();
    static unsigned ImageRevision();
    static void RefreshAssets();
    bool Load(const std::string& file);
    bool Save(const std::string& file) const;
};
