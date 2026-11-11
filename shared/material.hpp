#pragma once

#include <array>
#include <string>

enum class MaterialType { Standard, AlphaCutout, Count };
struct MaterialTypeInfo
{
    const char* key;
    const char* label;
};
inline constexpr std::array<MaterialTypeInfo, static_cast<std::size_t>(MaterialType::Count)> materialTypes{{
    {"standard", "Standard"},
    {"alphaCutout", "Alpha cutout"}
}};

enum class MaterialSlot { BaseColor, Normal, ORM, Emissive, Opacity, Count };
inline constexpr std::size_t materialSlotCount = static_cast<std::size_t>(MaterialSlot::Count);
inline constexpr std::array<const char*, materialSlotCount> materialSlotLabels{
    "Base color", "Normal", "ORM", "Emissive", "Opacity"
};

struct Material
{
    MaterialType type = MaterialType::Standard;
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
