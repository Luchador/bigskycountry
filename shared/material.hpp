#pragma once

#include <array>
#include <string>

enum class MaterialType { Standard, AlphaCutout, Terrain, Unlit, Transition, Foliage, Count };
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
    {"transition", "Transition"},
    {"foliage", "Foliage"}
}};

enum class MaterialSlot { BaseColor, Normal, ORM, Emissive, Opacity, Translucency, OuterBaseColor, OuterNormal, OuterORM, Subsurface, Count };
inline constexpr std::size_t materialSlotCount = static_cast<std::size_t>(MaterialSlot::Count);
inline constexpr std::array<const char*, materialSlotCount> materialSlotLabels{
    "Base color", "Normal", "ORM", "Emissive", "Opacity", "Translucency",
    "Outer base color", "Outer normal", "Outer ORM", "Subsurface"
};

struct Material
{
    MaterialType type = MaterialType::Standard;
    std::array<float, 3> emissiveTint{1, 1, 1}; // Display (sRGB) picker values.
    float translucency = 1;
    float subsurfaceStrength = .5f; // Thin-leaf transmission from directional lights, 0..1.
    bool useVertexAlphaAO = false; // Multiply ORM ambient visibility by mesh vertex alpha.
    static bool IsCutout(MaterialType type) { return type == MaterialType::AlphaCutout || type == MaterialType::Foliage; }
    static bool SupportsVertexAlphaAO(MaterialType type)
    {
        return type == MaterialType::Standard || type == MaterialType::AlphaCutout || type == MaterialType::Foliage;
    }
    bool Uses(MaterialSlot slot) const
    {
        if (type == MaterialType::Transition)
            return slot <= MaterialSlot::ORM || (slot >= MaterialSlot::OuterBaseColor && slot <= MaterialSlot::OuterORM);
        if (type == MaterialType::Foliage)
            return slot < MaterialSlot::Translucency || slot == MaterialSlot::Subsurface;
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
    static void RefreshDefinitions();
    // Rename links live beside definitions and are cooked for saved worlds/undo references.
    static std::string ResolveFile(const std::string& file);
    bool Load(const std::string& file);
    bool Save(const std::string& file) const;
};
