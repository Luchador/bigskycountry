#include "material_editor.hpp"
#include "math.hpp"
#include <algorithm>
#include <cmath>

bool MaterialEditor::Open(TextureCatalog& assets, const std::string& relative)
{
    if (!Material::ValidAssetPath(relative))
        return SDL_SetError("Invalid material path");
    Material next;
    if (!next.Load(Material::Resolve(assets.Root(), relative)))
        return false;
    catalog = &assets;
    path = relative;
    yaw = pitch = 0.0;
    material = std::move(next);
    previewDirty = true;
    return true;
}

bool MaterialEditor::Commit(Material next)
{
    if (!catalog || !next.Save(Material::Resolve(catalog->Root(), path)))
        return false;
    material = std::move(next);
    previewDirty = true;
    return true;
}

bool MaterialEditor::SetType(MaterialType type)
{
    if (type == material.type)
        return true;
    auto next = material;
    next.type = type;
    return Commit(std::move(next));
}

bool MaterialEditor::SetTexture(MaterialSlot slot, const std::string& relative)
{
    if (!catalog)
        return false;
    const auto index = static_cast<std::size_t>(slot);
    if (!relative.empty() && !catalog->Image(relative))
        return SDL_SetError("Cannot read texture '%s'", relative.c_str());
    if (material.textures.at(index) == relative)
        return true;
    auto next = material;
    next.textures[index] = relative;
    return Commit(std::move(next));
}

bool MaterialEditor::Orbit(double deltaX, double deltaY, double viewportSpan)
{
    if (viewportSpan <= 0.0) return false;
    // Dragging across the shorter viewport dimension turns the sphere half a revolution.
    const double sensitivity = std::numbers::pi / viewportSpan;
    constexpr double pitchLimit = Math::DegreesToRadians(89.0);
    const double nextPitch = std::clamp(pitch + deltaY * sensitivity, -pitchLimit, pitchLimit);
    const double nextYaw = std::remainder(yaw - deltaX * sensitivity, 2.0 * std::numbers::pi);
    if (nextYaw == yaw && nextPitch == pitch) return false;
    yaw = nextYaw;
    pitch = nextPitch;
    previewDirty = true;
    return true;
}

SDL_Surface* MaterialEditor::Preview(int pixelWidth, int pixelHeight)
{
    if (!catalog || pixelWidth <= 0 || pixelHeight <= 0) return nullptr;
    if (!preview || preview->w != pixelWidth || preview->h != pixelHeight)
    {
        preview.reset(SDL_CreateSurface(pixelWidth, pixelHeight, SDL_PIXELFORMAT_BGRA32));
        if (!preview) return nullptr;
        previewDirty = true;
    }
    if (previewDirty || ImagesChanged())
    {
        RenderSphere(*preview);
        previewDirty = false;
        imageRevision = catalog->ImageRevision();
    }
    return preview.get();
}

SDL_Surface* MaterialEditor::Thumbnail(std::size_t slot)
{
    return catalog ? catalog->Thumbnail(material.textures.at(slot)) : nullptr;
}
