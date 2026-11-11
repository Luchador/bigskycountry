#pragma once
#include "material.hpp"
#include "texture_catalog.hpp"

using MaterialSurface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;

class MaterialEditor
{
public:
    bool Open(TextureCatalog& assets, const std::string& path);
    bool SetType(MaterialType type);
    bool SetTexture(MaterialSlot slot, const std::string& path);
    const Material& Current() const { return material; }
    bool Orbit(double deltaX, double deltaY, double viewportSpan);
    SDL_Surface* Preview(int pixelWidth, int pixelHeight);
    SDL_Surface* Thumbnail(std::size_t slot);
    bool ImagesChanged() const { return catalog && imageRevision != catalog->ImageRevision(); }

private:
    bool Commit(Material next);
    MaterialSurface RenderSphere(int width, int height) const;
    TextureCatalog* catalog = nullptr;
    std::string path;
    Material material;
    double yaw = 0.0, pitch = 0.0;
    MaterialSurface preview{nullptr, SDL_DestroySurface};
    Uint64 imageRevision = 0;
};
