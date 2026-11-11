#pragma once
#include "material.hpp"
#include "texture_catalog.hpp"

using MaterialSurface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;
MaterialSurface RenderMaterialSphere(const Material& material, TextureCatalog& assets);

class MaterialEditor
{
public:
    bool Open(TextureCatalog& assets, const std::string& path);
    bool SetType(MaterialType type);
    bool SetTexture(MaterialSlot slot, const std::string& path);
    const Material& Current() const { return material; }
    SDL_Surface* Preview();
    SDL_Surface* Thumbnail(std::size_t slot);
    bool ImagesChanged() const { return catalog && imageRevision != catalog->ImageRevision(); }

private:
    bool Commit(Material next);
    TextureCatalog* catalog = nullptr;
    std::string path;
    Material material;
    MaterialSurface preview{nullptr, SDL_DestroySurface};
    Uint64 imageRevision = 0;
};
