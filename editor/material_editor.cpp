#include "material_editor.hpp"

bool MaterialEditor::Open(TextureCatalog& assets, const std::string& relative)
{
    if (!Material::ValidTexturePath(relative))
        return SDL_SetError("Invalid material path");
    Material next;
    if (!next.Load(assets.Root() + relative))
        return false;
    catalog = &assets;
    path = relative;
    material = std::move(next);
    preview.reset();
    return true;
}

bool MaterialEditor::Commit(Material next)
{
    if (!catalog || !next.Save(catalog->Root() + path))
        return false;
    material = std::move(next);
    preview.reset();
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

SDL_Surface* MaterialEditor::Preview()
{
    if (catalog && (!preview || ImagesChanged()))
    {
        preview = RenderMaterialSphere(material, *catalog);
        imageRevision = catalog->ImageRevision();
    }
    return preview.get();
}

SDL_Surface* MaterialEditor::Thumbnail(std::size_t slot)
{
    return catalog ? catalog->Thumbnail(material.textures.at(slot)) : nullptr;
}
