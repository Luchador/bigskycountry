#pragma once

#include "material_editor.hpp"
#include <SDL3/SDL.h>
#include <memory>
#ifndef _WIN32
#include <set>
#include "sdl_material_editor.hpp"
#endif
#include <string>
#include <vector>

// Model previews and texture thumbnails are loaded on demand.
class ContentBrowser
{
public:
    enum class Tab { Models, Textures };
    void SetTab(Tab next);
    Tab ActiveTab() const { return tab; }
    TextureCatalog& Textures() { return textures; }
    void Refresh();
    SDL_Surface* RoadMaterialThumbnail(const std::string& path, bool strip = false);
#ifndef _WIN32
    SDL_Texture* RoadMaterialTexture(const std::string& path, SDL_Renderer* renderer, bool strip = false);
#endif
#ifndef _WIN32
    // A model-row press selects the preview and returns the path available to drag.
    std::string HandlePointer(const SDL_Event& event, SDL_FPoint local, int width, int height);
    SdlMaterialEditor& MaterialWindow() { return materialWindow; }
    const SdlMaterialEditor& MaterialWindow() const { return materialWindow; }
    bool HandleMaterialPopup(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event, SDL_Rect panel);
    bool MaterialPopupOpen() const { return popup != MaterialPopup::Closed; }
    bool Draw(SDL_Renderer* renderer, int width, int height);
#endif

    struct Row
    {
        std::string label;
        std::string modelPath; // Empty for a folder heading.
    };

    const std::vector<Row>& Rows() const { return rows; }
    const std::string& Selected() const { return selected; }
    const std::string& ScanStatus() const { return scanStatus; }
    const std::string& PreviewStatus() const { return previewStatus; }
    SDL_Surface* Preview() const { return surface.get(); }
    void Select(const std::string& path);
    bool UpdatePreview(); // Returns true when the selected model or its preview changed.

private:
    struct MaterialThumbnail
    {
        MaterialEditor preview;
        std::string path;
        unsigned revision = 0;
        SDL_Surface* surface = nullptr; // Borrowed from preview.
#ifndef _WIN32
        SDL_Texture* texture = nullptr; // Owned by the main renderer.
#endif
    };
    std::array<MaterialThumbnail, 2> materialThumbnails; // Primary and transition inspector slots.
    Tab tab = Tab::Models;
    TextureCatalog textures;
    bool texturesScanned = false;
#ifndef _WIN32
    SdlMaterialEditor materialWindow;
    void UpdateFolderRows();
    std::set<std::string> collapsedFolders;
    std::vector<std::size_t> folderRows;
    void DuplicateSelectedMaterial(SDL_Window* window);
    void RevealSelectedTexture();
    bool DrawMaterialPopup(SDL_Renderer* renderer) const;
    enum class MaterialPopup { Closed, AddMenu, DuplicateMenu, Name };
    bool assetListFocused = false;
    MaterialPopup popup = MaterialPopup::Closed;
    SDL_FRect popupBounds{};
    bool replaceName = false;
    std::string materialName, nameError;
    float folderScroll = 0, textureScroll = 0;
    std::string selectedTexture;
#endif
    std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> surface{nullptr, SDL_DestroySurface};
    std::vector<Row> rows;
    std::string selected;
    std::string scanStatus;
    std::string previewStatus;
    std::size_t triangleCount = 0;
    float scroll = 0;
    bool previewDirty = true;
    SDL_Texture* preview = nullptr; // Owned by the editor renderer. Replaced while it is live.
};
