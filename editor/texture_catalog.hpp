#pragma once

#include <SDL3/SDL.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

class TextureCatalog
{
public:
    static constexpr int thumbnailSize = 64;

    struct Item
    {
        std::string path; // UTF-8, relative to assets/textures.
        std::string name;
        bool material = false;
    };

    TextureCatalog();
    void Refresh();
    const std::vector<std::string>& Folders() const { return folders; } // Relative paths, including the root ("").
    const std::string& Folder() const { return folder; }
    void SelectFolder(const std::string& path);
    std::string NewMaterialName() const;
    bool CreateMaterial(const std::string& name, std::string& createdPath, std::string& error);
    const std::vector<Item>& Items() const { return items; }
    const std::string& Status() const { return status; }
    const std::string& Root() const { return root; }
    Uint64 ImageRevision() const { return imageRevision; }
    // Borrowed until a refresh detects a changed source. All materials share these pixels.
    SDL_Surface* Image(const std::string& path);
    SDL_Surface* Thumbnail(const std::string& path);
    SDL_Surface* Thumbnail(std::size_t index);
#ifndef _WIN32
    SDL_Texture* ThumbnailTexture(std::size_t index, SDL_Renderer* renderer);
#endif

private:
    using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;
    struct ImageEntry
    {
        Surface image{nullptr, SDL_DestroySurface}, thumbnail{nullptr, SDL_DestroySurface};
        SDL_PathInfo source{};
    };
    struct PathLess
    {
        bool operator()(const std::string& a, const std::string& b) const
        {
#ifdef _WIN32
            return SDL_strcasecmp(a.c_str(), b.c_str()) < 0;
#else
            return a < b;
#endif
        }
    };
    std::map<std::string, ImageEntry, PathLess> images;
    Uint64 imageRevision = 0;
#ifndef _WIN32
    std::vector<SDL_Texture*> thumbnailTextures;
#endif
    void ReadFolder();
    std::vector<std::string> folders;
    std::string folder;
    std::string root;
    std::string status;
    std::vector<Item> items;
};
