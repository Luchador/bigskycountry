#pragma once

#include <SDL3/SDL.h>
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
        std::string folder;
    };

    void Refresh();
    const std::vector<Item>& Items() const { return items; }
    const std::string& Status() const { return status; }
    SDL_Surface* Thumbnail(std::size_t index);
#ifndef _WIN32
    SDL_Texture* ThumbnailTexture(std::size_t index, SDL_Renderer* renderer);
#endif

private:
    using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;
    struct Preview
    {
        Surface surface{nullptr, SDL_DestroySurface};
        bool attempted = false;
#ifndef _WIN32
        SDL_Texture* texture = nullptr; // The renderer owns textures through shutdown.
#endif
    };

    std::string root;
    std::string status;
    std::vector<Item> items;
    std::vector<Preview> previews;
};
