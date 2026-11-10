#include "texture_catalog.hpp"
#include "texture_loader.hpp"
#include <SDL3_image/SDL_image.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

namespace
{
    namespace fs = std::filesystem;
    using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;

    std::string Utf8(const fs::path& path)
    {
        const auto bytes = path.generic_u8string();
        return {bytes.begin(), bytes.end()};
    }

    bool IsImage(const fs::path& path)
    {
        static constexpr const char* extensions[] = {
            ".bmp", ".cur", ".gif", ".ico", ".jpeg", ".jpg", ".jpe", ".lbm", ".iff",
            ".pcx", ".png", ".apng", ".pnm", ".pbm", ".pgm", ".ppm", ".qoi", ".svg",
            ".tga", ".xcf", ".xpm", ".xv"
        };
        const std::string ext = Utf8(path.extension());
        return std::any_of(std::begin(extensions), std::end(extensions),
                           [&](const char* candidate) { return SDL_strcasecmp(ext.c_str(), candidate) == 0; });
    }

    Surface MakeThumbnail(const std::string& root, const std::string& path)
    {
        constexpr int size = TextureCatalog::thumbnailSize;
        Surface source(nullptr, SDL_DestroySurface);
        // Use the world loader for RLE true-color TGAs, including right-origin images.
        const char* extension = SDL_strrchr(path.c_str(), '.');
        if (extension && SDL_strcasecmp(extension, ".tga") == 0)
        {
            source.reset(TextureLoader::LoadTgaSurface(path.c_str()));
        }
        if (!source)
        {
            source.reset(IMG_Load((root + path).c_str()));
        }
        Surface thumb(nullptr, SDL_DestroySurface);
        if (!source || source->w <= 0 || source->h <= 0)
        {
            return thumb;
        }
        thumb.reset(SDL_CreateSurface(size, size, SDL_PIXELFORMAT_BGRA32));
        if (!thumb)
        {
            return thumb;
        }
        const double scale = double(size) / std::max(source->w, source->h);
        const int w = std::max(1, static_cast<int>(std::lround(source->w * scale)));
        const int h = std::max(1, static_cast<int>(std::lround(source->h * scale)));
        const SDL_Rect image{(size - w) / 2, (size - h) / 2, w, h};
        SDL_FillSurfaceRect(thumb.get(), nullptr, SDL_MapSurfaceRGB(thumb.get(), 37, 42, 46));
        const Uint32 light = SDL_MapSurfaceRGB(thumb.get(), 122, 128, 132);
        const Uint32 dark = SDL_MapSurfaceRGB(thumb.get(), 83, 89, 93);
        constexpr int cell = 8;
        for (int y = 0; y < h; y += cell)
        {
            for (int x = 0; x < w; x += cell)
            {
                SDL_Rect tile{image.x + x, image.y + y, std::min(cell, w - x), std::min(cell, h - y)};
                SDL_FillSurfaceRect(thumb.get(), &tile, (x / cell + y / cell) % 2 ? light : dark);
            }
        }

        // Convert first so indexed images and color keys retain transparency when scaled.
        if (source->format != SDL_PIXELFORMAT_BGRA32)
        {
            source.reset(SDL_ConvertSurface(source.get(), SDL_PIXELFORMAT_BGRA32));
        }
        if (!source || !SDL_SetSurfaceBlendMode(source.get(), SDL_BLENDMODE_BLEND) ||
            !SDL_BlitSurfaceScaled(source.get(), nullptr, thumb.get(), &image, SDL_SCALEMODE_LINEAR))
        {
            thumb.reset();
        }
        return thumb;
    }
}

void TextureCatalog::Refresh()
{
#ifndef _WIN32
    for (const auto& preview : previews)
    {
        if (preview.texture)
        {
            SDL_DestroyTexture(preview.texture);
        }
    }
#endif
    previews.clear();
    items.clear();
    status.clear();
    const char* base = SDL_GetBasePath();
    if (base)
    {
        root = std::string(base) + "assets/textures/";
        const fs::path folder(std::u8string(root.begin(), root.end()));
        std::error_code error;
        fs::recursive_directory_iterator it(folder, fs::directory_options::skip_permission_denied, error), end;
        for (; !error && it != end; it.increment(error))
        {
            std::error_code statError;
            if (it->is_regular_file(statError) && IsImage(it->path()))
            {
                const auto relative = it->path().lexically_relative(folder);
                items.push_back({Utf8(relative), Utf8(relative.filename()), Utf8(relative.parent_path())});
            }
        }
        if (error && error != std::errc::no_such_file_or_directory)
        {
            status = "Cannot read texture folder";
        }
    }
    else
    {
        status = "Cannot read texture folder";
    }
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.path < b.path; });
    previews.resize(items.size());
    if (items.empty() && status.empty())
    {
        status = "No textures found";
    }
}

SDL_Surface* TextureCatalog::Thumbnail(std::size_t index)
{
    auto& preview = previews.at(index);
    if (!preview.attempted)
    {
        preview.attempted = true;
        preview.surface = MakeThumbnail(root, items[index].path);
        if (!preview.surface)
        {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Texture preview '%s': %s", items[index].path.c_str(), SDL_GetError());
        }
    }
    return preview.surface.get();
}

#ifndef _WIN32
SDL_Texture* TextureCatalog::ThumbnailTexture(std::size_t index, SDL_Renderer* renderer)
{
    auto& preview = previews.at(index);
    if (!preview.texture)
    {
        if (SDL_Surface* thumb = Thumbnail(index))
        {
            preview.texture = SDL_CreateTextureFromSurface(renderer, thumb);
        }
    }
    return preview.texture;
}
#endif
