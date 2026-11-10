#include "texture_catalog.hpp"
#include "editor_theme.hpp"
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

    fs::path FromUtf8(const std::string& text)
    {
        return fs::path(std::u8string(text.begin(), text.end()));
    }

    bool ValidMaterialName(const std::string& name)
    {
        if (name.empty() || name.front() == ' ' || name.back() == ' ' || name.back() == '.' ||
            name.find_first_of("<>:\"/\\|?*") != std::string::npos ||
            std::any_of(name.begin(), name.end(), [](unsigned char c) { return c < 32; })) return false;

        // Windows device names are reserved even with an extension.
        const auto stem = name.substr(0, name.find('.'));
        for (const char* device : {"CON", "PRN", "AUX", "NUL", "CONIN$", "CONOUT$"})
            if (SDL_strcasecmp(stem.c_str(), device) == 0) return false;
        const bool port = SDL_strncasecmp(stem.c_str(), "COM", 3) == 0 || SDL_strncasecmp(stem.c_str(), "LPT", 3) == 0;
        if (port && stem.size() == 4 && stem[3] >= '1' && stem[3] <= '9') return false;
        return true;
    }

    bool IsImage(const fs::path& path)
    {
        static constexpr const char* extensions[] = {
            ".bmp", ".cur", ".gif", ".ico", ".jpeg", ".jpg", ".jpe", ".lbm", ".iff",
            ".pcx", ".png", ".apng", ".pnm", ".pbm", ".pgm", ".ppm", ".qoi", ".svg",
            ".tga", ".tif", ".tiff", ".xcf", ".xpm", ".xv"
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
            source.reset(TextureLoader::LoadTgaSurfaceFromFile((root + path).c_str()));
        }
        if (!source)
        {
            const std::string file = root + path;
            // Keep PNG thumbnails in straight alpha on every platform.
            const bool png = extension && (SDL_strcasecmp(extension, ".png") == 0 || SDL_strcasecmp(extension, ".apng") == 0);
            source.reset(png ? SDL_LoadPNG(file.c_str()) : IMG_Load(file.c_str()));
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
        const auto pixel = [&](SDL_Color c) { return SDL_MapSurfaceRGB(thumb.get(), c.r, c.g, c.b); };
        SDL_FillSurfaceRect(thumb.get(), nullptr, pixel(EditorTheme::preview));
        const Uint32 light = pixel(EditorTheme::checkerLight);
        const Uint32 dark = pixel(EditorTheme::checkerDark);
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
        // libtiff's RGBA reader and Windows Imaging Component return associated alpha.
        const bool tiff = extension && (SDL_strcasecmp(extension, ".tif") == 0 || SDL_strcasecmp(extension, ".tiff") == 0);
        const SDL_BlendMode blend = tiff ? SDL_BLENDMODE_BLEND_PREMULTIPLIED : SDL_BLENDMODE_BLEND;
        if (!source || !SDL_SetSurfaceBlendMode(source.get(), blend) ||
            !SDL_BlitSurfaceScaled(source.get(), nullptr, thumb.get(), &image, SDL_SCALEMODE_LINEAR))
        {
            thumb.reset();
        }
        return thumb;
    }
}

void TextureCatalog::Refresh()
{
    root.clear();
    fs::path assets;
    std::error_code error;
#ifdef BIGSKY_PROJECT_ASSETS
    if (fs::is_directory(BIGSKY_PROJECT_ASSETS, error)) assets = BIGSKY_PROJECT_ASSETS;
#endif
    if (assets.empty())
    {
        if (const char* base = SDL_GetBasePath()) assets = FromUtf8(base) / "assets";
    }
    folders = {""};
    if (!assets.empty())
    {
        const auto directory = assets / "textures";
        root = Utf8(directory) + "/";
        fs::recursive_directory_iterator it(directory, fs::directory_options::skip_permission_denied, error), end;
        for (; !error && it != end; it.increment(error))
        {
            std::error_code stat;
            if (!it->is_symlink(stat) && it->is_directory(stat))
                folders.push_back(Utf8(it->path().lexically_relative(directory)));
        }
    }
    // Compare path components so each subtree stays together (including names such as "road-old").
    std::sort(folders.begin(), folders.end(), [](const auto& a, const auto& b) { return FromUtf8(a) < FromUtf8(b); });
    if (std::find(folders.begin(), folders.end(), folder) == folders.end()) folder.clear();
    ReadFolder();
    if (error && error != std::errc::no_such_file_or_directory) status = "Cannot read texture folders";
}

void TextureCatalog::SelectFolder(const std::string& path)
{
    if (path != folder && std::find(folders.begin(), folders.end(), path) != folders.end())
    {
        folder = path;
        ReadFolder();
    }
}

void TextureCatalog::ReadFolder()
{
#ifndef _WIN32
    for (const auto& preview : previews)
        if (preview.texture) SDL_DestroyTexture(preview.texture);
#endif
    previews.clear();
    items.clear();
    status.clear();
    if (root.empty())
    {
        status = "Cannot locate texture folder";
        return;
    }

    const auto directory = FromUtf8(root) / FromUtf8(folder);
    std::error_code error;
    fs::directory_iterator it(directory, error), end;
    for (; !error && it != end; it.increment(error))
    {
        std::error_code stat;
        if (!it->is_regular_file(stat)) continue;
        const auto& file = it->path();
        const bool material = SDL_strcasecmp(Utf8(file.extension()).c_str(), ".bmat") == 0;
        if (material || IsImage(file))
            items.push_back({Utf8(file.lexically_relative(FromUtf8(root))), Utf8(material ? file.stem() : file.filename()), material});
    }
    if (error && error != std::errc::no_such_file_or_directory) status = "Cannot read texture folder";
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.path < b.path; });
    previews.resize(items.size());
}

std::string TextureCatalog::NewMaterialName() const
{
    std::string name = "New Material";
    auto inUse = [&](const Item& item) { return item.material && SDL_strcasecmp(item.name.c_str(), name.c_str()) == 0; };
    for (int number = 2; std::any_of(items.begin(), items.end(), inUse); ++number)
    {
        name = "New Material " + std::to_string(number);
    }
    return name;
}

bool TextureCatalog::CreateMaterial(const std::string& name, std::string& createdPath, std::string& error)
{
    if (!ValidMaterialName(name))
    {
        error = "Enter a valid material filename.";
        return false;
    }
    if (root.empty())
    {
        error = "Cannot locate the texture folder.";
        return false;
    }
    const auto base = FromUtf8(root);
    const auto directory = base / FromUtf8(folder);
    std::error_code ec;
    // Permit a new project to start with an empty textures directory.
    if (folder.empty()) fs::create_directories(base, ec);
    const auto filename = name + ".bmat";
    fs::directory_iterator it(directory, ec), end;
    for (; !ec && it != end; it.increment(ec))
    {
        if (SDL_strcasecmp(Utf8(it->path().filename()).c_str(), filename.c_str()) == 0)
        {
            error = "That material name is already in use.";
            return false;
        }
    }
    if (ec)
    {
        error = "Cannot read the destination folder.";
        return false;
    }

    const auto file = directory / FromUtf8(filename);
    const auto path = Utf8(file);
    SDL_IOStream* output = SDL_IOFromFile(path.c_str(), "wbx");
    if (!output)
    {
        error = std::string("Cannot create material: ") + SDL_GetError();
        return false;
    }
    constexpr char contents[] = "{\n  \"version\": 1\n}\n";
    const bool written = SDL_WriteIO(output, contents, sizeof(contents) - 1) == sizeof(contents) - 1;
    if (!SDL_CloseIO(output) || !written)
    {
        error = "Could not finish writing the material.";
        fs::remove(file, ec);
        return false;
    }
    createdPath = Utf8(file.lexically_relative(base));
    ReadFolder();
    return true;
}

SDL_Surface* TextureCatalog::Thumbnail(std::size_t index)
{
    if (items.at(index).material) return nullptr;
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
