#include "texture_loader.hpp"
#include <SDL3_image/SDL_image.h>

#include <algorithm>
#include <memory>
#include <span>
#include <string>

namespace
{
    using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;

    bool StripFooter(std::span<const Uint8>& data, size_t imageStart)
    {
        constexpr char signature[] = "TRUEVISION-XFILE.";
        constexpr size_t footerSize = 26;

        if (data.size() < footerSize || SDL_memcmp(data.data() + data.size() - sizeof(signature), signature, sizeof(signature)) != 0)
        {
            return true; // Original TGA files have no footer.
        }

        // TGA 2.0 may append extension and developer areas after the image packets.
        const size_t footerStart = data.size() - footerSize;
        size_t imageEnd = footerStart;

        for (size_t field = 0; field < 8; field += 4)
        {
            Uint32 offset;
            SDL_memcpy(&offset, data.data() + footerStart + field, sizeof(offset));
            offset = SDL_Swap32LE(offset);

            if (offset != 0)
            {
                if (offset < imageStart || offset >= footerStart)
                {
                    return SDL_SetError("Invalid TGA footer offset");
                }
                imageEnd = std::min(imageEnd, static_cast<size_t>(offset));
            }
        }

        data = data.first(imageEnd);

        return true;
    }

    bool ReadPackets(std::span<const Uint8> data, SDL_Surface* surface, int bytesPerPixel, bool topOrigin, bool rightOrigin)
    {
        const size_t pixelCount = static_cast<size_t>(surface->w) * surface->h;
        size_t written = 0;

        while (written < pixelCount)
        {
            if (data.empty())
            {
                return SDL_SetError("Missing TGA packet");
            }

            const bool repeat = (data[0] & 0x80) != 0;
            const size_t count = (data[0] & 0x7f) + 1;
            const size_t payloadSize = (repeat ? 1 : count) * bytesPerPixel;
            data = data.subspan(1);

            if (count > pixelCount - written || data.size() < payloadSize)
            {
                return SDL_SetError("Invalid or truncated TGA packet");
            }

            for (size_t i = 0; i < count; ++i, ++written)
            {
                const int column = static_cast<int>(written % surface->w);
                const int row = static_cast<int>(written / surface->w);
                const int x = rightOrigin ? surface->w - 1 - column : column;
                const int y = topOrigin ? row : surface->h - 1 - row;
                auto* destination = static_cast<Uint8*>(surface->pixels) + static_cast<size_t>(y) * surface->pitch;
                const Uint8* pixel = data.data() + (repeat ? 0 : i * bytesPerPixel);
                SDL_memcpy(destination + x * bytesPerPixel, pixel, bytesPerPixel);
            }

            data = data.subspan(payloadSize);
        }

        return true;
    }

    enum class DiscOpacity { SourceAlpha, BrightnessTimesAlpha };

    SDL_Texture* CreateDiscTexture(SDL_Renderer* renderer, SDL_Surface* source, SDL_Rect& bounds, DiscOpacity opacity)
    {
        Surface rgba(SDL_ConvertSurface(source, SDL_PIXELFORMAT_RGBA32), SDL_DestroySurface);
        if (!rgba)
        {
            return nullptr;
        }

        int left = rgba->w, top = rgba->h, right = -1, bottom = -1;
        for (int y = 0; y < rgba->h; ++y)
        {
            auto* pixel = static_cast<Uint8*>(rgba->pixels) + static_cast<size_t>(y) * rgba->pitch;
            for (int x = 0; x < rgba->w; ++x, pixel += 4)
            {
                if (opacity == DiscOpacity::BrightnessTimesAlpha)
                {
                    const int brightness = (54 * pixel[0] + 183 * pixel[1] + 19 * pixel[2] + 128) / 256;
                    pixel[3] = static_cast<Uint8>((brightness * pixel[3] + 127) / 255);
                    pixel[0] = pixel[1] = pixel[2] = 255;
                }
                if (pixel[3] >= 128)
                {
                    left = std::min(left, x);
                    right = std::max(right, x);
                    top = std::min(top, y);
                    bottom = std::max(bottom, y);
                }
            }
        }
        if (right < left)
        {
            SDL_SetError("Mask contains no pixels with at least half opacity");
            return nullptr;
        }
        bounds = {left, top, right - left + 1, bottom - top + 1};
        return SDL_CreateTextureFromSurface(renderer, rgba.get());
    }

    SDL_Surface* DecodeTga(std::span<const Uint8> data, Sint64 maxSize)
    {
        constexpr size_t headerSize = 18;

        if (data.size() < headerSize)
        {
            SDL_SetError("Truncated TGA header");
            return nullptr;
        }

        const int width = data[12] | (data[13] << 8);
        const int height = data[14] | (data[15] << 8);
        const int depth = data[16];
        const Uint8 descriptor = data[17];

        if (data[1] != 0 || data[2] != 10 || (depth != 24 && depth != 32) || (descriptor & 0xc0) != 0)
        {
            SDL_SetError("Expected a non-interleaved 24-bit or 32-bit RLE true-color TGA");
            return nullptr;
        }

        if (width == 0 || height == 0 || (maxSize > 0 && (width > maxSize || height > maxSize)))
        {
            SDL_SetError("Unsupported TGA dimensions: %d x %d", width, height);
            return nullptr;
        }

        const int bytesPerPixel = depth / 8;
        const size_t imageStart = headerSize + data[0]; // Skip the optional image ID.

        if (!StripFooter(data, imageStart))
        {
            return nullptr;
        }

        const size_t pixelCount = static_cast<size_t>(width) * height;
        const size_t minPacketBytes = ((pixelCount + 127) / 128) * (1 + bytesPerPixel);

        if (data.size() < imageStart || data.size() - imageStart < minPacketBytes)
        {
            SDL_SetError("Truncated TGA image data");
            return nullptr;
        }

        const SDL_PixelFormat format = depth == 24 ? SDL_PIXELFORMAT_BGR24 : SDL_PIXELFORMAT_BGRA32;
        Surface surface(SDL_CreateSurface(width, height, format), SDL_DestroySurface);

        if (!surface || !ReadPackets(data.subspan(imageStart), surface.get(), bytesPerPixel, (descriptor & 0x20) != 0, (descriptor & 0x10) != 0))
        {
            return nullptr;
        }

        return surface.release();
    }

    SDL_Surface* LoadTgaFile(const char* path, Sint64 maxSize)
    {
        size_t size = 0;
        std::unique_ptr<void, decltype(&SDL_free)> file(SDL_LoadFile(path, &size), SDL_free);
        SDL_Surface* surface = file ? DecodeTga({static_cast<const Uint8*>(file.get()), size}, maxSize) : nullptr;
        if (!surface)
        {
            const std::string reason = SDL_GetError();
            SDL_SetError("Could not load texture '%s': %s", path, reason.c_str());
        }
        return surface;
    }

    SDL_Surface* LoadTga(const char* relativePath, Sint64 maxSize)
    {
        if (!relativePath)
        {
            SDL_SetError("A texture path is required");
            return nullptr;
        }

        std::string name(relativePath);
        std::replace(name.begin(), name.end(), '\\', '/');

        if (name.empty() || name.front() == '/' || name.find(':') != std::string::npos || ("/" + name + "/").find("/../") != std::string::npos)
        {
            SDL_SetError("Expected a path relative to assets/textures: '%s'", relativePath);
            return nullptr;
        }

        const char* base = SDL_GetBasePath();

        if (!base)
        {
            return nullptr;
        }

        const std::string path = std::string(base) + "assets/textures/" + name;
        return LoadTgaFile(path.c_str(), maxSize);
    }

    SDL_Texture* LoadTexture(SDL_Renderer* renderer, const char* relativePath, SDL_Rect* discBounds, DiscOpacity opacity)
    {
        if (!renderer)
        {
            SDL_SetError("A renderer is required");
            return nullptr;
        }
        const Sint64 maxSize = SDL_GetNumberProperty(SDL_GetRendererProperties(renderer), SDL_PROP_RENDERER_MAX_TEXTURE_SIZE_NUMBER, 0);
        Surface surface(LoadTga(relativePath, maxSize), SDL_DestroySurface);
        if (!surface)
        {
            return nullptr;
        }
        return discBounds ? CreateDiscTexture(renderer, surface.get(), *discBounds, opacity)
                          : SDL_CreateTextureFromSurface(renderer, surface.get());
    }
}

SDL_Surface* TextureLoader::LoadTgaSurface(const char* relativePath)
{
    return LoadTga(relativePath, 0);
}

SDL_Surface* TextureLoader::LoadTgaSurfaceFromFile(const char* path)
{
    return LoadTgaFile(path, 0);
}

SDL_Texture* TextureLoader::Load(SDL_Renderer* renderer, const char* relativePath)
{
    return LoadTexture(renderer, relativePath, nullptr, DiscOpacity::SourceAlpha);
}

SDL_Texture* TextureLoader::Load(SDL_Renderer* renderer, const char* relativePath, SDL_Rect& alphaBounds)
{
    return LoadTexture(renderer, relativePath, &alphaBounds, DiscOpacity::SourceAlpha);
}

SDL_Texture* TextureLoader::LoadMask(SDL_Renderer* renderer, const char* relativePath, SDL_Rect& discBounds)
{
    return LoadTexture(renderer, relativePath, &discBounds, DiscOpacity::BrightnessTimesAlpha);
}

SDL_Surface* TextureLoader::LoadImageFile(const std::string& path)
{
    Surface source(nullptr, SDL_DestroySurface);
    // Use the world loader for RLE true-color TGAs, including right-origin images.
    const char* extension = SDL_strrchr(path.c_str(), '.');
    if (extension && SDL_strcasecmp(extension, ".tga") == 0)
    {
        source.reset(TextureLoader::LoadTgaSurfaceFromFile(path.c_str()));
    }
    if (!source)
    {
        // Keep PNG thumbnails in straight alpha on every platform.
        const bool png = extension && (SDL_strcasecmp(extension, ".png") == 0 || SDL_strcasecmp(extension, ".apng") == 0);
        source.reset(png ? SDL_LoadPNG(path.c_str()) : IMG_Load(path.c_str()));
    }
    if (!source) return nullptr;
    if (source->format != SDL_PIXELFORMAT_RGBA32)
    {
        source.reset(SDL_ConvertSurface(source.get(), SDL_PIXELFORMAT_RGBA32));
        if (!source) return nullptr;
    }
    // WIC and libtiff supply associated alpha. Keep the shared image in straight RGBA.
    if (extension && (SDL_strcasecmp(extension, ".tif") == 0 || SDL_strcasecmp(extension, ".tiff") == 0))
    {
        for (int y = 0; y < source->h; ++y)
        {
            auto* pixel = static_cast<Uint8*>(source->pixels) + y * source->pitch;
            for (int x = 0; x < source->w; ++x, pixel += 4)
                if (pixel[3] && pixel[3] < 255)
                    for (int c = 0; c < 3; ++c) pixel[c] = std::min(255, (pixel[c] * 255 + pixel[3] / 2) / pixel[3]);
        }
    }
    SDL_SetSurfaceBlendMode(source.get(), SDL_BLENDMODE_BLEND);
    return source.release();
}
