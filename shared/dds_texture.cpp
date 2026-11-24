#include "asset_paths.hpp"
#include "dds_texture.hpp"
#include <SDL3/SDL.h>
#define BCDEC_IMPLEMENTATION
#include <bcdec.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

namespace
{
    Uint32 Read32(const Uint8* p)
    {
        Uint32 value;
        std::memcpy(&value, p, sizeof(value));
        return SDL_Swap32LE(value);
    }

    constexpr Uint32 Blocks(Uint32 length) { return (length + 3) / 4; }

    const auto linearColor = []
    {
        std::array<float, 256> values{};
        for (size_t i = 0; i < values.size(); ++i)
        {
            const float c = static_cast<float>(i) / 255;
            values[i] = c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
        }
        return values;
    }();
}

std::string DdsTexture::CookedPath(const std::string& source, Format format, bool preserveCoverage)
{
    // A source can supply several material slots with different channel layouts.
    const char* folder = format == Format::BaseColor ? "BC7_UNORM_SRGB" :
                         format == Format::Packed ? "BC7_UNORM" :
                         format == Format::Normal ? "BC5_UNORM" : "BC4_UNORM";
    return AssetPaths::Root(AssetRoot::Cooked) + "textures/" + (preserveCoverage ? "foliage/" : "") + folder + "/" + source + ".dds";
}

SDL_Surface* DdsTexture::LoadColorSurface(const std::string& source)
{
    DdsTexture texture;
    if (!texture.Load(CookedPath(source, Format::BaseColor), Format::BaseColor)) return nullptr;
    texture.mips.resize(1); // Only the base level is needed for SDL's sprite texture.
    texture.Decode();
    const auto& mip = texture.mips.front();
    SDL_Surface* surface = SDL_CreateSurface(mip.width, mip.height, SDL_PIXELFORMAT_RGBA32);
    if (surface)
    {
        for (Uint32 y = 0; y < mip.height; ++y)
            std::memcpy(static_cast<Uint8*>(surface->pixels) + size_t(y) * surface->pitch,
                        mip.pixels.data() + size_t(y) * mip.width * 4, size_t(mip.width) * 4);
    }
    return surface;
}

bool DdsTexture::Load(const std::string& path, Format expected)
{
    size_t size = 0;
    std::unique_ptr<void, decltype(&SDL_free)> file(SDL_LoadFile(path.c_str(), &size), SDL_free);
    if (!file) return false;
    const auto* data = static_cast<const Uint8*>(file.get());
    constexpr size_t header = 148;
    if (size < header || std::memcmp(data, "DDS ", 4) || Read32(data + 4) != 124 ||
        Read32(data + 76) != 32 || std::memcmp(data + 84, "DX10", 4) ||
        Read32(data + 128) != static_cast<Uint32>(expected) || Read32(data + 132) != 3 ||
        Read32(data + 136) != 0 || Read32(data + 140) != 1)
    {
        return SDL_SetError("Expected a 2D DX10 DDS of format %u", static_cast<unsigned>(expected));
    }
    Uint32 w = Read32(data + 16), h = Read32(data + 12);
    const Uint32 count = Read32(data + 28);
    if (!w || !h || w > 16384 || h > 16384 || !count || count > 15)
    {
        return SDL_SetError("Invalid DDS dimensions or mip count");
    }
    std::vector<Mip> levels;
    size_t offset = 0;
    const Uint32 blockBytes = expected == Format::Scalar ? 8 : 16;
    for (Uint32 level = 0; level < count; ++level)
    {
        const size_t bytes = size_t(Blocks(w)) * Blocks(h) * blockBytes;
        if (bytes > size - header - offset || (w == 1 && h == 1 && level + 1 != count))
        {
            return SDL_SetError("Invalid DDS mip chain");
        }
        levels.push_back({w, h, static_cast<Uint32>(offset), {}});
        offset += bytes;
        w = std::max(1u, w / 2);
        h = std::max(1u, h / 2);
    }
    if (offset != size - header || levels.back().width != 1 || levels.back().height != 1)
    {
        return SDL_SetError("Expected a complete DDS mip chain");
    }
    blocks.assign(data + header, data + size);
    mips = std::move(levels);
    format = expected;
    return true;
}

void DdsTexture::Solid(Format expected, std::array<Uint8, 4> rgba)
{
    format = expected;
    blocks.clear();
    mips = {{1, 1, 0, std::vector<Uint8>(rgba.begin(), rgba.begin() + Channels())}};
}

bool DdsTexture::FromImage(SDL_Surface* rgba, Format expected, bool preserveCoverage)
{
    if (!rgba || rgba->format != SDL_PIXELFORMAT_RGBA32 || rgba->w <= 0 || rgba->h <= 0 || rgba->w > 16384 || rgba->h > 16384)
        return SDL_SetError("Invalid material source image");
    blocks.clear();
    mips.clear();
    format = expected;
    if (preserveCoverage)
    {
        if (format != Format::BaseColor && format != Format::Scalar)
            return SDL_SetError("Coverage mipmaps require base color or opacity");
        return CoverageMips(rgba);
    }
    std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> scaled(nullptr, SDL_DestroySurface);
    for (;;)
    {
        const auto w = static_cast<Uint32>(rgba->w), h = static_cast<Uint32>(rgba->h);
        auto& mip = mips.emplace_back(Mip{w, h, 0, {}});
        const auto channels = Channels();
        mip.pixels.resize(size_t(w) * h * channels);
        for (Uint32 y = 0; y < h; ++y)
        {
            const auto* src = static_cast<const Uint8*>(rgba->pixels) + size_t(y) * rgba->pitch;
            auto* dst = mip.pixels.data() + size_t(y) * w * channels;
            if (channels == 4)
                std::memcpy(dst, src, w * 4);
            else
            {
                for (Uint32 x = 0; x < w; ++x)
                    std::memcpy(dst + x * channels, src + x * 4, channels);
            }
        }
        if (w == 1 && h == 1) return true;
        scaled.reset(SDL_ScaleSurface(rgba, std::max(1u, w / 2), std::max(1u, h / 2), SDL_SCALEMODE_LINEAR));
        rgba = scaled.get();
        if (!rgba) return false;
    }
}

bool DdsTexture::CoverageMips(SDL_Surface* rgba)
{
    const Uint32 channels = Channels(), alpha = channels - 1;
    Uint32 width = rgba->w, height = rgba->h;
    std::vector<float> data(size_t(width) * height * channels);
    for (Uint32 y = 0; y < height; ++y)
    {
        const auto* row = static_cast<const Uint8*>(rgba->pixels) + size_t(y) * rgba->pitch;
        for (Uint32 x = 0; x < width; ++x)
            for (Uint32 c = 0; c < channels; ++c)
                data[(size_t(y) * width + x) * channels + c] = c == alpha ? row[x * 4 + c] / 255.f : linearColor[row[x * 4 + c]];
    }
    const auto quantize = [](float value) { return std::round(std::clamp(value, 0.f, 1.f) * 255.f) / 255.f; };
    const auto coverage = [&](float scale)
    {
        std::size_t covered = 0;
        // Sample four positions per texel, with the same repeat and bilinear
        // filtering as the renderer. Counting texel centers alone underestimates
        // how quickly tiny leaf silhouettes disappear between texel centers.
        for (Uint32 y = 0; y < height; ++y)
            for (Uint32 x = 0; x < width; ++x)
            {
                const auto a = [&](Uint32 px, Uint32 py) {
                    return quantize(data[(size_t(py % height) * width + px % width) * channels + alpha] * scale);
                };
                const float a00 = a(x, y), a10 = a(x + 1, y), a01 = a(x, y + 1), a11 = a(x + 1, y + 1);
                for (float v : {.25f, .75f})
                    for (float u : {.25f, .75f})
                        covered += std::lerp(std::lerp(a00, a10, u), std::lerp(a01, a11, u), v) >= .5f;
            }
        return double(covered) / (double(width) * height * 4);
    };
    const double target = coverage(1);
    for (;;)
    {
        float scale = 1;
        if (!mips.empty() && target > 0 && target < 1)
        {
            double bestError = std::abs(coverage(1) - target);
            float low = 0, high = 1;
            while (high < 256 && coverage(high) < target) high *= 2;
            const auto consider = [&](float candidate, double value) {
                const double error = std::abs(value - target);
                if (error < bestError || (error == bestError && std::abs(candidate - 1) < std::abs(scale - 1)))
                { scale = candidate; bestError = error; }
            };
            consider(high, coverage(high));
            for (int i = 0; i < 12; ++i)
            {
                const float middle = (low + high) * .5f;
                const double value = coverage(middle);
                consider(middle, value);
                if (value < target) low = middle; else high = middle;
            }
        }
        auto& mip = mips.emplace_back(Mip{width, height, 0, {}});
        mip.pixels.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i)
        {
            float value = data[i];
            if (i % channels == alpha) value *= scale;
            else value = value <= .0031308f ? value * 12.92f : 1.055f * std::pow(value, 1 / 2.4f) - .055f;
            mip.pixels[i] = static_cast<Uint8>(std::round(std::clamp(value, 0.f, 1.f) * 255.f));
        }
        if (width == 1 && height == 1) return true;
        const Uint32 nextWidth = std::max(1u, width / 2), nextHeight = std::max(1u, height / 2);
        std::vector<float> next(size_t(nextWidth) * nextHeight * channels);
        // Area filtering also covers the last row/column of odd-sized images.
        // Keep unscaled float data for the next level: coverage corrections must
        // not compound, and small nonzero alpha must survive quantization.
        for (Uint32 y = 0; y < nextHeight; ++y)
            for (Uint32 x = 0; x < nextWidth; ++x)
            {
                const double x0 = double(x) * width / nextWidth, x1 = double(x + 1) * width / nextWidth;
                const double y0 = double(y) * height / nextHeight, y1 = double(y + 1) * height / nextHeight;
                for (Uint32 sy = Uint32(y0); sy < std::ceil(y1) && sy < height; ++sy)
                    for (Uint32 sx = Uint32(x0); sx < std::ceil(x1) && sx < width; ++sx)
                    {
                        const float weight = float((std::min(x1, double(sx + 1)) - std::max(x0, double(sx))) *
                                                   (std::min(y1, double(sy + 1)) - std::max(y0, double(sy))) / ((x1 - x0) * (y1 - y0)));
                        for (Uint32 c = 0; c < channels; ++c)
                            next[(size_t(y) * nextWidth + x) * channels + c] += data[(size_t(sy) * width + sx) * channels + c] * weight;
                    }
            }
        data = std::move(next);
        width = nextWidth; height = nextHeight;
    }
}

bool DdsTexture::SaveMipSource(const std::string& file) const
{
    if (mips.empty() || !blocks.empty() || (format != Format::BaseColor && format != Format::Scalar))
        return SDL_SetError("Expected uncompressed foliage mipmaps");
    // DX10 DDS, tightly packed rows; texconv keeps an existing complete chain.
    std::array<Uint32, 37> header{};
    header[0] = 0x20534444; header[1] = 124; header[2] = 0x2100f;
    header[3] = mips[0].height; header[4] = mips[0].width; header[5] = mips[0].width * Channels();
    header[7] = static_cast<Uint32>(mips.size());
    header[19] = 32; header[20] = 4; header[21] = 0x30315844; // DX10
    header[27] = mips.size() > 1 ? 0x401008 : 0x1000;
    header[32] = format == Format::BaseColor ? 29 : 61; // RGBA8 sRGB or R8 linear.
    header[33] = 3; header[35] = 1; // Texture2D, one layer.
    for (auto& value : header) value = SDL_Swap32LE(value);
    SDL_IOStream* out = SDL_IOFromFile(file.c_str(), "wb");
    if (!out) return false;
    bool written = SDL_WriteIO(out, header.data(), sizeof(header)) == sizeof(header);
    for (const auto& mip : mips)
        if (written) written = SDL_WriteIO(out, mip.pixels.data(), mip.pixels.size()) == mip.pixels.size();
    const bool closed = SDL_CloseIO(out);
    return written && closed;
}

std::size_t DdsTexture::DataBytes() const
{
    std::size_t bytes = blocks.size();
    for (const auto& mip : mips) bytes += mip.pixels.size();
    return bytes;
}

void DdsTexture::Decode()
{
    if (blocks.empty()) return;
    const Uint32 channels = Channels();
    for (Mip& mip : mips)
    {
        mip.pixels.resize(size_t(mip.width) * mip.height * channels);
        const Uint8* src = blocks.data() + mip.offset;
        for (Uint32 y = 0; y < mip.height; y += 4)
        {
            for (Uint32 x = 0; x < mip.width; x += 4, src += BlockBytes())
            {
                alignas(8) Uint8 tile[64];
                switch (format)
                {
                    case Format::Packed:
                    case Format::BaseColor: bcdec_bc7(src, tile, 16); break;
                    case Format::Normal: bcdec_bc5(src, tile, 8); break;
                    case Format::Scalar: bcdec_bc4(src, tile, 4); break;
                }
                for (Uint32 row = 0; row < std::min(4u, mip.height - y); ++row)
                {
                    auto* dst = mip.pixels.data() + (size_t(y + row) * mip.width + x) * channels;
                    std::memcpy(dst, tile + row * 4 * channels, std::min(4u, mip.width - x) * channels);
                }
            }
        }
    }
    blocks.clear();
    blocks.shrink_to_fit();
}

SDL_GPUTexture* DdsTexture::Upload(SDL_GPUDevice* device)
{
    if (mips.empty())
    {
        SDL_SetError("No DDS texture loaded");
        return nullptr;
    }
    SDL_GPUTextureCreateInfo info{};
    info.type = SDL_GPU_TEXTURETYPE_2D;
    info.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;
    info.format = format == Format::BaseColor ? SDL_GPU_TEXTUREFORMAT_BC7_RGBA_UNORM_SRGB :
                  format == Format::Packed ? SDL_GPU_TEXTUREFORMAT_BC7_RGBA_UNORM :
                  format == Format::Normal ? SDL_GPU_TEXTUREFORMAT_BC5_RG_UNORM : SDL_GPU_TEXTUREFORMAT_BC4_R_UNORM;
    // D3D requires a block-aligned base level, even when the BC format is supported.
    const bool blockAligned = mips.front().width % 4 == 0 && mips.front().height % 4 == 0;
    const bool compressed = blockAligned && !blocks.empty() && SDL_GPUTextureSupportsFormat(device, info.format, info.type, info.usage);
    if (!compressed)
    {
        info.format = format == Format::BaseColor ? SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM_SRGB :
                      format == Format::Packed ? SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM :
                      format == Format::Normal ? SDL_GPU_TEXTUREFORMAT_R8G8_UNORM : SDL_GPU_TEXTUREFORMAT_R8_UNORM;
        Decode();
    }
    info.width = mips.front().width;
    info.height = mips.front().height;
    info.layer_count_or_depth = 1;
    info.num_levels = static_cast<Uint32>(mips.size());

    // D3D12 footprints require 256-byte rows and 512-byte subresource offsets.
    struct Footprint { Uint32 offset, pitch, rowBytes, rows; };
    std::vector<Footprint> layouts;
    Uint32 total = 0;
    for (const Mip& mip : mips)
    {
        const Uint32 rowBytes = compressed ? Blocks(mip.width) * BlockBytes() : mip.width * Channels();
        const Uint32 rows = compressed ? Blocks(mip.height) : mip.height;
        total = (total + 511) & ~511u;
        const Uint32 pitch = (rowBytes + 255) & ~255u;
        layouts.push_back({total, pitch, rowBytes, rows});
        total += pitch * rows;
    }
    SDL_GPUTransferBufferCreateInfo transfer{SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, total, 0};
    SDL_GPUTransferBuffer* buffer = SDL_CreateGPUTransferBuffer(device, &transfer);
    if (!buffer) return nullptr;
    auto* mapped = static_cast<Uint8*>(SDL_MapGPUTransferBuffer(device, buffer, false));
    if (!mapped)
    {
        SDL_ReleaseGPUTransferBuffer(device, buffer);
        return nullptr;
    }
    for (size_t i = 0; i < mips.size(); ++i)
    {
        const auto& f = layouts[i];
        const auto& mip = mips[i];
        const Uint8* src = compressed ? blocks.data() + mip.offset : mip.pixels.data();
        for (Uint32 row = 0; row < f.rows; ++row)
        {
            std::memcpy(mapped + f.offset + row * f.pitch, src + row * f.rowBytes, f.rowBytes);
        }
    }
    SDL_UnmapGPUTransferBuffer(device, buffer);
    SDL_GPUTexture* texture = SDL_CreateGPUTexture(device, &info);
    SDL_GPUCommandBuffer* commands = texture ? SDL_AcquireGPUCommandBuffer(device) : nullptr;
    SDL_GPUCopyPass* copy = commands ? SDL_BeginGPUCopyPass(commands) : nullptr;
    bool submitted = false;
    if (copy)
    {
        for (Uint32 i = 0; i < info.num_levels; ++i)
        {
            const auto& f = layouts[i];
            const auto& mip = mips[i];
            SDL_GPUTextureTransferInfo src{buffer, f.offset, compressed ? f.pitch / BlockBytes() * 4 : f.pitch / Channels(),
                                          compressed ? f.rows * 4 : f.rows};
            SDL_GPUTextureRegion dst{texture, i, 0, 0, 0, 0, mip.width, mip.height, 1};
            SDL_UploadToGPUTexture(copy, &src, &dst, false);
        }
        SDL_EndGPUCopyPass(copy);
        submitted = SDL_SubmitGPUCommandBuffer(commands);
    }
    else if (commands)
    {
        SDL_CancelGPUCommandBuffer(commands);
    }
    SDL_ReleaseGPUTransferBuffer(device, buffer);
    if (!submitted && texture)
    {
        SDL_ReleaseGPUTexture(device, texture);
        return nullptr;
    }
    return texture;
}

std::array<float, 4> DdsTexture::Sample(float u, float v, float footprint) const
{
    const float lod = std::log2(std::max(1.0f, footprint * std::max(mips.front().width, mips.front().height)));
    const auto& mip = mips[static_cast<size_t>(std::min(lod + 0.5f, float(mips.size() - 1)))];
    const float x = (u - std::floor(u)) * mip.width - 0.5f;
    const float y = (v - std::floor(v)) * mip.height - 0.5f;
    const int ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
    auto texel = [&](int px, int py)
    {
        const Uint32 col = (px + static_cast<int>(mip.width)) % static_cast<int>(mip.width);
        const Uint32 row = (py + static_cast<int>(mip.height)) % static_cast<int>(mip.height);
        return mip.pixels.data() + (size_t(row) * mip.width + col) * Channels();
    };
    const auto a = texel(ix, iy), b = texel(ix + 1, iy), c = texel(ix, iy + 1), d = texel(ix + 1, iy + 1);
    std::array<float, 4> value{0, 0, 0, 1};
    for (Uint32 i = 0; i < Channels(); ++i)
    {
        auto channel = [&](Uint8 byte) { return format == Format::BaseColor && i < 3 ? linearColor[byte] : byte / 255.0f; };
        value[i] = std::lerp(std::lerp(channel(a[i]), channel(b[i]), x - ix),
                             std::lerp(channel(c[i]), channel(d[i]), x - ix), y - iy);
    }
    return value;
}
