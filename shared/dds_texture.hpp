#pragma once

#include <SDL3/SDL_gpu.h>
#include <array>
#include <string>
#include <vector>

// Single-layer, 2D DDS files produced by the material texture cooker.
class DdsTexture
{
public:
    enum class Format : Uint32 { Packed = 98, BaseColor = 99, Normal = 83, Scalar = 80 }; // DXGI format IDs.
    static std::string CookedPath(const std::string& source, Format format, bool preserveCoverage = false);
    // Read the full-resolution RGBA image used by the sky sprites.
    static SDL_Surface* LoadColorSurface(const std::string& source);
    bool Load(const std::string& path, Format expected);
    // Foliage preserves coverage at the shader's 0.5 cutoff (alpha for color, red for opacity).
    bool FromImage(SDL_Surface* rgba, Format expected, bool preserveCoverage = false);
    // Encoder input with the complete, already-filtered mip chain. No GPU required.
    bool SaveMipSource(const std::string& file) const;
    void Solid(Format expected, std::array<Uint8, 4> rgba);
    // Decode the block data for CPU sampling. Upload can use either representation.
    void Decode();
    std::size_t DataBytes() const;
    SDL_GPUTexture* Upload(SDL_GPUDevice* device);
    std::array<float, 4> Sample(float u, float v, float footprint) const;

private:
    struct Mip
    {
        Uint32 width, height, offset;
        std::vector<Uint8> pixels;
    };
    Format format{};
    std::vector<Mip> mips;
    std::vector<Uint8> blocks;
    bool CoverageMips(SDL_Surface* rgba);
    Uint32 BlockBytes() const { return format == Format::Scalar ? 8 : 16; }
    Uint32 Channels() const { return format == Format::Scalar ? 1 : format == Format::Normal ? 2 : 4; }
};
