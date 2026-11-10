#pragma once

#include <SDL3/SDL_gpu.h>
#include <array>
#include <string>
#include <vector>

// Single-layer, 2D DDS files produced by the material texture cooker.
class DdsTexture
{
public:
    enum class Format : Uint32 { BaseColor = 99, Normal = 83, Scalar = 80 }; // DXGI format IDs.
    bool Load(const std::string& path, Format expected);
    // Decode the block data for CPU sampling. Upload can use either representation.
    void Decode();
    SDL_GPUTexture* Upload(SDL_GPUDevice* device);
    std::array<float, 3> Sample(float u, float v, float footprint) const;

private:
    struct Mip
    {
        Uint32 width, height, offset;
        std::vector<Uint8> pixels;
    };
    Format format{};
    std::vector<Mip> mips;
    std::vector<Uint8> blocks;
    Uint32 BlockBytes() const { return format == Format::Scalar ? 8 : 16; }
    Uint32 Channels() const { return format == Format::BaseColor ? 4 : format == Format::Normal ? 2 : 1; }
};
