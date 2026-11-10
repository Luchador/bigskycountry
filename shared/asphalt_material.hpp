#pragma once

#include "dds_texture.hpp"
#include <SDL3/SDL_pixels.h>

struct PavementLighting
{
    std::array<float, 3> light{}, view{};
};

class AsphaltMaterial
{
public:
    AsphaltMaterial() = default;
    ~AsphaltMaterial();
    AsphaltMaterial(const AsphaltMaterial&) = delete;
    AsphaltMaterial& operator=(const AsphaltMaterial&) = delete;
    bool Prepare(SDL_GPUDevice* device);
    void Bind(SDL_GPURenderPass* pass) const;
    SDL_FColor Shade(float u, float v, float footprint, const PavementLighting& basis) const;

    static constexpr double tileMeters = 4.0;

private:
    bool Load();
    void Release();
    SDL_GPUDevice* device = nullptr;
    SDL_GPUSampler* sampler = nullptr;
    std::array<SDL_GPUTexture*, 3> gpu{};
    std::array<DdsTexture, 3> cpu;
    bool attempted = false, ready = false;
};
