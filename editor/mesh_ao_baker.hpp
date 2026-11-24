#pragma once
#include "mesh_ao.hpp"

struct MeshAOBakeSettings
{
    int samples = 64;
    double distance = 3; // Model-space meters, before an instance's scale.
    double bias = .01;
    double strength = 1;
    bool twoSidedFoliage = true;
    bool Valid() const;
};

struct MeshAOBakeResult
{
    std::shared_ptr<MeshAOData> data;
    std::string error;
    bool cancelled = false;
};

// CPU-only, cancellable worker entry. Slot assignments are a snapshot taken on
// the UI thread; geometry is immutable. No rendering or asset writes here.
MeshAOBakeResult BakeMeshAO(const Mesh& mesh, const std::vector<MeshMaterialSlot>& slots,
    MeshAOBakeSettings settings, const std::atomic<bool>& cancel,
    std::atomic<std::size_t>& completed, std::atomic<std::size_t>& total);
