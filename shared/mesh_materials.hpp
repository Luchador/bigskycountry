#pragma once
#include "mesh.hpp"

namespace MeshMaterialIO
{
    // Versioned sidecar next to the model: model.glb.bmesh or model.fbx.bmesh. Geometry stays untouched.
    bool Load(const std::string& file, std::vector<MeshMaterialSlot>& slots);
    bool Save(const std::string& file, const std::vector<MeshMaterialSlot>& slots);
    bool Bind(Mesh& mesh, AssetRoot root);
    bool Assign(const Mesh& mesh, std::size_t slot, const std::string& material);
}
