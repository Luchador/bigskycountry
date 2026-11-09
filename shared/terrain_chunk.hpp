#pragma once

#include "mesh.hpp"

// A rigid, mesh-backed terrain chunk. Its mesh is authored at world scale.
class TerrainChunk : private MeshPlacement
{
public:
    using MeshPlacement::GetTransform;
    using MeshPlacement::Instance;
    using MeshPlacement::SetMesh;

    bool SetTransform(const ObjectTransform& t)
    {
        if (t.scale != Vector3{1, 1, 1})
        {
            return SDL_SetError("Terrain chunks use unit scale");
        }
        return MeshPlacement::SetTransform(t);
    }
};
