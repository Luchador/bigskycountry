#include "mesh.hpp"

#include <cmath>
#include <utility>

bool MeshTransform::Valid() const
{
    const auto finite = [](Vector3 v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    };
    const double determinant = Determinant();
    return finite(origin) && finite(x) && finite(y) && finite(z) &&
           std::isfinite(determinant) && determinant != 0.0;
}

MeshTransform ObjectTransform::ToMeshTransform() const
{
    const double cx = std::cos(rotation.x), sx = std::sin(rotation.x);
    const double cy = std::cos(rotation.y), sy = std::sin(rotation.y);
    const double cz = std::cos(rotation.z), sz = std::sin(rotation.z);

    // Columns of Rz * Ry, before the local X rotation and scale.
    const Vector3 x{cz * cy, sz * cy, -sy};
    const Vector3 y{-sz, cz, 0};
    const Vector3 z{cz * sy, sz * sy, cy};
    return {position, x * scale.x, (y * cx + z * sx) * scale.y, (z * cx - y * sx) * scale.z};
}

bool MeshPlacement::SetTransform(const ObjectTransform& value)
{
    const MeshTransform matrix = value.ToMeshTransform();
    if (!matrix.Valid())
    {
        return SDL_SetError("Object transform must be finite with nonzero scale");
    }
    transform = value;
    instance.transform = matrix;
    return true;
}

void MeshPlacement::SetMesh(std::shared_ptr<const Mesh> mesh)
{
    instance.mesh = std::move(mesh);
}
