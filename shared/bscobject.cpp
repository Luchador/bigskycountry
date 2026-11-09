#include "bscobject.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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

bool BSCObject::SetTransform(const ObjectTransform& value)
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

void BSCObject::SetMesh(std::shared_ptr<const Mesh> mesh)
{
    instance.mesh = std::move(mesh);
}

void BSCObject::Tick(float)
{
}

void BSCObject::TakeDamage(float damage)
{
    if (!invincible && std::isfinite(damage) && damage > 0.0f)
    {
        health = std::max(0.0f, health - damage);
    }
}
