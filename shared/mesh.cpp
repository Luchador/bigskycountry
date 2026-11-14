#include "mesh.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

void Bounds3::Include(Vector3 p)
{
    min = {std::min(min.x, p.x), std::min(min.y, p.y), std::min(min.z, p.z)};
    max = {std::max(max.x, p.x), std::max(max.y, p.y), std::max(max.z, p.z)};
}

void Mesh::UpdateBounds()
{
    bounds = {};
    for (const MeshVertex& vertex : vertices) bounds.Include(vertex.position);
}

MeshTransform MeshTransform::AxisRotation(Vector3 unitAxis, double radians)
{
    const double cosine = std::cos(radians), sine = std::sin(radians);
    const auto rotate = [&](Vector3 v)
    {
        return v * cosine + Math::Cross(unitAxis, v) * sine + unitAxis * (Math::Dot(unitAxis, v) * (1.0 - cosine));
    };
    return {{}, rotate({1, 0, 0}), rotate({0, 1, 0}), rotate({0, 0, 1})};
}

MeshTransform MeshTransform::NormalTransform() const
{
    const double sign = Determinant() < 0 ? -1.0 : 1.0;
    return {{}, Math::Cross(y, z) * sign, Math::Cross(z, x) * sign, Math::Cross(x, y) * sign};
}

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

void ObjectTransform::Rotate(Vector3 unitAxis, double radians)
{
    if (radians == 0.0)
    {
        return;
    }
    const MeshTransform basis = ObjectTransform{{}, rotation}.ToMeshTransform();
    const MeshTransform turn = MeshTransform::AxisRotation(unitAxis, radians);
    const Vector3 x = turn.Direction(basis.x);
    const Vector3 y = turn.Direction(basis.y);
    const Vector3 z = turn.Direction(basis.z);

    // Recover the same Rz * Ry * Rx representation used by ToMeshTransform.
    const double horizontal = std::hypot(x.x, x.y);
    const double pitch = std::atan2(-x.z, horizontal);
    constexpr double pi = std::numbers::pi;
    const auto difference = [&](Vector3 angles)
    {
        return Vector3{std::remainder(angles.x - rotation.x, 2 * pi),
                       std::remainder(angles.y - rotation.y, 2 * pi),
                       std::remainder(angles.z - rotation.z, 2 * pi)};
    };
    // At a Y pole, X and Z are coupled. Keep the previous Z angle there.
    const double heading = horizontal > 1.0e-12 ? std::atan2(x.y, x.x) : rotation.z;
    const Vector3 headingY{-std::sin(heading), std::cos(heading), 0};
    const double roll = std::atan2(-Math::Dot(headingY, z), Math::Dot(headingY, y));
    // Choose the equivalent angles closest to the starting inspector values.
    const Vector3 a = difference({roll, pitch, heading});
    const Vector3 b = difference({roll + pi, pi - pitch, heading + pi});
    rotation = rotation + (Math::Dot(a, a) <= Math::Dot(b, b) ? a : b);
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
