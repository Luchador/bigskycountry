#include "pawn.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

void Pawn::Teleport(const ObjectTransform& transform)
{
    if (SetTransform(transform)) previous = transform;
}

ObjectTransform Pawn::InterpolatedTransform(double fraction) const
{
    const auto& current = GetTransform();
    const double t = std::clamp(fraction, 0.0, 1.0);
    auto result = current;
    result.position = previous.position + (current.position - previous.position) * t;
    const auto angle = [t](double a, double b) { return a + std::remainder(b - a, 2 * std::numbers::pi) * t; };
    result.rotation = {angle(previous.rotation.x, current.rotation.x), angle(previous.rotation.y, current.rotation.y),
                       angle(previous.rotation.z, current.rotation.z)};
    return result;
}

const MeshInstance& Pawn::RenderInstance(double fraction) const
{
    rendered.mesh = Instance().mesh;
    rendered.transform = InterpolatedTransform(fraction).ToMeshTransform();
    return rendered;
}

Vector3 Pawn::Forward() const
{
    const Vector3 forward = Instance().transform.z;
    return Math::Normalize({forward.x, 0, forward.z});
}
