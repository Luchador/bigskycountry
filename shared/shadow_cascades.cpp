#include "shadow_cascades.hpp"
#include <algorithm>
#include <cmath>

bool ShadowCascade::Intersects(const Bounds3& bounds) const
{
    if (bounds.Empty()) return false;
    const auto c = (bounds.min + bounds.max) * .5 - center;
    const auto h = (bounds.max - bounds.min) * .5;
    const auto overlaps = [&](Vector3 axis, double extent) {
        const double projected = std::abs(axis.x)*h.x + std::abs(axis.y)*h.y + std::abs(axis.z)*h.z;
        return std::abs(Math::Dot(c, axis)) <= extent + projected;
    };
    return overlaps(right, radius) && overlaps(up, radius) && overlaps(forward, depthRadius);
}

std::array<SDL_FColor, 3> ShadowCascade::Rows(Vector3 eye) const
{
    const auto row = [&](Vector3 axis, double scale, double offset) {
        return SDL_FColor{float(axis.x*scale), float(axis.y*scale), float(axis.z*scale),
                          float(Math::Dot(eye-center, axis)*scale + offset)};
    };
    // Conventional shadow depth: nearest light-facing surface wins (LESS).
    return {row(right, 1/radius, 0), row(up, 1/radius, 0), row(forward, .5/depthRadius, .5)};
}

void ShadowCascades::Build(const CameraView& camera, Vector3 lightDirection, double scaleX, double scaleY, double meters)
{
    active = false;
    if (!std::isfinite(meters) || meters <= .1 || scaleX <= 0 || scaleY <= 0 || Math::Length(lightDirection) < .5) return;
    distance = meters;
    const Vector3 forward = Math::Normalize(lightDirection) * -1;
    const Vector3 reference = std::abs(forward.y) > .95 ? Vector3{0,0,1} : Vector3{0,1,0};
    const Vector3 right = Math::Normalize(Math::Cross(reference, forward));
    const Vector3 up = Math::Cross(forward, right);
    constexpr std::array fractions{.025, .10, .35, 1.0};
    double nearDepth = .1;
    for (int i = 0; i < count; ++i)
    {
        auto& cascade = cascades[i];
        cascade.right = right; cascade.up = up; cascade.forward = forward;
        cascade.farDepth = meters * fractions[i];
        // Include the preceding cascade's final 10% for a seamless blend.
        const double near = i ? nearDepth*.9 : .1;
        const double far = cascade.farDepth;
        const double midpoint = (near+far)*.5;
        double radius = std::sqrt(far*far/(scaleX*scaleX) + far*far/(scaleY*scaleY) + (far-midpoint)*(far-midpoint));
        radius = std::ceil(radius*16)/16; // Stable extent when the camera turns.
        radius *= double(resolution)/(resolution-8); // Guard texels for PCF and snapping.
        cascade.radius = radius;
        cascade.texelMeters = 2*radius/resolution;
        auto center = camera.GetPosition() + camera.Forward()*midpoint;
        for (auto axis : {right, up})
        {
            const double coordinate = Math::Dot(center,axis);
            center = center + axis*(std::round(coordinate/cascade.texelMeters)*cascade.texelMeters-coordinate);
        }
        // Extend toward the light so off-screen elevated casters can shade the slice.
        const double reach = std::clamp(meters*.5, 500.0, 4000.0);
        cascade.center = center - forward*(reach*.5);
        cascade.depthRadius = radius + reach*.5;
        nearDepth = far;
    }
    active = true;
}

bool ShadowCascades::Intersects(const Bounds3& bounds) const
{
    if (!active) return false;
    return std::ranges::any_of(cascades, [&](const auto& c) { return c.Intersects(bounds); });
}
