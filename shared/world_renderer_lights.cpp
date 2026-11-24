#include "world_renderer.hpp"
#include "camera_view.hpp"

#include <algorithm>
#include <bit>
#include <cmath>

void WorldRenderer::SubmitSpotLight(SpotLight light)
{
    if (!camera || lights.spots.size() >= maxSpotLights || !std::isfinite(light.range) || light.range <= 0 ||
        !std::isfinite(light.innerCos) || !std::isfinite(light.outerCos) ||
        light.outerCos <= 0 || light.innerCos <= light.outerCos || light.innerCos > 1) return;
    light.direction = Math::Normalize(light.direction);
    if (light.direction == Vector3{}) return;
    // The emitter may be behind the camera or off screen while its beam is visible.
    const Vector3 center = light.position + light.direction * (light.range * .5);
    const double radius = light.range * std::sqrt(1.25 - light.outerCos);
    if (!SphereVisible(center, radius)) return;
    light.position = light.position - camera->GetPosition();
    lights.spots.push_back(light);
}

void WorldRenderer::SubmitEmissiveTriangle(Vector3 a, Vector3 b, Vector3 c, SDL_FColor color)
{
    // Unlike editor overlays, luminous surfaces still receive distance fog.
    const size_t first = vertices.size();
    SubmitTriangle(a,b,c,color);
    for (size_t i = first; i < vertices.size(); ++i) vertices[i].lit = 2;
}

std::vector<SDL_FColor> WorldRenderer::BuildSpotLightData() const
{
    const size_t count = lights.spots.size();
    if (!count) return {SDL_FColor{}}; // A valid binding even when lighting is disabled.
    const int columns = (viewport.w + lightTilePixels - 1) / lightTilePixels;
    const int rows = (viewport.h + lightTilePixels - 1) / lightTilePixels;
    const size_t groups = (count + 127) / 128; // One uint4 bitset per group of 128 lights.
    std::vector<std::array<Uint32,4>> masks(size_t(columns) * rows * groups);
    std::vector<SDL_FColor> data;
    data.reserve(count * 3 + masks.size());
    const auto pack = [](Vector3 v, double w) { return SDL_FColor{float(v.x),float(v.y),float(v.z),float(w)}; };
    for (size_t i = 0; i < count; ++i)
    {
        const auto& spot = lights.spots[i];
        data.push_back(pack(spot.position, spot.range));
        data.push_back(pack(spot.direction, spot.outerCos));
        data.push_back(pack(spot.intensity, 1 / (spot.innerCos - spot.outerCos)));

        const Vector3 p = camera->WorldDirectionToView(spot.position);
        const Vector3 d = camera->WorldDirectionToView(spot.direction);
        // Project a conservative view-space box enclosing the entire cone. Do
        // not clip just its emitter; near-plane intersections cover all tiles.
        const Vector3 end = p + d * spot.range;
        const double radius = spot.range * std::sqrt(1 - spot.outerCos * spot.outerCos) / spot.outerCos;
        const Vector3 extent{radius * std::sqrt(std::max(0.0,1-d.x*d.x)),
                             radius * std::sqrt(std::max(0.0,1-d.y*d.y)),
                             radius * std::sqrt(std::max(0.0,1-d.z*d.z))};
        const Vector3 lo{std::min(p.x,end.x-extent.x),std::min(p.y,end.y-extent.y),std::min(p.z,end.z-extent.z)};
        const Vector3 hi{std::max(p.x,end.x+extent.x),std::max(p.y,end.y+extent.y),std::max(p.z,end.z+extent.z)};
        if (hi.z < worldNearDepth) continue;
        int left = 0, right = columns-1, top = 0, bottom = rows-1;
        if (lo.z > worldNearDepth)
        {
            const double x0 = std::min(lo.x/lo.z,lo.x/hi.z)*focal + viewport.w*.5;
            const double x1 = std::max(hi.x/lo.z,hi.x/hi.z)*focal + viewport.w*.5;
            const double y0 = viewport.h*.5 - std::max(hi.y/lo.z,hi.y/hi.z)*focal;
            const double y1 = viewport.h*.5 - std::min(lo.y/lo.z,lo.y/hi.z)*focal;
            if (x1 < 0 || x0 >= viewport.w || y1 < 0 || y0 >= viewport.h) continue;
            left = int(std::clamp(x0,0.0,double(viewport.w-1))) / lightTilePixels;
            right = int(std::clamp(x1,0.0,double(viewport.w-1))) / lightTilePixels;
            top = int(std::clamp(y0,0.0,double(viewport.h-1))) / lightTilePixels;
            bottom = int(std::clamp(y1,0.0,double(viewport.h-1))) / lightTilePixels;
        }
        for (int y = top; y <= bottom; ++y)
            for (int x = left; x <= right; ++x)
                masks[(size_t(y)*columns+x)*groups + i/128][(i/32)%4] |= Uint32(1) << (i%32);
    }
    for (const auto& mask : masks) data.push_back(std::bit_cast<SDL_FColor>(mask));
    return data;
}
