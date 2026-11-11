#include "material_editor.hpp"
#include "editor_theme.hpp"
#include "mesh.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double pi = std::numbers::pi;
    constexpr int size = 320;
    constexpr double radius = size * .425;

    Mesh BuildSphere()
    {
        Mesh mesh;
        constexpr int slices = 64, rings = 32;
        for (int j = 0; j <= rings; ++j)
        {
            const double v = double(j) / rings;
            for (int i = 0; i <= slices; ++i)
            {
                const double u = double(i) / slices, a = (u - .5) * 2 * pi;
                const Vector3 n{std::sin(pi * v) * std::sin(a), std::cos(pi * v), std::sin(pi * v) * std::cos(a)};
                mesh.vertices.push_back({n, n, {float(u), float(v)}});
                if (i == slices || j == rings)
                    continue;
                const unsigned k = j * (slices + 1) + i;
                mesh.indices.insert(mesh.indices.end(), {k, k + 1, k + slices + 2, k, k + slices + 2, k + slices + 1});
            }
        }
        return mesh;
    }

    const auto srgbToLinear = [] {
        std::array<float, 256> values{};
        for (int i = 0; i < 256; ++i)
        {
            const double c = i / 255.;
            values[i] = float(c <= .04045 ? c / 12.92 : std::pow((c + .055) / 1.055, 2.4));
        }
        return values;
    }();

    using Color = std::array<float, 4>;
    Color Sample(SDL_Surface* image, double u, double v, Color fallback, bool srgb = false)
    {
        if (!image)
            return fallback;
        const double x = u * image->w - .5, y = std::clamp(v * image->h - .5, 0., double(image->h - 1));
        const int left = int(std::floor(x)), top = int(y);
        const float fx = float(x - left), fy = float(y - top);
        Color color{};
        for (int j = 0; j < 2; ++j)
        {
            const auto* row = static_cast<const Uint8*>(image->pixels) + std::min(top + j, image->h - 1) * image->pitch;
            for (int i = 0; i < 2; ++i)
            {
                const auto* p = row + 4 * ((left + i + image->w) % image->w);
                const float w = (i ? fx : 1 - fx) * (j ? fy : 1 - fy);
                for (int c = 0; c < 4; ++c)
                    color[c] += w * (srgb && c < 3 ? srgbToLinear[p[c]] : p[c] / 255.f);
            }
        }
        return color;
    }

    Uint8 Display(double radiance)
    {
        const double x = std::max(0., radiance);
        const double mapped = x / (1 + x);
        const double s = mapped <= .0031308 ? 12.92 * mapped : 1.055 * std::pow(mapped, 1. / 2.4) - .055;
        return static_cast<Uint8>(std::round(s * 255));
    }

    struct Shader
    {
        SDL_Surface *base, *normal, *orm, *emissive, *opacity;
        bool cutout;
        Vector3 toEye;
        const Vector3 light = Math::Normalize({-.48, .68, .72});
        const Vector3 half = Math::Normalize(light + toEye);
        const double fresnelWeight = std::pow(1.0 - std::clamp(Math::Dot(toEye, half), 0.0, 1.0), 5);

        void Shade(Uint8* pixel, Vector3 n, double u, double v) const
        {
            const auto albedo = Sample(base, u, v, {.48f, .48f, .48f, 1}, true);
            if (cutout && albedo[3] * Sample(opacity, u, v, {1, 1, 1, 1})[0] < .5f)
                return;
            n = Math::Normalize(n);
            if (normal)
            {
                const auto map = Sample(normal, u, v, {.5f, .5f, 1, 1});
                const Vector3 tangent = Math::Normalize({n.z, 0, -n.x});
                // Sphere V increases toward the south pole.
                const Vector3 bitangent = Math::Cross(tangent, n);
                n = Math::Normalize(tangent * (2 * map[0] - 1) + bitangent * (2 * map[1] - 1) + n * (2 * map[2] - 1));
            }
            const auto packed = Sample(orm, u, v, {1, .5f, 0, 1});
            const auto glow = Sample(emissive, u, v, {0, 0, 0, 1}, true);
            const double ao = packed[0], rough = std::max(.085, double(packed[1])), metal = packed[2];
            const double nl = std::max(0., Math::Dot(n, light)), nv = std::clamp(Math::Dot(n, toEye), .001, 1.);
            const double nh = std::max(0., Math::Dot(n, half));
            const double a2 = rough * rough * rough * rough;
            const double denom = nh * nh * (a2 - 1) + 1;
            const double distribution = a2 / (pi * denom * denom);
            const double k = (rough + 1) * (rough + 1) / 8;
            const double visibility = .25 / ((nv * (1 - k) + k) * (nl * (1 - k) + k));
            const double ambient = .16 + .17 * (.5 + .5 * n.y);
            // A broad studio reflection keeps metallic surfaces readable away from the direct light.
            const auto reflection = n * (2 * nv) - toEye;
            const double studio =
                .09 + .2 * (.5 + .5 * reflection.y) +
                1.6 * std::pow(std::max(0., Math::Dot(reflection, light)), 2 + 20 * (1 - rough)) / (1 + rough);
            for (int c = 0; c < 3; ++c)
            {
                const double f0 = .04 * (1 - metal) + albedo[c] * metal;
                const double f = f0 + (1 - f0) * fresnelWeight;
                const double diffuse = albedo[c] * (1 - metal) * (1 - f);
                pixel[2 - c] = Display(glow[c] + ao * (ambient * diffuse + studio * f0) +
                                       2.6 * nl * (diffuse / pi + distribution * visibility * f));
            }
        }
    };

    double Edge(SDL_FPoint a, SDL_FPoint b, double x, double y)
    {
        return (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
    }
}

MaterialSurface MaterialEditor::RenderSphere() const
{
    const auto image = [&](MaterialSlot slot) {
        return catalog->Image(material.textures[static_cast<std::size_t>(slot)]);
    };
    // Orthographic camera aimed at the center of the sphere. UVs and lighting stay in mesh space.
    const Vector3 right{std::cos(yaw), 0.0, -std::sin(yaw)};
    const Vector3 toEye{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
    const Vector3 up = Math::Cross(toEye, right);
    const auto project = [&](Vector3 point) -> SDL_FPoint {
        return {float(size * .5 + radius * Math::Dot(point, right)), float(size * .5 - radius * Math::Dot(point, up))};
    };
    const Shader shader{image(MaterialSlot::BaseColor), image(MaterialSlot::Normal),
                        image(MaterialSlot::ORM),       image(MaterialSlot::Emissive),
                        image(MaterialSlot::Opacity),   material.type == MaterialType::AlphaCutout, toEye};
    MaterialSurface surface(SDL_CreateSurface(size, size, SDL_PIXELFORMAT_BGRA32), SDL_DestroySurface);
    if (!surface)
        return surface;
    constexpr auto background = EditorTheme::field;
    SDL_FillSurfaceRect(surface.get(), nullptr, SDL_MapSurfaceRGB(surface.get(), background.r, background.g, background.b));
    static const Mesh sphere = BuildSphere();
    for (std::size_t i = 0; i < sphere.indices.size(); i += 3)
    {
        const auto& a = sphere.vertices[sphere.indices[i]];
        const auto& b = sphere.vertices[sphere.indices[i + 1]];
        const auto& c = sphere.vertices[sphere.indices[i + 2]];
        const auto p = project(a.position), q = project(b.position), r = project(c.position);
        const double area = Edge(p, q, r.x, r.y);
        if (area < .0001)
            continue; // Back of the convex mesh and degenerate pole triangles.
        for (int y = int(std::min({p.y, q.y, r.y})); y <= int(std::max({p.y, q.y, r.y})); ++y)
        {
            auto* row = static_cast<Uint8*>(surface->pixels) + y * surface->pitch;
            for (int x = int(std::min({p.x, q.x, r.x})); x <= int(std::max({p.x, q.x, r.x})); ++x)
            {
                const double wa = Edge(q, r, x + .5, y + .5) / area, wb = Edge(r, p, x + .5, y + .5) / area,
                             wc = 1 - wa - wb;
                if (std::min({wa, wb, wc}) < -1e-7)
                    continue;
                shader.Shade(row + 4 * x, a.normal * wa + b.normal * wb + c.normal * wc,
                             a.uv.x * wa + b.uv.x * wb + c.uv.x * wc, a.uv.y * wa + b.uv.y * wb + c.uv.y * wc);
            }
        }
    }
    return surface;
}
