#include "material_editor.hpp"
#include "editor_theme.hpp"
#include "mesh.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

namespace
{
    constexpr double pi = std::numbers::pi;

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

    struct ChannelValues
    {
        float unorm[256], srgb[256];

        ChannelValues()
        {
            for (int i = 0; i < 256; ++i)
            {
                const double c = i / 255.;
                unorm[i] = i / 255.f;
                srgb[i] = float(c <= .04045 ? c / 12.92 : std::pow((c + .055) / 1.055, 2.4));
            }
        }
    };
    const ChannelValues values;

    struct Color
    {
        float channel[4];
    };

    template<int channels, bool srgb = false>
    Color Sample(const SDL_Surface* image, double u, double v, Color fallback)
    {
        if (!image) return fallback;
        const double x = std::clamp(u, 0., 1.) * image->w - .5;
        const double y = std::clamp(v * image->h - .5, 0., double(image->h - 1));
        const int left = int(std::floor(x)), top = int(y);
        const float fx = float(x - left), fy = float(y - top);
        // Wrap U across the sphere's seam and clamp V at the poles.
        const int x0 = left < 0 ? image->w - 1 : left;
        const int x1 = x0 + 1 == image->w ? 0 : x0 + 1;
        const auto* row0 = static_cast<const Uint8*>(image->pixels) + top * image->pitch;
        const auto* row1 = top + 1 < image->h ? row0 + image->pitch : row0;
        const auto* p00 = row0 + 4 * x0;
        const auto* p10 = row0 + 4 * x1;
        const auto* p01 = row1 + 4 * x0;
        const auto* p11 = row1 + 4 * x1;
        const float w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy);
        const float w01 = (1 - fx) * fy, w11 = fx * fy;
        Color color{};
        for (int c = 0; c < channels; ++c)
        {
            const float* lut = srgb && c < 3 ? values.srgb : values.unorm;
            color.channel[c] = w00 * lut[p00[c]] + w10 * lut[p10[c]] + w01 * lut[p01[c]] + w11 * lut[p11[c]];
        }
        return color;
    }

    // Bounded unit-sphere normals do not need the overflow scaling of a general-purpose hypot.
    Vector3 UnitNormal(Vector3 v)
    {
        const double squared = Math::Dot(v, v);
        return squared > 0.0 ? v * (1.0 / std::sqrt(squared)) : Vector3{};
    }

    struct DisplayTransfer
    {
        static constexpr int bins = 4096;
        Uint8 code[bins];
        double nextCode[256];

        DisplayTransfer()
        {
            for (int i = 0; i < 255; ++i)
            {
                const double s = (i + .5) / 255.;
                nextCode[i] = s <= .04045 ? s / 12.92 : std::pow((s + .055) / 1.055, 2.4);
            }
            nextCode[255] = 2.0;
            int c = 0;
            for (int i = 0; i < bins; ++i)
            {
                while (double(i) / bins >= nextCode[c]) ++c;
                code[i] = static_cast<Uint8>(c);
            }
        }

        Uint8 Encode(double radiance) const
        {
            const double x = std::max(0., radiance);
            const double mapped = x / (1 + x);
            // Each bin spans less than one 8-bit sRGB step. One threshold comparison
            // gives the exact rounded code without three pow calls per shaded pixel.
            const auto c = code[std::min(int(mapped * bins), bins - 1)];
            return static_cast<Uint8>(c + (mapped >= nextCode[c]));
        }
    };
    const DisplayTransfer display;

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
            const auto albedo = Sample<4, true>(base, u, v, {.48f, .48f, .48f, 1});
            if (cutout && albedo.channel[3] * Sample<1>(opacity, u, v, {1, 1, 1, 1}).channel[0] < .5f)
                return;
            n = UnitNormal(n);
            if (normal)
            {
                const auto map = Sample<3>(normal, u, v, {.5f, .5f, 1, 1});
                const Vector3 tangent = UnitNormal({n.z, 0, -n.x});
                // Sphere V increases toward the south pole.
                const Vector3 bitangent = Math::Cross(tangent, n);
                n = UnitNormal(tangent * (2 * map.channel[0] - 1) + bitangent * (2 * map.channel[1] - 1) + n * (2 * map.channel[2] - 1));
            }
            const auto packed = Sample<3>(orm, u, v, {1, .5f, 0, 1});
            const auto glow = Sample<3, true>(emissive, u, v, {0, 0, 0, 1});
            const double ao = packed.channel[0], rough = std::max(.085, double(packed.channel[1])), metal = packed.channel[2];
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
                const double f0 = .04 * (1 - metal) + albedo.channel[c] * metal;
                const double f = f0 + (1 - f0) * fresnelWeight;
                const double diffuse = albedo.channel[c] * (1 - metal) * (1 - f);
                pixel[2 - c] = display.Encode(glow.channel[c] + ao * (ambient * diffuse + studio * f0) +
                                       2.6 * nl * (diffuse / pi + distribution * visibility * f));
            }
        }
    };

    double Edge(SDL_FPoint a, SDL_FPoint b, double x, double y)
    {
        return (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
    }

    struct Triangle
    {
        const std::uint32_t* index;
        SDL_FPoint p, q, r;
        double area;
        int left, right, top, bottom;
    };

    struct RasterJob
    {
        const Shader& shader;
        const Mesh& mesh;
        const std::vector<Triangle>& triangles;
        SDL_Surface& target;
        int endRow;
        std::atomic<int> nextRow;

        void DrawRows(int first, int end) const
        {
            for (const Triangle& tri : triangles)
            {
                const int top = std::max(first, tri.top), bottom = std::min(end - 1, tri.bottom);
                if (top > bottom) continue;
                const auto& a = mesh.vertices[tri.index[0]];
                const auto& b = mesh.vertices[tri.index[1]];
                const auto& c = mesh.vertices[tri.index[2]];
                for (int y = top; y <= bottom; ++y)
                {
                    auto* row = static_cast<Uint8*>(target.pixels) + y * target.pitch;
                    for (int x = tri.left; x <= tri.right; ++x)
                    {
                        const double wa = Edge(tri.q, tri.r, x + .5, y + .5) / tri.area;
                        const double wb = Edge(tri.r, tri.p, x + .5, y + .5) / tri.area;
                        const double wc = 1 - wa - wb;
                        if (wa < -1e-7 || wb < -1e-7 || wc < -1e-7) continue;
                        shader.Shade(row + 4 * x, a.normal * wa + b.normal * wb + c.normal * wc,
                                     a.uv.x * wa + b.uv.x * wb + c.uv.x * wc, a.uv.y * wa + b.uv.y * wb + c.uv.y * wc);
                    }
                }
            }
        }

        static int SDLCALL Run(void* data)
        {
            auto& job = *static_cast<RasterJob*>(data);
            constexpr int rowsPerBatch = 24;
            for (int row = job.nextRow.fetch_add(rowsPerBatch, std::memory_order_relaxed); row < job.endRow;
                 row = job.nextRow.fetch_add(rowsPerBatch, std::memory_order_relaxed))
                job.DrawRows(row, std::min(row + rowsPerBatch, job.endRow));
            return 0;
        }
    };
}

void MaterialEditor::RenderSphere(SDL_Surface& target) const
{
    const int width = target.w, height = target.h;
    const auto image = [&](MaterialSlot slot) {
        return catalog->Image(material.textures[static_cast<std::size_t>(slot)]);
    };
    // Orthographic camera aimed at the center of the sphere. UVs and lighting stay in mesh space.
    const Vector3 right{std::cos(yaw), 0.0, -std::sin(yaw)};
    const Vector3 toEye{std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)};
    const Vector3 up = Math::Cross(toEye, right);
    const double radius = std::min(width, height) * .425;
    const auto project = [&](Vector3 point) -> SDL_FPoint {
        return {float(width * .5 + radius * Math::Dot(point, right)), float(height * .5 - radius * Math::Dot(point, up))};
    };
    const Shader shader{image(MaterialSlot::BaseColor), image(MaterialSlot::Normal),
                        image(MaterialSlot::ORM),       image(MaterialSlot::Emissive),
                        image(MaterialSlot::Opacity),   material.type == MaterialType::AlphaCutout, toEye};
    constexpr auto background = EditorTheme::field;
    SDL_FillSurfaceRect(&target, nullptr, SDL_MapSurfaceRGB(&target, background.r, background.g, background.b));
    static const Mesh sphere = BuildSphere();
    std::vector<SDL_FPoint> projected;
    projected.reserve(sphere.vertices.size());
    for (const auto& vertex : sphere.vertices) projected.push_back(project(vertex.position));

    std::vector<Triangle> triangles;
    triangles.reserve(sphere.indices.size() / 6);
    for (std::size_t i = 0; i < sphere.indices.size(); i += 3)
    {
        const auto* index = sphere.indices.data() + i;
        const auto p = projected[index[0]], q = projected[index[1]], r = projected[index[2]];
        const double area = Edge(p, q, r.x, r.y);
        if (area < .0001) continue;
        triangles.push_back({index, p, q, r, area,
            int(std::min({p.x, q.x, r.x})), int(std::max({p.x, q.x, r.x})),
            int(std::min({p.y, q.y, r.y})), int(std::max({p.y, q.y, r.y}))});
    }

    // Only disjoint scanlines are written concurrently. The source images and geometry
    // stay read-only until all workers finish, including during material edits and resize.
    const int firstRow = int(height * .5 - radius);
    const int endRow = std::min(height, int(height * .5 + radius) + 1);
    RasterJob job{shader, sphere, triangles, target, endRow, firstRow};
    constexpr int maxWorkers = 8;
    const int workers = std::min({maxWorkers, SDL_GetNumLogicalCPUCores(), 1 + (endRow - firstRow) / 96});
    SDL_Thread* threads[maxWorkers - 1];
    int started = 0;
    while (started + 1 < workers)
    {
        auto* thread = SDL_CreateThread(RasterJob::Run, "Material preview", &job);
        if (!thread) break;
        threads[started++] = thread;
    }
    RasterJob::Run(&job);
    for (int i = 0; i < started; ++i) SDL_WaitThread(threads[i], nullptr);
}
