#include "mesh_ao_baker.hpp"
#include "material.hpp"
#include "texture_loader.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <stdexcept>
#ifdef _WIN32
#include <objbase.h>
#endif

bool MeshAOBakeSettings::Valid() const
{
    return samples >= 16 && samples <= 1024 && std::isfinite(distance) && distance >= .001 && distance <= 10000 &&
        std::isfinite(bias) && bias >= .000001 && bias < distance &&
        std::isfinite(strength) && strength >= 0 && strength <= 4;
}

namespace
{
    struct Cancelled {};
    void Check(const std::atomic<bool>& cancel) { if (cancel.load(std::memory_order_relaxed)) throw Cancelled{}; }

    struct Mask
    {
        int w = 0, h = 0;
        std::vector<Uint8> pixels;
        float Sample(SDL_FPoint uv) const
        {
            // Bilinear, repeat addressing, matching the material sampler. The
            // base mip avoids turning transparent space on leaf cards opaque.
            const double x = (uv.x - std::floor(uv.x)) * w - .5, y = (uv.y - std::floor(uv.y)) * h - .5;
            const int ix = int(std::floor(x)), iy = int(std::floor(y));
            const auto at = [&](int a, int b) { return pixels[std::size_t((b + h) % h) * w + (a + w) % w] / 255.f; };
            return std::lerp(std::lerp(at(ix, iy), at(ix + 1, iy), float(x - ix)),
                             std::lerp(at(ix, iy + 1), at(ix + 1, iy + 1), float(x - ix)), float(y - iy));
        }
    };
    struct Surface
    {
        bool foliage = false, invisible = false;
        std::shared_ptr<const Mask> colorAlpha, opacity;
        bool Opaque(SDL_FPoint uv) const
        { return !invisible && (colorAlpha ? colorAlpha->Sample(uv) : 1) * (opacity ? opacity->Sample(uv) : 1) >= .5f; }
    };
    std::vector<Surface> Surfaces(const std::vector<MeshMaterialSlot>& slots, const std::atomic<bool>& cancel)
    {
        std::map<std::pair<std::string, int>, std::shared_ptr<Mask>> masks;
        const auto load = [&](const std::string& path, int channel) -> std::shared_ptr<const Mask> {
            if (path.empty()) return {};
            const auto key = std::pair{path, channel};
            if (auto found = masks.find(key); found != masks.end()) return found->second;
            Check(cancel);
            const auto file = AssetPaths::Root(AssetRoot::Project) + "textures/" + path;
            using Image = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;
            Image image(TextureLoader::LoadImageFile(file), SDL_DestroySurface);
            if (!image) throw std::runtime_error("Cannot read AO cutout texture '" + path + "': " + SDL_GetError());
            Image rgba(SDL_ConvertSurface(image.get(), SDL_PIXELFORMAT_RGBA32), SDL_DestroySurface);
            if (!rgba) throw std::runtime_error(SDL_GetError());
            if (std::uint64_t(rgba->w) * rgba->h > 64ull * 1024 * 1024) throw std::runtime_error("AO cutout texture is too large: " + path);
            auto mask = std::make_shared<Mask>(); mask->w = rgba->w; mask->h = rgba->h;
            mask->pixels.resize(std::size_t(mask->w) * mask->h);
            for (int y = 0; y < mask->h; ++y)
            {
                Check(cancel);
                const auto* row = static_cast<const Uint8*>(rgba->pixels) + y * rgba->pitch;
                for (int x = 0; x < mask->w; ++x) mask->pixels[std::size_t(y) * mask->w + x] = row[x * 4 + channel];
            }
            masks.emplace(key, mask); return mask;
        };
        std::vector<Surface> result(slots.size() + 1); // Last entry is unassigned geometry.
        for (std::size_t i = 0; i < slots.size(); ++i)
        {
            Check(cancel);
            if (slots[i].material.empty()) continue;
            Material material;
            if (!material.Load(AssetPaths::Root(AssetRoot::Project) + "textures/" + slots[i].material))
                throw std::runtime_error("Cannot read AO material '" + slots[i].material + "': " + SDL_GetError());
            auto& surface = result[i];
            surface.foliage = material.type == MaterialType::Foliage;
            surface.invisible = material.type == MaterialType::Unlit;
            if (Material::IsCutout(material.type))
            {
                surface.colorAlpha = load(material.textures[std::size_t(MaterialSlot::BaseColor)], 3);
                surface.opacity = load(material.textures[std::size_t(MaterialSlot::Opacity)], 0);
            }
        }
        return result;
    }
    double Axis(Vector3 p, int axis) { return axis == 0 ? p.x : axis == 1 ? p.y : p.z; }
    struct Triangle
    {
        Vector3 p, u, v, center;
        Bounds3 bounds;
        SDL_FPoint uv, du, dv;
        std::size_t surface;
    };
    struct Node { Bounds3 bounds; std::size_t first = 0, count = 0, left = 0, right = 0; };
    class Occluders
    {
    public:
        Occluders(const Mesh& mesh, const std::vector<Surface>& surfaces, const std::atomic<bool>& cancel)
            : surfaces(surfaces), cancel(cancel), foliage(mesh.vertices.size(), false)
        {
            triangles.reserve(mesh.indices.size() / 3);
            std::size_t part = 0;
            for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
            {
                if ((i & 1023) == 0) Check(cancel);
                while (part < mesh.parts.size() && mesh.parts[part].firstIndex + mesh.parts[part].indexCount <= i) ++part;
                const auto slot = part < mesh.parts.size() && mesh.parts[part].firstIndex <= i ? mesh.parts[part].slot : surfaces.size() - 1;
                const auto surface = std::min(slot, surfaces.size() - 1);
                const auto ia = mesh.indices[i], ib = mesh.indices[i + 1], ic = mesh.indices[i + 2];
                const auto &a = mesh.vertices.at(ia), &b = mesh.vertices.at(ib), &c = mesh.vertices.at(ic);
                if (surfaces[surface].foliage) foliage[ia] = foliage[ib] = foliage[ic] = true;
                if (surfaces[surface].invisible) continue;
                const auto u = b.position - a.position, v = c.position - a.position;
                if (Math::Length(Math::Cross(u, v)) < 1e-14) continue;
                Bounds3 bounds; bounds.Include(a.position); bounds.Include(b.position); bounds.Include(c.position);
                triangles.push_back({a.position, u, v, (a.position + b.position + c.position) * (1.0 / 3), bounds,
                    a.uv, {b.uv.x - a.uv.x, b.uv.y - a.uv.y}, {c.uv.x - a.uv.x, c.uv.y - a.uv.y}, surface});
            }
            if (!triangles.empty()) Build(0, triangles.size());
        }
        bool Foliage(std::size_t vertex) const { return foliage[vertex]; }
        bool Hit(Vector3 origin, Vector3 direction, double distance) const
        {
            if (nodes.empty()) return false;
            std::array<std::size_t, 64> stack{}; std::size_t count = 1;
            while (count)
            {
                const auto& node = nodes[stack[--count]];
                double entryDistance = 0, exitDistance = distance;
                for (int axis = 0; axis < 3 && entryDistance <= exitDistance; ++axis)
                {
                    const auto d = Axis(direction, axis), p = Axis(origin, axis);
                    const auto lo = Axis(node.bounds.min, axis), hi = Axis(node.bounds.max, axis);
                    if (std::abs(d) < 1e-15) { if (p < lo || p > hi) entryDistance = exitDistance + 1; }
                    else
                    {
                        const auto a = (lo - p) / d, b = (hi - p) / d;
                        entryDistance = std::max(entryDistance, std::min(a, b)); exitDistance = std::min(exitDistance, std::max(a, b));
                    }
                }
                if (entryDistance > exitDistance) continue;
                if (!node.count) { stack[count++] = node.left; stack[count++] = node.right; continue; }
                for (std::size_t i = node.first; i < node.first + node.count; ++i)
                {
                    const auto& t = triangles[i];
                    const auto cross = Math::Cross(direction, t.v);
                    const double det = Math::Dot(t.u, cross);
                    if (std::abs(det) < 1e-12) continue;
                    const auto p = origin - t.p;
                    const double u = Math::Dot(p, cross) / det;
                    if (u < 0 || u > 1) continue;
                    const auto q = Math::Cross(p, t.u);
                    const double v = Math::Dot(direction, q) / det;
                    if (v < 0 || u + v > 1) continue;
                    const double depth = Math::Dot(t.v, q) / det;
                    if (depth > 1e-8 && depth < distance && surfaces[t.surface].Opaque({
                        float(t.uv.x + u * t.du.x + v * t.dv.x), float(t.uv.y + u * t.du.y + v * t.dv.y)})) return true;
                }
            }
            return false;
        }
    private:
        std::size_t Build(std::size_t first, std::size_t end)
        {
            Check(cancel);
            const auto index = nodes.size(); nodes.emplace_back();
            Bounds3 bounds, centers;
            for (auto i = first; i < end; ++i)
            { bounds.Include(triangles[i].bounds.min); bounds.Include(triangles[i].bounds.max); centers.Include(triangles[i].center); }
            nodes[index].bounds = bounds;
            if (end - first <= 8) { nodes[index].first = first; nodes[index].count = end - first; return index; }
            const auto size = centers.max - centers.min;
            const int axis = size.x >= size.y && size.x >= size.z ? 0 : size.y >= size.z ? 1 : 2;
            const auto middle = first + (end - first) / 2;
            std::nth_element(triangles.begin() + first, triangles.begin() + middle, triangles.begin() + end,
                [axis](const Triangle& a, const Triangle& b) { return Axis(a.center, axis) < Axis(b.center, axis); });
            const auto left = Build(first, middle), right = Build(middle, end);
            nodes[index].left = left; nodes[index].right = right;
            return index;
        }
        const std::vector<Surface>& surfaces;
        const std::atomic<bool>& cancel;
        std::vector<bool> foliage;
        std::vector<Triangle> triangles;
        std::vector<Node> nodes;
    };
}

MeshAOBakeResult BakeMeshAO(const Mesh& mesh, const std::vector<MeshMaterialSlot>& slots,
    MeshAOBakeSettings settings, const std::atomic<bool>& cancel,
    std::atomic<std::size_t>& completed, std::atomic<std::size_t>& total)
{
    try
    {
#ifdef _WIN32
        // Source TIFF/JPEG cutout maps can use SDL_image's WIC decoder. An
        // std::async worker does not inherit the editor thread's COM apartment.
        struct Apartment
        {
            HRESULT result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
            ~Apartment() { if (SUCCEEDED(result)) CoUninitialize(); }
        } apartment;
        if (FAILED(apartment.result) && apartment.result != RPC_E_CHANGED_MODE)
            throw std::runtime_error("Cannot initialize the AO texture decoder");
#endif
        if (!settings.Valid()) throw std::runtime_error("Invalid AO bake settings");
        completed = 0; std::size_t count = 0;
        for (std::size_t l = 0; l <= mesh.lods.size(); ++l) count += mesh.Level(l).vertices.size();
        total = count;
        const auto surfaces = Surfaces(slots, cancel);
        auto data = std::make_shared<MeshAOData>(); data->alpha.resize(mesh.lods.size() + 1);
        for (std::size_t l = 0; l < data->alpha.size(); ++l)
        {
            Check(cancel);
            const auto& level = mesh.Level(l);
            const Occluders occluders(level, surfaces, cancel);
            auto& alpha = data->alpha[l]; alpha.resize(level.vertices.size(), 1);
            // Deterministic cosine-weighted hemisphere. Leaf cards sample both
            // sides with a uniform sphere, so flipped duplicate faces agree.
            for (std::size_t i = 0; i < level.vertices.size(); ++i)
            {
                Check(cancel);
                const auto& vertex = level.vertices[i];
                const bool sphere = settings.twoSidedFoliage && occluders.Foliage(i);
                auto normal = Math::Normalize(vertex.normal);
                if (Math::Length(normal) < .5) normal = {0, 1, 0};
                if (sphere) normal = {0, 1, 0};
                const auto tangent = Math::Normalize(Math::Cross(std::abs(normal.y) < .9 ? Vector3{0, 1, 0} : Vector3{1, 0, 0}, normal));
                const auto bitangent = Math::Cross(normal, tangent);
                int hits = 0;
                for (int s = 0; s < settings.samples; ++s)
                {
                    if ((s & 31) == 0) Check(cancel);
                    const double u = (s + .5) / settings.samples;
                    const double z = sphere ? 1 - 2 * u : std::sqrt(1 - u);
                    const double r = std::sqrt(std::max(0.0, 1 - z * z));
                    const double phi = s * (std::numbers::pi * (3 - std::sqrt(5.0)));
                    const auto direction = tangent * (r * std::cos(phi)) + bitangent * (r * std::sin(phi)) + normal * z;
                    hits += occluders.Hit(vertex.position + direction * settings.bias, direction, settings.distance - settings.bias);
                }
                alpha[i] = float(std::pow(std::max(0.0, 1.0 - double(hits) / settings.samples), settings.strength));
                completed.fetch_add(1, std::memory_order_relaxed);
            }
        }
        Check(cancel);
        return {std::move(data), {}, false};
    }
    catch (const Cancelled&) { return {{}, {}, true}; }
    catch (const std::exception& error) { return {{}, error.what(), false}; }
}
