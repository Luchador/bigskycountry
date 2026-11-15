#include "road_names.hpp"

#include "asset_paths.hpp"
#include "camera_view.hpp"
#include "view_geometry.hpp"
#include "viewport_picker.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"
#include <ttf2mesh.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <unordered_map>

namespace
{
    constexpr float capHeight = 11, padding = 3, rasterScale = 2;
    constexpr double repeatDistance = 420;
    using Font = std::unique_ptr<ttf_t, decltype(&ttf_free)>;
    using Glyph = std::unique_ptr<ttf_mesh_t, decltype(&ttf_free_mesh)>;
    using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;
    using Renderer = std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)>;

    struct Label
    {
        // SDL owns textures through the viewport renderer, which outlives Draw but
        // is destroyed before RoadEditor. Only evict/destroy them while it is live.
        SDL_Texture* texture = nullptr;
        float width = 0, height = 0;
        Uint64 used = 0;
    };

    struct Run
    {
        std::vector<SDL_FPoint> points;
        std::vector<double> distances;

        void Append(SDL_FPoint p)
        {
            const double step = points.empty() ? 0 : std::hypot(p.x - points.back().x, p.y - points.back().y);
            if (!points.empty() && step < 0.01) return;
            distances.push_back(distances.empty() ? 0 : distances.back() + step);
            points.push_back(p);
        }

        double Length() const { return distances.empty() ? 0 : distances.back(); }

        SDL_FPoint At(double distance) const
        {
            if (distance <= 0) return points.front();
            if (distance >= Length()) return points.back();
            const auto upper = std::upper_bound(distances.begin(), distances.end(), distance);
            const auto i = static_cast<std::size_t>(upper - distances.begin());
            const float t = static_cast<float>((distance - distances[i - 1]) / (distances[i] - distances[i - 1]));
            return {std::lerp(points[i - 1].x, points[i].x, t), std::lerp(points[i - 1].y, points[i].y, t)};
        }
    };

    struct LabelView
    {
        const CameraView& camera;
        SDL_Rect viewport;
        bool plan;
        Vector3 center;
        double scale, focal;
        std::array<Vector3, 4> sides;

        Vector3 View(Vector3 p) const
        {
            if (plan) return {(p.x - center.x) * scale, (p.z - center.z) * scale, 1};
            return camera.WorldToView(p);
        }

        bool Visible(const Bounds3& bounds) const
        {
            if (bounds.Empty()) return true;
            const Vector3 middle = (bounds.min + bounds.max) * 0.5;
            const auto v = View(middle);
            const double radius = Math::Length(bounds.max - middle) * (plan ? scale : 1);
            if (!plan && v.z + radius < worldNearDepth) return false;
            for (const auto side : sides)
                if (Math::Dot(side, v) + radius * Math::Length(side) < 0) return false;
            return true;
        }

        bool Project(Vector3 from, Vector3 to, SDL_FPoint& a, SDL_FPoint& b) const
        {
            auto p = View(from), q = View(to);
            if (!plan && !ViewGeometry::ClipLine(p, q, {0, 0, 1}, worldNearDepth)) return false;
            for (auto side : sides)
                if (!ViewGeometry::ClipLine(p, q, side, 0)) return false;
            a = ViewGeometry::ScreenPoint(p, focal, viewport);
            b = ViewGeometry::ScreenPoint(q, focal, viewport);
            return true;
        }
    };

    std::vector<Run> VisibleRuns(const Road& road, const LabelView& view)
    {
        std::vector<Run> runs;
        bool connected = false;
        for (std::size_t segment = 0; segment + 1 < road.points.size(); ++segment)
        {
            const auto samples = road.SegmentSamples(segment);
            for (std::size_t i = 1; i < samples.size(); ++i)
            {
                SDL_FPoint a{}, b{};
                if (!view.Project(samples[i - 1], samples[i], a, b))
                {
                    connected = false;
                    continue;
                }
                if (!connected || std::hypot(a.x - runs.back().points.back().x, a.y - runs.back().points.back().y) > 0.5)
                    runs.emplace_back();
                auto& run = runs.back();
                run.Append(a);
                run.Append(b);
                connected = true;
            }
        }
        return runs;
    }

    struct Placement
    {
        SDL_FRect rect, bounds;
        double angle;
        const Label* label;
        const std::string* name;
    };

    std::optional<Placement> Place(const Run& run, double distance, const Label& label, const std::string& name, SDL_Rect view)
    {
        const double half = label.width * 0.5;
        if (run.Length() < label.width + 16 || distance < half || distance + half > run.Length()) return {};
        const auto a = run.At(distance - half), b = run.At(distance + half), middle = run.At(distance);
        double dx = b.x - a.x, dy = b.y - a.y;
        const double chord = std::hypot(dx, dy);
        // Do not put a straight label across a hairpin or a sharp bend.
        if (chord < label.width * 0.9) return {};
        dx /= chord;
        dy /= chord;
        if (dx < 0 || (std::abs(dx) < 1e-5 && dy > 0)) { dx = -dx; dy = -dy; }
        const float lift = label.height * 0.5f + 5;
        const float x = middle.x + static_cast<float>(dy) * lift;
        const float y = middle.y - static_cast<float>(dx) * lift;
        const float w = static_cast<float>(std::abs(dx) * label.width + std::abs(dy) * label.height) + 10;
        const float h = static_cast<float>(std::abs(dy) * label.width + std::abs(dx) * label.height) + 10;
        const SDL_FRect bounds{x - w * 0.5f, y - h * 0.5f, w, h};
        // Leave the viewport's status readouts and clipped edges unobstructed.
        if (bounds.x < 4 || bounds.y < 44 || bounds.x + w > view.w - 4 || bounds.y + h > view.h - 4) return {};
        return Placement{{x - label.width * 0.5f, y - label.height * 0.5f, label.width, label.height},
                         bounds, std::atan2(dy, dx) * 180 / std::numbers::pi, &label, &name};
    }

    bool Overlaps(const Placement& a, const Placement& b)
    {
        if (SDL_HasRectIntersectionFloat(&a.bounds, &b.bounds)) return true;
        // Split roads with the same name should not produce a cluster of duplicate labels.
        return *a.name == *b.name &&
               std::hypot(a.rect.x + a.rect.w * 0.5 - b.rect.x - b.rect.w * 0.5,
                          a.rect.y + a.rect.h * 0.5 - b.rect.y - b.rect.h * 0.5) < repeatDistance * 0.8;
    }
}

struct RoadNames::Cache
{
    Font font{nullptr, ttf_free};
    bool fontAttempted = false;
    std::unordered_map<int, Glyph> glyphs;
    std::unordered_map<std::string, Label> labels;
    SDL_Renderer* owner = nullptr;
    Uint64 frame = 0;
    struct PlanSource
    {
        std::uint64_t revision;
        std::string name;
        bool operator==(const PlanSource&) const = default;
    };
    std::vector<PlanSource> planSources;
    std::vector<Placement> planLabels;
    Vector3 planCenter{};
    double planScale = 0;
    int planWidth = 0, planHeight = 0;
    bool planValid = false;

    void PruneLabels()
    {
        for (auto it = labels.begin(); it != labels.end();)
        {
            if (frame - it->second.used > 120)
            {
                SDL_DestroyTexture(it->second.texture);
                it = labels.erase(it);
            }
            else ++it;
        }
    }

    bool LoadFont()
    {
        if (fontAttempted) return font != nullptr;
        fontAttempted = true;
        const auto path = AssetPaths::Root(AssetRoot::Cooked) + "fonts/OpenSans-Semibold.ttf";
        size_t size = 0;
        auto* data = static_cast<Uint8*>(SDL_LoadFile(path.c_str(), &size));
        ttf_t* loaded = nullptr;
        if (data && size <= std::numeric_limits<int>::max())
            ttf_load_from_mem(data, static_cast<int>(size), &loaded, false);
        SDL_free(data);
        font.reset(loaded);
        if (!font) SDL_Log("Road names: could not load %s", path.c_str());
        return font != nullptr;
    }

    Label Bake(SDL_Renderer* renderer, const std::string& text)
    {
        Label result;
        const int capital = ttf_find_glyph(font.get(), 'H');
        if (capital < 0 || font->glyphs[capital].ybounds[1] <= 0) return result;
        const float scale = capHeight / font->glyphs[capital].ybounds[1];
        float advance = 0, bottom = 0, top = capHeight;
        std::vector<SDL_FPoint> triangles;
        const auto append = [&](int id)
        {
            if (id < 0) id = 0;
            auto& g = font->glyphs[id];
            auto it = glyphs.find(id);
            if (it == glyphs.end())
            {
                ttf_mesh_t* mesh = nullptr;
                if (g.ncontours) ttf_glyph2mesh(&g, &mesh, TTF_QUALITY_NORMAL, TTF_FEATURES_DFLT);
                it = glyphs.emplace(id, Glyph(mesh, ttf_free_mesh)).first;
            }
            if (const auto* mesh = it->second.get())
                for (int i = 0; i < mesh->nfaces; ++i)
                    for (const int vertex : {mesh->faces[i].v1, mesh->faces[i].v2, mesh->faces[i].v3})
                    {
                        const auto p = mesh->vert[vertex];
                        triangles.push_back({advance + p.x * scale, p.y * scale});
                        bottom = std::min(bottom, p.y * scale);
                        top = std::max(top, p.y * scale);
                    }
            advance += g.advance * scale;
        };
        const char* cursor = text.c_str();
        while (*cursor)
        {
            int id = ttf_find_glyph(font.get(), SDL_StepUTF8(&cursor, nullptr));
            if (id < 0) id = 0;
            // Bound texture sizes even for very long lists of road aliases.
            if (advance + font->glyphs[id].advance * scale > 320)
            {
                append(ttf_find_glyph(font.get(), 0x2026));
                break;
            }
            append(id);
        }
        if (triangles.empty()) return result;
        const int width = static_cast<int>(std::ceil((advance + padding * 2) * rasterScale));
        const int height = static_cast<int>(std::ceil((top - bottom + padding * 2) * rasterScale));
        Surface surface(SDL_CreateSurface(width, height, SDL_PIXELFORMAT_RGBA32), SDL_DestroySurface);
        if (!surface) return result;
        Renderer raster(SDL_CreateSoftwareRenderer(surface.get()), SDL_DestroyRenderer);
        if (!raster || !SDL_SetRenderDrawColor(raster.get(), 0, 0, 0, 0) || !SDL_RenderClear(raster.get())) return result;
        std::vector<SDL_Vertex> vertices(triangles.size());
        // Rasterize once at double resolution, including the dark halo. Each
        // visible label is then just one textured quad, even while orbiting.
        for (int pass = 0; pass < 9; ++pass)
        {
            const double angle = pass * std::numbers::pi / 4;
            const float ox = pass == 8 ? 0 : static_cast<float>(std::cos(angle)) * 1.25f;
            const float oy = pass == 8 ? 0 : static_cast<float>(std::sin(angle)) * 1.25f;
            const SDL_FColor color = pass == 8 ? SDL_FColor{0.96f, 0.96f, 0.96f, 1} : SDL_FColor{0.06f, 0.06f, 0.06f, 1};
            for (std::size_t i = 0; i < triangles.size(); ++i)
                vertices[i] = {{(triangles[i].x + padding + ox) * rasterScale,
                                (top - triangles[i].y + padding + oy) * rasterScale}, color, {}};
            if (!SDL_RenderGeometry(raster.get(), nullptr, vertices.data(), static_cast<int>(vertices.size()), nullptr, 0)) return result;
        }
        if (!SDL_RenderPresent(raster.get())) return result;
        result.texture = SDL_CreateTextureFromSurface(renderer, surface.get());
        if (result.texture && (!SDL_SetTextureBlendMode(result.texture, SDL_BLENDMODE_BLEND) ||
                               !SDL_SetTextureScaleMode(result.texture, SDL_SCALEMODE_LINEAR)))
        {
            SDL_DestroyTexture(result.texture);
            result.texture = nullptr;
        }
        result.width = width / rasterScale;
        result.height = height / rasterScale;
        return result;
    }
};

RoadNames::RoadNames() : cache(std::make_unique<Cache>()) {}
RoadNames::~RoadNames() = default;

bool RoadNames::Draw(SDL_Renderer* renderer, const WorldDocument& world, const CameraView& camera,
                     SDL_Rect scene, bool planView, Vector3 planCenter, double planScale)
{
    if (scene.w <= 0 || scene.h <= 0) return true;
    if (cache->owner != renderer)
    {
        cache->planLabels.clear();
        cache->planValid = false;
        cache->labels.clear(); // Textures belong to their original renderer.
        cache->owner = renderer;
    }
    ++cache->frame;
    const SDL_Rect viewport{0, 0, scene.w, scene.h};
    const double focal = planView ? 1 : ViewportFocalLength(scene);
    const LabelView view{camera, viewport, planView, planCenter, planScale, focal, ViewGeometry::SidePlanes(focal, viewport)};
    std::vector<Cache::PlanSource> sources;
    if (planView)
    {
        for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
        {
            const auto& road = world.roads[i];
            if (road.style.railroad || world.hierarchy.EditorHidden({SceneKind::Road, i}) || !view.Visible(road.bounds)) continue;
            auto name = road.NameList();
            if (!name.empty()) sources.push_back({road.revision, std::move(name)});
        }
        if (cache->planValid && cache->planSources == sources && cache->planCenter == planCenter &&
            cache->planScale == planScale && cache->planWidth == scene.w && cache->planHeight == scene.h)
        {
            // Idle map frames reuse both projected splines and label placement.
            // Refresh the texture ages so a cached layout cannot outlive its labels.
            for (const auto& p : cache->planLabels)
            {
                cache->labels.at(*p.name).used = cache->frame;
                if (!SDL_RenderTextureRotated(renderer, p.label->texture, nullptr, &p.rect, p.angle, nullptr, SDL_FLIP_NONE)) return false;
            }
            cache->PruneLabels();
            return true;
        }
    }
    cache->planValid = false;
    cache->planLabels.clear();
    struct Candidate { Run run; const Label* label; const std::string* name; };
    std::vector<Candidate> candidates;
    for (int i = 0; i < static_cast<int>(world.roads.size()); ++i)
    {
        const auto& road = world.roads[i];
        if (road.style.railroad || world.hierarchy.EditorHidden({SceneKind::Road, i}) || !view.Visible(road.bounds)) continue;
        const auto name = road.NameList();
        if (name.empty()) continue;
        auto runs = VisibleRuns(road, view);
        if (std::none_of(runs.begin(), runs.end(), [](const Run& run) { return run.Length() >= 40; })) continue;
        if (!cache->LoadFont()) return true; // Missing optional lettering must not stop the editor.
        auto [it, added] = cache->labels.try_emplace(name);
        if (added) it->second = cache->Bake(renderer, name);
        it->second.used = cache->frame;
        if (!it->second.texture) continue;
        for (auto& run : runs)
            if (run.Length() >= it->second.width + 16)
                candidates.push_back({std::move(run), &it->second, &it->first});
    }
    std::vector<Placement> placed;
    // Give each road a central label before adding repeats on long visible runs.
    // Try nearby positions if the preferred label collides with another road name.
    for (int repeat = 0; repeat < 9 && placed.size() < 128; ++repeat)
        for (const auto& candidate : candidates)
        {
            const double step = std::max(repeatDistance, candidate.label->width * 2.0);
            const double offset = ((repeat + 1) / 2) * step * (repeat % 2 ? 1 : -1);
            const double distance = candidate.run.Length() * 0.5 + offset;
            for (double shift : {0.0, step * 0.25, -step * 0.25})
            {
                auto placement = Place(candidate.run, distance + shift, *candidate.label, *candidate.name, viewport);
                if (!placement || std::any_of(placed.begin(), placed.end(), [&](const Placement& other) { return Overlaps(*placement, other); })) continue;
                if (!SDL_RenderTextureRotated(renderer, candidate.label->texture, nullptr, &placement->rect,
                                              placement->angle, nullptr, SDL_FLIP_NONE)) return false;
                placed.push_back(*placement);
                break;
            }
            if (placed.size() >= 128) break;
        }
    if (planView)
    {
        cache->planSources = std::move(sources);
        cache->planLabels = placed;
        cache->planCenter = planCenter;
        cache->planScale = planScale;
        cache->planWidth = scene.w;
        cache->planHeight = scene.h;
        cache->planValid = true;
    }
    // Renames, deletions and travelling around the map must not retain an
    // unbounded history of label textures. Shared names use a single cache entry.
    cache->PruneLabels();
    return true;
}
