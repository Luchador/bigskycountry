#include "marker.hpp"
#include "free_camera.hpp"
#include "world_renderer.hpp"
#include <ttf2mesh.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{
    using Font = std::unique_ptr<ttf_t, decltype(&ttf_free)>;
    using GlyphMesh = std::unique_ptr<ttf_mesh_t, decltype(&ttf_free_mesh)>;

    Font LoadFont()
    {
        const auto path = AssetPaths::Root(AssetRoot::Cooked) + "fonts/OpenSans-Semibold.ttf";
        size_t size = 0;
        auto* data = static_cast<Uint8*>(SDL_LoadFile(path.c_str(), &size));
        ttf_t* font = nullptr;
        if (data && size <= std::numeric_limits<int>::max())
            ttf_load_from_mem(data, static_cast<int>(size), &font, false);
        SDL_free(data);
        if (!font) SDL_Log("Could not load marker font: %s", path.c_str());
        return Font(font, ttf_free);
    }

    std::shared_ptr<Mesh> MakeLettering(const std::string& text, double& radius)
    {
        static const auto font = LoadFont();
        auto mesh = std::make_shared<Mesh>();
        if (!font) return mesh;

        // Tessellate each distinct glyph just once, including across different markers.
        static std::unordered_map<int, GlyphMesh> glyphs;
        const int capital = ttf_find_glyph(font.get(), 'H');
        if (capital < 0 || font->glyphs[capital].ybounds[1] <= 0) return mesh;
        const double meters = Marker::capHeight / font->glyphs[capital].ybounds[1];
        double advance = 0;
        double bottom = std::numeric_limits<double>::max();
        const char* cursor = text.c_str();
        while (*cursor)
        {
            int id = ttf_find_glyph(font.get(), SDL_StepUTF8(&cursor, nullptr));
            if (id < 0) id = 0; // The typeface's missing-glyph symbol.
            auto& glyph = font->glyphs[id];
            auto it = glyphs.find(id);
            if (it == glyphs.end())
            {
                ttf_mesh_t* triangles = nullptr;
                if (glyph.ncontours) ttf_glyph2mesh(&glyph, &triangles, TTF_QUALITY_NORMAL, TTF_FEATURES_DFLT);
                it = glyphs.emplace(id, GlyphMesh(triangles, ttf_free_mesh)).first;
            }
            if (const auto* data = it->second.get())
            {
                const auto first = static_cast<std::uint32_t>(mesh->vertices.size());
                for (int i = 0; i < data->nvert; ++i)
                {
                    const auto p = data->vert[i];
                    const double y = p.y * meters;
                    bottom = std::min(bottom, y);
                    mesh->vertices.push_back({{advance + p.x * meters, y, 0}, {0, 0, -1}});
                }
                for (int i = 0; i < data->nfaces; ++i)
                {
                    const auto f = data->faces[i];
                    // Clockwise to the viewer, matching the world's front-face winding.
                    mesh->indices.insert(mesh->indices.end(), {first + f.v1, first + f.v3, first + f.v2});
                }
            }
            advance += glyph.advance * meters;
        }
        double radiusSquared = 0;
        for (auto& v : mesh->vertices)
        {
            v.position.x -= advance * 0.5;
            v.position.y -= bottom; // Descenders also clear the top by the full gap.
            radiusSquared = std::max(radiusSquared, Math::Dot(v.position, v.position));
        }
        radius = std::sqrt(radiusSquared);
        return mesh;
    }
}

bool Marker::SetText(std::string value)
{
    const char* p = value.data();
    size_t remaining = value.size();
    int count = 0;
    while (remaining)
    {
        const Uint32 c = SDL_StepUTF8(&p, &remaining);
        if (++count > maxCharacters || c < 32 || (c >= 127 && c <= 159) ||
            c == 0x2028 || c == 0x2029 || c == SDL_INVALID_UNICODE_CODEPOINT)
            return SDL_SetError("Marker text must be a single line of up to %d characters", maxCharacters);
    }
    if (value != text)
    {
        text = std::move(value);
        lettering.reset();
    }
    return true;
}

MeshInstance Marker::Label(const FreeCamera& camera) const
{
    if (text.empty() || !IsLoaded()) return {};
    if (!lettering) lettering = MakeLettering(text, localLabelRadius);
    if (lettering->indices.empty()) return {};

    const auto& instance = Instance();
    if (measuredTransform != GetTransform() || measuredMesh.lock() != instance.mesh)
    {
        Vector3 lo = instance.transform.Point(instance.mesh->vertices[instance.mesh->indices.front()].position);
        Vector3 hi = lo;
        for (const auto index : instance.mesh->indices)
        {
            const auto p = instance.transform.Point(instance.mesh->vertices[index].position);
            lo.x = std::min(lo.x, p.x);
            lo.z = std::min(lo.z, p.z);
            hi.x = std::max(hi.x, p.x);
            hi.y = std::max(hi.y, p.y);
            hi.z = std::max(hi.z, p.z);
        }
        labelOrigin = {(lo.x + hi.x) * 0.5, hi.y + labelGap, (lo.z + hi.z) * 0.5};
        measuredTransform = GetTransform();
        measuredMesh = instance.mesh;
    }
    Vector3 away = labelOrigin - camera.GetPosition();
    if (std::hypot(away.x, away.z) < 1e-8) away = camera.ViewRay(0, 0);
    const Vector3 forward = Math::Normalize({away.x, 0, away.z});
    const Vector3 right{forward.z, 0, -forward.x};
    const auto& scale = GetTransform().scale;
    // Keep the billboard readable even when the marker mesh is mirrored.
    return {lettering, {labelOrigin, right * std::abs(scale.x), {0, std::abs(scale.y), 0}, forward}};
}

void Marker::SubmitLabel(WorldRenderer& renderer, const FreeCamera& camera) const
{
    const auto label = Label(camera);
    if (!label.mesh) return;
    const auto& scale = GetTransform().scale;
    const double worldRadius = localLabelRadius * std::max(std::abs(scale.x), std::abs(scale.y));
    if (renderer.SphereVisible(label.transform.origin, worldRadius))
        renderer.SubmitMesh(label, {textColor[0] / 255.f, textColor[1] / 255.f, textColor[2] / 255.f, 1});
}
