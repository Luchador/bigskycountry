#include "content_browser.hpp"
#include "editor_theme.hpp"
#include "free_camera.hpp"
#include "world_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>

namespace
{
    constexpr float rowHeight = 24.0f;
    constexpr int previewWidth = 194, previewHeight = 116;
    constexpr auto ink = EditorTheme::text, muted = EditorTheme::secondary;
    namespace fs = std::filesystem;

    std::string Utf8(const fs::path& path)
    {
        const auto bytes = path.generic_u8string();
        return {bytes.begin(), bytes.end()};
    }

#ifndef _WIN32
    constexpr float textureRowHeight = 64.0f;

    struct Layout
    {
        SDL_FRect refresh, modelsTab, texturesTab, image;
        int listTop;
        static constexpr int pageTop = 70;

        Layout(int w, int h)
        {
            const float buttonWidth = w >= 140 ? 62.0f : 24.0f;
            refresh = {w - 10.0f - buttonWidth, 10, buttonWidth, 24};
            const float width = static_cast<float>(std::max(0, w - 24));
            const float height = std::min(width * previewHeight / previewWidth, static_cast<float>(h / 3));
            const float tabWidth = width / 2;
            modelsTab = {12, 40, tabWidth, 24};
            texturesTab = {12 + tabWidth, 40, tabWidth, 24};
            image = {12, pageTop, width, height};
            listTop = static_cast<int>(image.y + height) + 32;
        }
    };

    bool Fill(SDL_Renderer* renderer, SDL_FRect rect, SDL_Color color)
    {
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a) && SDL_RenderFillRect(renderer, &rect);
    }

    bool Text(SDL_Renderer* renderer, float x, float y, const std::string& text, SDL_Color color)
    {
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a) &&
               SDL_RenderDebugText(renderer, x, y, text.c_str());
    }

    bool DrawRefresh(SDL_Renderer* renderer, const SDL_FRect& rect)
    {
        if (!Fill(renderer, rect, EditorTheme::button))
        {
            return false;
        }
        if (rect.w >= 60)
        {
            return Text(renderer, rect.x + 3, rect.y + 8, "Refresh", ink);
        }
        const float x = rect.x + rect.w * 0.5f, y = rect.y + rect.h * 0.5f;
        const SDL_FPoint arc[] = {{x+5,y+2}, {x+3,y+5}, {x-2,y+5}, {x-5,y+2}, {x-5,y-2},
                                  {x-2,y-5}, {x+2,y-5}, {x+5,y-2}};
        const SDL_FPoint head[] = {{x+5,y-6}, {x+5,y-2}, {x+1,y-2}};
        return SDL_SetRenderDrawColor(renderer, ink.r, ink.g, ink.b, ink.a) &&
               SDL_RenderLines(renderer, arc, SDL_arraysize(arc)) && SDL_RenderLines(renderer, head, SDL_arraysize(head));
    }

#endif

    SDL_Surface* MakePreview(const std::shared_ptr<const Mesh>& mesh)
    {
        using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)>;
        using Renderer = std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)>;
        Surface surface(SDL_CreateSurface(previewWidth, previewHeight, SDL_PIXELFORMAT_BGRA32), SDL_DestroySurface);
        Renderer offscreen(surface ? SDL_CreateSoftwareRenderer(surface.get()) : nullptr, SDL_DestroyRenderer);
        if (!offscreen)
        {
            return nullptr;
        }

        Vector3 lo = mesh->vertices.front().position, hi = lo;
        for (const MeshVertex& v : mesh->vertices)
        {
            lo = {std::min(lo.x, v.position.x), std::min(lo.y, v.position.y), std::min(lo.z, v.position.z)};
            hi = {std::max(hi.x, v.position.x), std::max(hi.y, v.position.y), std::max(hi.z, v.position.z)};
        }
        // Fit in preview space, so asset units and distance from the model origin do not affect framing.
        const double size = Math::Length(hi - lo);
        const double scale = size > 0 ? 2.0 / size : 1.0;
        const Vector3 center = (lo + hi) * 0.5;
        const Vector3 eye = Math::Normalize({0.6, 0.55, -1.25});
        const Vector3 light = Math::Normalize({-0.5, 0.8, -0.65});
        FreeCamera camera;
        camera.LookAt(eye, {});
        double distance = 0.0;
        struct PreviewVertex { Vector3 p; SDL_FColor color; };
        std::vector<PreviewVertex> vertices;
        vertices.reserve(mesh->vertices.size());
        for (const auto& v : mesh->vertices)
        {
            const Vector3 p = (v.position - center) * scale;
            const Vector3 view = camera.WorldDirectionToView(p);
            // Fit the actual projected vertices with a margin in both dimensions.
            const double extent = std::max(std::abs(view.y), std::abs(view.x) * previewHeight / previewWidth);
            distance = std::max(distance, std::max(worldNearDepth + 0.01, extent / 0.85) - view.z);
            const float shade = 0.54f + 0.46f * static_cast<float>(std::max(0.0, Math::Dot(v.normal, light)));
            vertices.push_back({p, {v.color.r * shade, v.color.g * shade, v.color.b * shade, 1}});
        }
        camera.LookAt(eye * distance, {});
        WorldRenderer scene;
        if (!EditorTheme::SetColor(offscreen.get(), EditorTheme::preview) || !SDL_RenderClear(offscreen.get()) ||
            !scene.Begin(offscreen.get(), camera))
        {
            return nullptr;
        }
        for (std::size_t i = 0; i < mesh->indices.size(); i += 3)
        {
            const auto& a = vertices[mesh->indices[i]];
            const auto& b = vertices[mesh->indices[i + 1]];
            const auto& c = vertices[mesh->indices[i + 2]];
            scene.SubmitTriangle({a.p, b.p, c.p}, {a.color, b.color, c.color});
        }
        if (!scene.End() || !SDL_RenderPresent(offscreen.get()))
        {
            return nullptr;
        }
        return surface.release();
    }
}

void ContentBrowser::SetTab(Tab next)
{
    tab = next;
    if (tab == Tab::Textures && !texturesScanned)
    {
        Refresh();
    }
}

void ContentBrowser::Refresh()
{
    if (tab == Tab::Textures)
    {
        textures.Refresh();
#ifndef _WIN32
        if (std::none_of(textures.Items().begin(), textures.Items().end(),
                         [&](const TextureCatalog::Item& item) { return item.path == selectedTexture; }))
        {
            selectedTexture.clear();
        }
#endif
        texturesScanned = true;
        return;
    }
    rows.clear();
    scanStatus.clear();
    previewDirty = true;
    std::map<std::string, std::vector<std::string>> folders;
    const char* base = SDL_GetBasePath();
    std::error_code error;
    if (base)
    {
        const std::string path = std::string(base) + "assets/models";
        const fs::path root(std::u8string(path.begin(), path.end()));
        fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, error), end;
        for (; !error && it != end; it.increment(error))
        {
            std::error_code statError;
            if (it->is_regular_file(statError) && SDL_strcasecmp(Utf8(it->path().extension()).c_str(), ".glb") == 0)
            {
                const fs::path relative = it->path().lexically_relative(root);
                folders[Utf8(relative.parent_path())].push_back(Utf8(relative.filename()));
            }
        }
    }
    if (!base || (error && error != std::errc::no_such_file_or_directory))
    {
        scanStatus = "Cannot read model folder";
    }
    for (auto& [folder, files] : folders)
    {
        std::sort(files.begin(), files.end());
        if (!folder.empty())
        {
            rows.push_back({folder + "/", {}});
        }
        for (const std::string& file : files)
        {
            rows.push_back({file, folder.empty() ? file : folder + "/" + file});
        }
    }
    if (selected.empty() || std::none_of(rows.begin(), rows.end(), [&](const Row& row) { return row.modelPath == selected; }))
    {
        const auto first = std::find_if(rows.begin(), rows.end(), [](const Row& row) { return !row.modelPath.empty(); });
        selected = first == rows.end() ? std::string{} : first->modelPath;
        scroll = 0;
    }
    if (rows.empty() && scanStatus.empty())
    {
        scanStatus = "No GLB models found";
    }
}

void ContentBrowser::Select(const std::string& path)
{
    if (selected != path)
    {
        selected = path;
        previewDirty = true;
    }
}

bool ContentBrowser::UpdatePreview()
{
    if (!previewDirty)
    {
        return false;
    }
    surface.reset();
    previewDirty = false;
    previewStatus.clear();
    if (!selected.empty())
    {
        MeshAssets assets;
        const auto mesh = assets.Load(selected);
        if (mesh)
        {
            surface.reset(MakePreview(mesh));
        }
        if (surface)
        {
            triangleCount = mesh->indices.size() / 3;
            previewStatus = std::to_string(mesh->vertices.size()) + " verts   " + std::to_string(triangleCount) + " tris";
        }
        else
        {
            previewStatus = "Preview unavailable";
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "Preview '%s': %s", selected.c_str(), SDL_GetError());
        }
    }
    return true;
}

#ifndef _WIN32
std::string ContentBrowser::HandlePointer(const SDL_Event& event, SDL_FPoint local, int width, int height)
{
    const Layout layout(width, height);
    const bool models = tab == Tab::Models;
    const int top = models ? layout.listTop : Layout::pageTop + (textures.Status().empty() ? 0 : 24);
    const float step = models ? rowHeight : textureRowHeight;
    float& offset = models ? scroll : textureScroll;
    const auto count = models ? rows.size() : textures.Items().size();
    if (event.type == SDL_EVENT_MOUSE_WHEEL)
    {
        const float steps = event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -event.wheel.y : event.wheel.y;
        const float limit = std::max(0.0f, count * step - std::max(0, height - top));
        offset = std::clamp(offset - steps * step * 2, 0.0f, limit);
    }
    if (event.type != SDL_EVENT_MOUSE_BUTTON_DOWN || event.button.button != SDL_BUTTON_LEFT)
    {
        return {};
    }
    if (SDL_PointInRectFloat(&local, &layout.refresh))
    {
        Refresh();
    }
    else if (SDL_PointInRectFloat(&local, &layout.modelsTab))
    {
        SetTab(Tab::Models);
    }
    else if (SDL_PointInRectFloat(&local, &layout.texturesTab))
    {
        SetTab(Tab::Textures);
    }
    else if (local.y >= top)
    {
        const auto index = static_cast<std::size_t>((local.y - top + offset) / step);
        if (!models && index < count)
        {
            selectedTexture = textures.Items()[index].path;
        }
        else if (models && index < count && !rows[index].modelPath.empty())
        {
            Select(rows[index].modelPath);
            return rows[index].modelPath;
        }
    }
    return {};
}

bool ContentBrowser::Draw(SDL_Renderer* renderer, int width, int height)
{
    if (width <= 0 || height <= 0)
    {
        return true;
    }
    const bool models = tab == Tab::Models;
    if (models && UpdatePreview())
    {
        if (preview)
        {
            SDL_DestroyTexture(preview);
        }
        preview = surface ? SDL_CreateTextureFromSurface(renderer, surface.get()) : nullptr;
        if (surface && !preview)
        {
            return false;
        }
    }
    const Layout layout(width, height);
    if (!Fill(renderer, {0, 0, static_cast<float>(width), static_cast<float>(height)}, EditorTheme::panel) ||
        (width >= 78 && !Text(renderer, 12, 18, width >= 212 ? "Content browser" : "Content", ink)) ||
        !DrawRefresh(renderer, layout.refresh))
    {
        return false;
    }
    auto drawTab = [&](const SDL_FRect& rect, const char* label, bool active)
    {
        const auto color = active ? EditorTheme::selected : EditorTheme::button;
        return Fill(renderer, rect, color) &&
               Text(renderer, rect.x + std::max(1.0f, (rect.w - SDL_strlen(label) * 8) / 2), rect.y + 8, label, active ? ink : muted);
    };
    if (!drawTab(layout.modelsTab, "Models", models) || !drawTab(layout.texturesTab, "Textures", !models))
    {
        return false;
    }

    int top = Layout::pageTop;
    if (models)
    {
        if (!Fill(renderer, layout.image, EditorTheme::preview))
        {
            return false;
        }
        if (preview)
        {
            const float w = layout.image.h * previewWidth / previewHeight;
            const SDL_FRect image{layout.image.x + (layout.image.w - w) * 0.5f, layout.image.y, w, layout.image.h};
            if (!SDL_RenderTexture(renderer, preview, nullptr, &image))
            {
                return false;
            }
        }
        const std::string status = !scanStatus.empty() ? scanStatus :
            (width < 190 && preview ? std::to_string(triangleCount) + " tris" : previewStatus);
        if (!Text(renderer, 12, layout.image.y + layout.image.h + 10, status, muted))
        {
            return false;
        }
        top = layout.listTop;
    }
    else if (!textures.Status().empty())
    {
        if (!Text(renderer, 12, top + 6, textures.Status(), muted))
        {
            return false;
        }
        top += 24;
    }

    const int listHeight = std::max(0, height - top);
    if (listHeight == 0)
    {
        return true;
    }
    SDL_Rect panel;
    if (!SDL_GetRenderViewport(renderer, &panel))
    {
        return false;
    }
    const SDL_Rect list{panel.x, panel.y + top, width, listHeight};
    if (!SDL_SetRenderViewport(renderer, &list))
    {
        return false;
    }
    const float step = models ? rowHeight : textureRowHeight;
    const auto count = models ? rows.size() : textures.Items().size();
    const float limit = std::max(0.0f, count * step - listHeight);
    float& offset = models ? scroll : textureScroll;
    offset = std::min(offset, limit);
    const auto first = static_cast<std::size_t>(offset / step);
    float y = first * step - offset;
    for (auto i = first; i < count && y < listHeight; ++i, y += step)
    {
        if (!models)
        {
            const auto& item = textures.Items()[i];
            if (selectedTexture == item.path &&
                !Fill(renderer, {5, y, static_cast<float>(std::max(0, width - 12)), step}, EditorTheme::selected))
            {
                return false;
            }
            const SDL_FRect icon{10, y + 5, 54, 54};
            SDL_Texture* thumb = textures.ThumbnailTexture(i, renderer);
            if (thumb)
            {
                if (!SDL_RenderTexture(renderer, thumb, nullptr, &icon))
                {
                    return false;
                }
            }
            else if (!Fill(renderer, icon, EditorTheme::field) || !Text(renderer, 33, y + 28, "?", muted))
            {
                return false;
            }

            // Use two lines for long filenames in the narrow sidebar.
            const size_t chars = static_cast<size_t>(std::max(1, (width - 79) / 8));
            const char* second = item.name.c_str();
            for (size_t n = 0; n < chars && *second; ++n)
            {
                SDL_StepUTF8(&second, nullptr);
            }
            if (!Text(renderer, 72, y + 10, item.name.substr(0, second - item.name.c_str()), ink) ||
                !Text(renderer, 72, y + 23, second, ink) || !Text(renderer, 72, y + 42, item.folder, muted))
            {
                return false;
            }
            continue;
        }
        const Row& row = rows[i];
        const bool folder = row.modelPath.empty();
        if ((!folder && row.modelPath == selected &&
             !Fill(renderer, {5, y, static_cast<float>(std::max(0, width - 12)), rowHeight}, EditorTheme::selected)) ||
            !Text(renderer, folder ? 12 : 22, y + 8, row.label, folder ? muted : ink))
        {
            return false;
        }
    }
    if (limit > 0)
    {
        const float thumb = std::min(static_cast<float>(listHeight), std::max(12.0f, static_cast<float>(listHeight) * listHeight / (count * step)));
        const float top = (listHeight - thumb) * offset / limit;
        if (!Fill(renderer, {static_cast<float>(width - 4), top, 3, thumb}, EditorTheme::button))
        {
            return false;
        }
    }
    return SDL_SetRenderViewport(renderer, &panel);
}
#endif
