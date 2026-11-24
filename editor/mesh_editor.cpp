#include "mesh_editor.hpp"
#include "mesh_materials.hpp"
#include "editor_theme.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    constexpr float panelWidth = 280, rowHeight = 88, header = 108;
    bool Fill(SDL_Renderer* r, SDL_FRect box, SDL_Color c)
    { return EditorTheme::SetColor(r, c) && SDL_RenderFillRect(r, &box); }
    bool Text(SDL_Renderer* r, float x, float y, const std::string& s, SDL_Color c = EditorTheme::text)
    { return EditorTheme::SetColor(r, c) && SDL_RenderDebugText(r, x, y, s.c_str()); }
    std::string Fit(const std::string& text, std::size_t count)
    { return text.size() <= count ? text : text.substr(0, count - 3) + "..."; }
}

SDL_FRect MeshEditorWindow::View() const
{
    int w = 0, h = 0;
    SDL_GetWindowSize(window, &w, &h);
    return {0, 0, std::max(1.f, w - panelWidth), float(h)};
}
SDL_FRect MeshEditorWindow::Slot(std::size_t i) const
{ return {View().w + 10, header + float(i) * rowHeight - scroll, panelWidth - 20, rowHeight - 8}; }
SDL_FRect MeshEditorWindow::Clear(std::size_t i) const
{
    const auto slot = Slot(i);
    return {slot.x + slot.w - 32, slot.y + 48, 26, 24};
}

void MeshEditorWindow::DestroyThumbnails()
{
    for (auto* thumbnail : thumbnails) SDL_DestroyTexture(thumbnail);
    thumbnails.clear();
    materialPreviews.clear();
}

void MeshEditorWindow::Close()
{
    if (orbit) SDL_CaptureMouse(false);
    orbit.reset(); hover.reset();
    DestroyThumbnails();
    preview.reset(); // Release GPU buffers before their renderer/device.
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    renderer = nullptr; window = nullptr;
    instance = {};
    catalog = nullptr;
}

void MeshEditorWindow::Frame()
{
    yaw = 0.75; pitch = 0.12; distance = 1.8;
    dirty = true;
}

bool MeshEditorWindow::Open(SDL_Window* owner, TextureCatalog& textures, const std::string& relative)
{
    MeshAssets assets(AssetRoot::Project);
    auto mesh = assets.Load(relative);
    if (!mesh) return false;
    if (!window)
    {
        const auto properties = SDL_CreateProperties();
        SDL_SetPointerProperty(properties, SDL_PROP_WINDOW_CREATE_PARENT_POINTER, owner);
        // SDL's Windows utility flag creates a hidden substitute owner, ignoring
        // the supplied main editor. A regular owned window stays above that editor.
#ifndef _WIN32
        SDL_SetBooleanProperty(properties, SDL_PROP_WINDOW_CREATE_UTILITY_BOOLEAN, true);
#endif
        SDL_SetBooleanProperty(properties, SDL_PROP_WINDOW_CREATE_HIDDEN_BOOLEAN, true);
        SDL_SetBooleanProperty(properties, SDL_PROP_WINDOW_CREATE_RESIZABLE_BOOLEAN, true);
        SDL_SetBooleanProperty(properties, SDL_PROP_WINDOW_CREATE_HIGH_PIXEL_DENSITY_BOOLEAN, true);
        SDL_SetNumberProperty(properties, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, 900);
        SDL_SetNumberProperty(properties, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, 600);
        int x, y, w, h;
        SDL_GetWindowPosition(owner, &x, &y); SDL_GetWindowSize(owner, &w, &h);
        SDL_SetNumberProperty(properties, SDL_PROP_WINDOW_CREATE_X_NUMBER, x + std::max(0, (w - 900) * 2 / 3));
        SDL_SetNumberProperty(properties, SDL_PROP_WINDOW_CREATE_Y_NUMBER, y + std::max(0, (h - 600) / 2));
        window = SDL_CreateWindowWithProperties(properties);
        SDL_DestroyProperties(properties);
        if (!window) return false;
        SDL_SetWindowMinimumSize(window, 560, 320);
        renderer = SDL_CreateRenderer(window, nullptr);
        if (!renderer) { Close(); return false; }
        preview = std::make_unique<WorldRenderer>(AssetRoot::Project);
    }
    if (orbit) SDL_CaptureMouse(false);
    orbit.reset(); hover.reset();
    preview->Forget(&instance);
    instance = {std::move(mesh), {}};
    // Normalize only the preview placement, preserving the source mesh and UVs.
    const auto center = (instance.mesh->bounds.min + instance.mesh->bounds.max) * 0.5;
    const double radius = std::max(1e-6, Math::Length(instance.mesh->bounds.max - center));
    instance.transform = {center * (-1 / radius), {1 / radius, 0, 0}, {0, 1 / radius, 0}, {0, 0, 1 / radius}};
    previewLOD = -1;
    catalog = &textures; path = relative; scroll = 0; status.clear();
    imagesDirty = true;
    Frame();
    SDL_SetWindowTitle(window, ("Mesh Editor - " + relative).c_str());
    SDL_ShowWindow(window);
    if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) SDL_RestoreWindow(window);
    SDL_RaiseWindow(window);
    return true;
}

bool MeshEditorWindow::ContainsScreen(float x, float y) const
{
    if (!window || (SDL_GetWindowFlags(window) & (SDL_WINDOW_HIDDEN | SDL_WINDOW_MINIMIZED))) return false;
    int wx, wy, w, h;
    SDL_GetWindowPosition(window, &wx, &wy); SDL_GetWindowSize(window, &w, &h);
    return x >= wx && y >= wy && x < wx + w && y < wy + h;
}

std::optional<std::size_t> MeshEditorWindow::SlotAtScreen(float x, float y) const
{
    if (!window || !instance.mesh || !instance.mesh->materials) return {};
    int wx, wy; SDL_GetWindowPosition(window, &wx, &wy);
    const SDL_FPoint point{x - wx, y - wy};
    const auto view = View();
    if (point.y < header || point.y >= view.h - 32) return {};
    for (std::size_t i = 0; i < instance.mesh->materials->slots.size(); ++i)
    {
        const auto slot = Slot(i);
        if (SDL_PointInRectFloat(&point, &slot)) return i;
    }
    return {};
}

void MeshEditorWindow::Hover(std::optional<std::size_t> slot)
{
    if (hover != slot) { hover = slot; dirty = true; }
}

bool MeshEditorWindow::Drop(std::size_t slot, const std::string& material)
{
    if (!instance.mesh) return false;
    const bool ok = MeshMaterialIO::Assign(*instance.mesh, slot, material);
    status = ok ? "Saved" : SDL_GetError();
    if (ok) imagesDirty = true;
    dirty = true;
    return ok;
}

bool MeshEditorWindow::Handle(const SDL_Event& event)
{
    if (!window || SDL_GetWindowFromEvent(&event) != window) return false;
    if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) { Close(); return true; }
    if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST || event.type == SDL_EVENT_WINDOW_HIDDEN)
    {
        if (orbit) SDL_CaptureMouse(false);
        orbit.reset();
    }
    if (event.type == SDL_EVENT_WINDOW_EXPOSED || event.type == SDL_EVENT_WINDOW_RESIZED ||
        event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED || event.type == SDL_EVENT_WINDOW_RESTORED) dirty = true;
    if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat)
    {
        if (event.key.key == SDLK_ESCAPE) { Close(); return true; }
        if (event.key.key == SDLK_F) Frame();
    }
    const auto view = View();
    if (event.type == SDL_EVENT_MOUSE_WHEEL)
    {
        if (event.wheel.mouse_x < view.w)
            distance = std::clamp(distance * std::exp(-event.wheel.y * 0.15), 1.05, 50.0);
        else
        {
            const float count = float(instance.mesh->materials->slots.size());
            scroll = std::clamp(scroll - event.wheel.y * 40, 0.f, std::max(0.f, count * rowHeight - (view.h - header - 32)));
        }
        dirty = true;
    }
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT)
    {
        const SDL_FPoint point{event.button.x, event.button.y};
        if (SDL_PointInRectFloat(&point, &view)) { orbit = point; SDL_CaptureMouse(true); }
        else if (point.x >= view.w + 10 && point.x < view.w + panelWidth - 10 && point.y >= 72 && point.y < 100)
        {
            ++previewLOD;
            if (previewLOD > int(instance.mesh->lods.size())) previewLOD = -1;
            dirty = true;
        }
        else if (point.y >= header && point.y < view.h - 32)
        {
            for (std::size_t i = 0; i < instance.mesh->materials->slots.size(); ++i)
            {
                const auto clear = Clear(i);
                if (SDL_PointInRectFloat(&point, &clear)) Drop(i, {});
            }
        }
    }
    if (event.type == SDL_EVENT_MOUSE_MOTION && orbit)
    {
        if (!(event.motion.state & SDL_BUTTON_LMASK)) { orbit.reset(); SDL_CaptureMouse(false); }
        else
        {
            yaw -= (event.motion.x - orbit->x) * 0.008;
            pitch = std::clamp(pitch + (event.motion.y - orbit->y) * 0.008, -1.45, 1.45);
            orbit = {event.motion.x, event.motion.y};
            dirty = true;
        }
    }
    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT && orbit)
    { orbit.reset(); SDL_CaptureMouse(false); }
    return true; // Utility-window input must never edit or navigate the world.
}

bool MeshEditorWindow::UpdateThumbnails()
{
    if (!imagesDirty) return true;
    DestroyThumbnails();
    const auto& slots = instance.mesh->materials->slots;
    materialPreviews.resize(slots.size());
    thumbnails.resize(slots.size(), nullptr);
    for (std::size_t i = 0; i < slots.size(); ++i)
        if (!slots[i].material.empty() && materialPreviews[i].Open(*catalog, slots[i].material))
            if (auto* image = materialPreviews[i].Preview(64, 64)) thumbnails[i] = SDL_CreateTextureFromSurface(renderer, image);
    materialRevision = Material::Revision(); assignmentRevision = instance.mesh->materials->revision;
    imagesDirty = false;
    return true;
}

bool MeshEditorWindow::Draw()
{
    if (!window || (SDL_GetWindowFlags(window) & (SDL_WINDOW_MINIMIZED | SDL_WINDOW_HIDDEN))) return true;
    if (Material::Revision() != materialRevision || instance.mesh->materials->revision != assignmentRevision) imagesDirty = true;
    if (!dirty && !imagesDirty) return true;
    if (!UpdateThumbnails()) return false;
    const auto view = View();
    int ww, wh, pw, ph;
    SDL_GetWindowSize(window, &ww, &wh);
    if (!SDL_GetRenderOutputSize(renderer, &pw, &ph)) return false;
    if (ww <= 0 || wh <= 0 || pw <= 0 || ph <= 0) return true;
    const float sx = float(pw) / ww, sy = float(ph) / wh;
    // Render the mesh at actual drawable resolution, then draw the controls in window units.
    SDL_SetRenderScale(renderer, 1, 1); SDL_SetRenderViewport(renderer, nullptr);
    EditorTheme::SetColor(renderer, EditorTheme::field); SDL_RenderClear(renderer);
    const SDL_Rect viewport{0, 0, int(std::lround(view.w * sx)), ph};
    SDL_SetRenderViewport(renderer, &viewport);
    camera.LookAt({std::sin(yaw) * std::cos(pitch) * distance, std::sin(pitch) * distance,
                   -std::cos(yaw) * std::cos(pitch) * distance}, {});
    const WorldLighting lights{{Math::Normalize(Vector3{-0.4, 0.7, -0.6}), {2.0, 1.9, 1.8}},
                               {Math::Normalize(Vector3{0.6, 0.2, 0.7}), {0.35, 0.4, 0.5}}, {0.22, 0.22, 0.24}};
    if (!preview->Begin(renderer, camera, &lights)) return false;
    const auto level = preview->MeshLOD(instance, previewLOD);
    preview->SubmitWorldMesh(instance, {}, WorldRenderer::MeshUVs::Authored, {1, 1, 1, 1}, previewLOD);
    if (!preview->End()) return false;
    // Large previews may need several budgeted upload frames before becoming visible.
    dirty = preview->PreparedVertices() != 0 || preview->UploadedBytes() != 0;
    SDL_SetRenderViewport(renderer, nullptr); SDL_SetRenderScale(renderer, sx, sy);
    Fill(renderer, {view.w, 0, panelWidth, view.h}, EditorTheme::panel);
    Text(renderer, view.w + 10, 14, "Material slots");
    Text(renderer, view.w + 10, 32, "Drag materials here");
    Text(renderer, view.w + 10, 50, std::to_string(instance.mesh->materials->slots.size()) + " slots | " +
         std::to_string(instance.mesh->Level(level).indices.size() / 3) + " tris");
    Fill(renderer, {view.w + 10, 72, panelWidth - 20, 28}, EditorTheme::button);
    Text(renderer, view.w + 18, 82, (previewLOD < 0 ? "Auto: " : "Preview: ") +
         std::string("LOD ") + std::to_string(level) + " / " + std::to_string(instance.mesh->lods.size()) + "  [click]");
    const SDL_Rect clip{int(view.w), int(header), int(panelWidth), std::max(0, int(view.h - header - 32))};
    SDL_SetRenderClipRect(renderer, &clip);
    const auto& slots = instance.mesh->materials->slots;
    for (std::size_t i = 0; i < slots.size(); ++i)
    {
        const auto box = Slot(i), clear = Clear(i);
        if (box.y + box.h < header || box.y >= view.h - 32) continue;
        Fill(renderer, box, hover == i ? EditorTheme::selected : EditorTheme::list);
        const SDL_FRect image{box.x + 5, box.y + 24, 50, 50};
        if (thumbnails[i]) SDL_RenderTexture(renderer, thumbnails[i], nullptr, &image);
        else { Fill(renderer, image, EditorTheme::field); Text(renderer, image.x + 20, image.y + 20, "M"); }
        Text(renderer, box.x + 6, box.y + 7, Fit(slots[i].name, 31));
        const auto& material = slots[i].material;
        Text(renderer, box.x + 64, box.y + 30, material.empty() ? "Unassigned" : Fit(material.substr(material.find_last_of('/') + 1), 23));
        Text(renderer, box.x + 64, box.y + 51, material.empty() ? "Mesh colors" : "Material");
        Fill(renderer, clear, EditorTheme::button); Text(renderer, clear.x + 9, clear.y + 8, "X");
    }
    SDL_SetRenderClipRect(renderer, nullptr);
    Text(renderer, 10, view.h - 22, "LMB: orbit  Wheel: zoom  F: frame");
    Text(renderer, view.w + 10, view.h - 22, Fit(status, 32));
    return SDL_RenderPresent(renderer);
}
