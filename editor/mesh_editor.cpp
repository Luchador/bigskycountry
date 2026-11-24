#include "mesh_editor.hpp"
#include "mesh_materials.hpp"
#include "editor_theme.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    constexpr float rowHeight = 88, header = 108, menuHeight = 26, dividerWidth = 6;
    // A larger preview space keeps the world's 0.1 m near plane tiny relative to
    // the mesh, even when zooming inside its original bounding sphere.
    constexpr double previewRadius = 100;
    constexpr SDL_FRect viewMenu{4, 0, 60, menuHeight}, viewOptions{4, menuHeight, 196, 52};
    bool Fill(SDL_Renderer* r, SDL_FRect box, SDL_Color c)
    { return EditorTheme::SetColor(r, c) && SDL_RenderFillRect(r, &box); }
    bool Text(SDL_Renderer* r, float x, float y, const std::string& s, SDL_Color c = EditorTheme::text)
    { return EditorTheme::SetColor(r, c) && SDL_RenderDebugText(r, x, y, s.c_str()); }
    std::string Fit(const std::string& text, std::size_t count)
    { return text.size() <= count ? text : count < 4 ? text.substr(0, count) : text.substr(0, count - 3) + "..."; }
    std::size_t Characters(float width) { return static_cast<std::size_t>(std::max(0.f, width) / 8); }

    std::shared_ptr<Mesh> VertexPreview(const Mesh& source)
    {
        auto mesh = std::make_shared<Mesh>();
        mesh->bounds = source.bounds;
        mesh->vertices = source.vertices;
        mesh->indices = source.indices;
        const bool imported = source.vertexColors.size() == source.vertices.size();
        for (std::size_t i = 0; i < mesh->vertices.size(); ++i)
        {
            mesh->vertices[i].color = imported ? source.vertexColors[i] : source.vertices[i].color;
            mesh->vertices[i].color.a = 1;
        }
        mesh->lodDistances = source.lodDistances;
        for (const auto& lod : source.lods) mesh->lods.push_back({VertexPreview(*lod.mesh), lod.threshold});
        return mesh;
    }
}

SDL_FRect MeshEditorWindow::Panel() const
{
    int w = 0, h = 0;
    SDL_GetWindowSize(window, &w, &h);
    const float available = std::max(0.f, w - dividerWidth);
    const float width = std::clamp(panelWidth, std::min(200.f, available / 2), std::max(available - 160, available / 2));
    return {w - width, menuHeight, width, std::max(0.f, h - menuHeight)};
}
SDL_FRect MeshEditorWindow::View() const
{
    const auto panel = Panel();
    return {0, panel.y, std::max(1.f, panel.x - dividerWidth), panel.h};
}
SDL_FRect MeshEditorWindow::Divider() const
{ const auto view = View(); return {view.w, view.y, dividerWidth, view.h}; }
SDL_FRect MeshEditorWindow::SlotClip() const
{ const auto panel = Panel(); return {panel.x, panel.y + header, panel.w, std::max(0.f, panel.h - header - 32)}; }
SDL_FRect MeshEditorWindow::LODButton() const
{ const auto panel = Panel(); return {panel.x + 10, panel.y + 72, panel.w - 20, 28}; }
SDL_FRect MeshEditorWindow::Slot(std::size_t i) const
{ const auto clip = SlotClip(); return {clip.x + 10, clip.y + float(i) * rowHeight - scroll, clip.w - 20, rowHeight - 8}; }
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
    CancelPointer();
    hover.reset(); menuOpen = false;
    SDL_DestroyCursor(resizeCursor); resizeCursor = nullptr;
    DestroyThumbnails();
    preview.reset(); // Release GPU buffers before their renderer/device.
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    renderer = nullptr; window = nullptr;
    instance = {};
    vertexInstance = {};
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
        resizeCursor = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_EW_RESIZE);
        preview = std::make_unique<WorldRenderer>(AssetRoot::Project);
    }
    CancelPointer();
    hover.reset(); menuOpen = false;
    preview->Forget(&instance);
    preview->Forget(&vertexInstance);
    vertexInstance = {};
    instance = {std::move(mesh), {}};
    // Normalize only the preview placement, preserving the source mesh and UVs.
    const auto center = (instance.mesh->bounds.min + instance.mesh->bounds.max) * 0.5;
    const double radius = std::max(1e-6, Math::Length(instance.mesh->bounds.max - center));
    const double scale = previewRadius / radius;
    instance.transform = {center * -scale, {scale, 0, 0}, {0, scale, 0}, {0, 0, scale}};
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
    if (!window || menuOpen || panelGrab || !instance.mesh || !instance.mesh->materials) return {};
    int wx, wy; SDL_GetWindowPosition(window, &wx, &wy);
    const SDL_FPoint point{x - wx, y - wy};
    const auto clip = SlotClip();
    if (!SDL_PointInRectFloat(&point, &clip)) return {};
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

void MeshEditorWindow::ClampScroll()
{
    const float count = instance.mesh && instance.mesh->materials ? float(instance.mesh->materials->slots.size()) : 0;
    scroll = std::clamp(scroll, 0.f, std::max(0.f, count * rowHeight - SlotClip().h));
}

void MeshEditorWindow::SetResizeCursor(bool active)
{
    if (active && resizeCursor) SDL_SetCursor(resizeCursor);
    else if (resizeCursor && SDL_GetCursor() == resizeCursor) SDL_SetCursor(SDL_GetDefaultCursor());
}

void MeshEditorWindow::CancelPointer()
{
    if (orbit || panelGrab) SDL_CaptureMouse(false);
    orbit.reset(); panelGrab.reset();
    SetResizeCursor(false);
}

bool MeshEditorWindow::HandleMenu(const SDL_Event& event)
{
    const auto choose = [&] {
        viewMode = menuRow == 0 ? ViewMode::Lit : ViewMode::VertexColors;
        menuOpen = false;
        dirty = true;
    };
    if (event.type == SDL_EVENT_KEY_DOWN)
    {
        if (!event.key.repeat && event.key.key == SDLK_V && (event.key.mod & SDL_KMOD_ALT))
        {
            CancelPointer();
            menuOpen = !menuOpen; menuRow = int(viewMode); dirty = true;
            return true;
        }
        if (!menuOpen) return false;
        if (event.key.key == SDLK_ESCAPE) menuOpen = false;
        else if (event.key.key == SDLK_UP || event.key.key == SDLK_DOWN) menuRow = 1 - menuRow;
        else if (event.key.key == SDLK_RETURN || event.key.key == SDLK_SPACE) choose();
        dirty = true;
        return true;
    }
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
    {
        const SDL_FPoint point{event.button.x, event.button.y};
        if (SDL_PointInRectFloat(&point, &viewMenu) && event.button.button == SDL_BUTTON_LEFT)
        {
            CancelPointer();
            menuOpen = !menuOpen; menuRow = int(viewMode); dirty = true;
            return true;
        }
        if (menuOpen)
        {
            if (event.button.button == SDL_BUTTON_LEFT && SDL_PointInRectFloat(&point, &viewOptions))
            {
                menuRow = int((point.y - viewOptions.y) / menuHeight);
                choose();
            }
            // Dismissing the menu does not start an orbit or clear a material.
            menuOpen = false; dirty = true;
            return true;
        }
        if (point.y < menuHeight) return true;
    }
    if (menuOpen && event.type == SDL_EVENT_MOUSE_MOTION)
    {
        const SDL_FPoint point{event.motion.x, event.motion.y};
        if (SDL_PointInRectFloat(&point, &viewOptions)) menuRow = int((point.y - viewOptions.y) / menuHeight);
        dirty = true;
    }
    return menuOpen && (event.type == SDL_EVENT_MOUSE_MOTION || event.type == SDL_EVENT_MOUSE_BUTTON_UP ||
                        event.type == SDL_EVENT_MOUSE_WHEEL);
}

bool MeshEditorWindow::DrawMenu()
{
    const auto panel = Panel();
    if (!Fill(renderer, {0, 0, panel.x + panel.w, menuHeight}, EditorTheme::menuBar) ||
        (menuOpen && !Fill(renderer, viewMenu, EditorTheme::selected)) ||
        !Text(renderer, 12, 9, "View")) return false;
    if (!menuOpen) return true;
    if (!Fill(renderer, viewOptions, EditorTheme::chrome)) return false;
    for (int i = 0; i < 2; ++i)
    {
        const SDL_FRect row{viewOptions.x, viewOptions.y + i * menuHeight, viewOptions.w, menuHeight};
        if ((menuRow == i && !Fill(renderer, row, EditorTheme::selected)) ||
            !Text(renderer, row.x + 8, row.y + 9, int(viewMode) == i ? "*" : "") ||
            !Text(renderer, row.x + 28, row.y + 9, i == 0 ? "Lit" : "Vertex Colors")) return false;
    }
    return true;
}

bool MeshEditorWindow::Handle(const SDL_Event& event)
{
    if (!window || SDL_GetWindowFromEvent(&event) != window) return false;
    if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) { Close(); return true; }
    if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST || event.type == SDL_EVENT_WINDOW_HIDDEN)
    {
        CancelPointer();
        menuOpen = false; dirty = true;
    }
    if (event.type == SDL_EVENT_WINDOW_MOUSE_LEAVE && !panelGrab) SetResizeCursor(false);
    if (event.type == SDL_EVENT_WINDOW_EXPOSED || event.type == SDL_EVENT_WINDOW_RESIZED ||
        event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED || event.type == SDL_EVENT_WINDOW_RESTORED)
    {
        ClampScroll(); dirty = true;
    }
    if (HandleMenu(event)) return true;
    if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat)
    {
        if (event.key.key == SDLK_ESCAPE)
        {
            if (panelGrab || orbit)
            {
                if (panelGrab) panelWidth = panelDragStart;
                CancelPointer(); dirty = true;
            }
            else Close();
            return true;
        }
        if (event.key.key == SDLK_F) Frame();
    }
    const auto view = View(), panel = Panel(), divider = Divider();
    if (event.type == SDL_EVENT_MOUSE_WHEEL)
    {
        const SDL_FPoint point{event.wheel.mouse_x, event.wheel.mouse_y};
        if (SDL_PointInRectFloat(&point, &view))
            distance = std::clamp(distance * std::exp(-event.wheel.y * 0.15), 0.02, 50.0);
        else if (SDL_PointInRectFloat(&point, &panel))
        {
            scroll -= event.wheel.y * 40;
            ClampScroll();
        }
        dirty = true;
    }
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT)
    {
        const SDL_FPoint point{event.button.x, event.button.y};
        const auto button = LODButton(), clip = SlotClip();
        if (SDL_PointInRectFloat(&point, &divider))
        {
            panelGrab = point.x - divider.x;
            panelDragStart = panelWidth;
            SDL_CaptureMouse(true); SetResizeCursor(true);
        }
        else if (SDL_PointInRectFloat(&point, &view)) { orbit = point; SDL_CaptureMouse(true); }
        else if (SDL_PointInRectFloat(&point, &button))
        {
            ++previewLOD;
            if (previewLOD > int(instance.mesh->lods.size())) previewLOD = -1;
            dirty = true;
        }
        else if (SDL_PointInRectFloat(&point, &clip))
        {
            for (std::size_t i = 0; i < instance.mesh->materials->slots.size(); ++i)
            {
                const auto clear = Clear(i);
                if (SDL_PointInRectFloat(&point, &clear)) Drop(i, {});
            }
        }
    }
    if (event.type == SDL_EVENT_MOUSE_MOTION)
    {
        const SDL_FPoint point{event.motion.x, event.motion.y};
        if (!(event.motion.state & SDL_BUTTON_LMASK)) CancelPointer();
        if (panelGrab)
        {
            const float available = panel.x + panel.w - dividerWidth;
            panelWidth = std::clamp(available - point.x + *panelGrab,
                                   std::min(200.f, available / 2), std::max(available - 160, available / 2));
            dirty = true;
        }
        else if (orbit)
        {
            yaw -= (point.x - orbit->x) * 0.008;
            pitch = std::clamp(pitch + (point.y - orbit->y) * 0.008, -1.45, 1.45);
            orbit = point;
            dirty = true;
        }
        SetResizeCursor(panelGrab || (!orbit && SDL_PointInRectFloat(&point, &divider)));
    }
    if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT) CancelPointer();
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
    ClampScroll();
    const auto view = View(), panel = Panel(), divider = Divider();
    int ww, wh, pw, ph;
    SDL_GetWindowSize(window, &ww, &wh);
    if (!SDL_GetRenderOutputSize(renderer, &pw, &ph)) return false;
    if (ww <= 0 || wh <= 0 || pw <= 0 || ph <= 0) return true;
    const float sx = float(pw) / ww, sy = float(ph) / wh;
    // Render the mesh at actual drawable resolution, then draw the controls in window units.
    SDL_SetRenderScale(renderer, 1, 1); SDL_SetRenderViewport(renderer, nullptr);
    EditorTheme::SetColor(renderer, EditorTheme::field); SDL_RenderClear(renderer);
    const SDL_Rect viewport{0, int(std::lround(view.y * sy)), int(std::lround(view.w * sx)), int(std::lround(view.h * sy))};
    SDL_SetRenderViewport(renderer, &viewport);
    camera.LookAt(Vector3{std::sin(yaw) * std::cos(pitch) * distance, std::sin(pitch) * distance,
                          -std::cos(yaw) * std::cos(pitch) * distance} * previewRadius, {});
    const WorldLighting lights{{Math::Normalize(Vector3{-0.4, 0.7, -0.6}), {2.0, 1.9, 1.8}},
                               {Math::Normalize(Vector3{0.6, 0.2, 0.7}), {0.35, 0.4, 0.5}}, {0.22, 0.22, 0.24}};
    const bool colors = viewMode == ViewMode::VertexColors;
    if (colors && !vertexInstance.mesh) vertexInstance = {VertexPreview(*instance.mesh), instance.transform};
    // Unit ambient and no directional light reproduce the raw linear colors,
    // with normal display encoding and the same depth-tested, cached GPU path.
    const WorldLighting colorLights{{{0, 1, 0}, {0, 0, 0}}, {{0, 1, 0}, {0, 0, 0}}, {1, 1, 1}};
    if (!preview->Begin(renderer, camera, colors ? &colorLights : &lights)) return false;
    const auto& shown = colors ? vertexInstance : instance;
    const auto level = preview->MeshLOD(shown, previewLOD);
    preview->SubmitWorldMesh(shown, {}, WorldRenderer::MeshUVs::Authored, {1, 1, 1, 1}, previewLOD);
    if (!preview->End()) return false;
    // Large previews may need several budgeted upload frames before becoming visible.
    dirty = preview->PreparedVertices() != 0 || preview->UploadedBytes() != 0;
    SDL_SetRenderViewport(renderer, nullptr); SDL_SetRenderScale(renderer, sx, sy);
    Fill(renderer, divider, EditorTheme::chrome);
    Fill(renderer, {divider.x + 2, divider.y + (divider.h - 32) / 2, 2, 32}, EditorTheme::disabled);
    Fill(renderer, panel, EditorTheme::panel);
    Text(renderer, panel.x + 10, panel.y + 14, "Material slots");
    Text(renderer, panel.x + 10, panel.y + 32, "Drag materials here");
    Text(renderer, panel.x + 10, panel.y + 50, Fit(std::to_string(instance.mesh->materials->slots.size()) + " slots | " +
         std::to_string(instance.mesh->Level(level).indices.size() / 3) + " tris", Characters(panel.w - 20)));
    const auto button = LODButton();
    Fill(renderer, button, EditorTheme::button);
    Text(renderer, button.x + 8, button.y + 10, Fit((previewLOD < 0 ? "Auto: " : "Preview: ") +
         std::string("LOD ") + std::to_string(level) + " / " + std::to_string(instance.mesh->lods.size()), Characters(button.w - 16)));
    const auto area = SlotClip();
    const SDL_Rect clip{int(area.x), int(area.y), int(area.w), int(area.h)};
    SDL_SetRenderClipRect(renderer, &clip);
    const auto& slots = instance.mesh->materials->slots;
    for (std::size_t i = 0; i < slots.size(); ++i)
    {
        const auto box = Slot(i), clear = Clear(i);
        if (box.y + box.h < area.y || box.y >= area.y + area.h) continue;
        Fill(renderer, box, hover == i ? EditorTheme::selected : EditorTheme::list);
        const SDL_FRect image{box.x + 5, box.y + 24, 50, 50};
        if (thumbnails[i]) SDL_RenderTexture(renderer, thumbnails[i], nullptr, &image);
        else { Fill(renderer, image, EditorTheme::field); Text(renderer, image.x + 20, image.y + 20, "M"); }
        Text(renderer, box.x + 6, box.y + 7, Fit(slots[i].name, Characters(box.w - 12)));
        const auto& material = slots[i].material;
        Text(renderer, box.x + 64, box.y + 30, Fit(material.empty() ? "Unassigned" : material.substr(material.find_last_of('/') + 1), Characters(box.w - 70)));
        Text(renderer, box.x + 64, box.y + 51, Fit(material.empty() ? "Mesh colors" : "Material", Characters(clear.x - box.x - 70)));
        Fill(renderer, clear, EditorTheme::button); Text(renderer, clear.x + 9, clear.y + 8, "X");
    }
    SDL_SetRenderClipRect(renderer, nullptr);
    Text(renderer, 10, view.y + view.h - 22, Fit("LMB: orbit  Wheel: zoom  F: frame", Characters(view.w - 20)));
    Text(renderer, panel.x + 10, panel.y + panel.h - 22, Fit(status, Characters(panel.w - 20)));
    return DrawMenu() && SDL_RenderPresent(renderer);
}
