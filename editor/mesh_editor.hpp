#pragma once
#include "material_editor.hpp"
#include "world_renderer.hpp"
#include "camera_view.hpp"
#include <optional>

// Modeless, owned utility window. Material definitions/textures stay shared assets.
class MeshEditorWindow
{
public:
    ~MeshEditorWindow() { Close(); }
    bool Open(SDL_Window* owner, TextureCatalog& catalog, const std::string& path);
    void Close();
    bool Handle(const SDL_Event& event);
    bool Draw();
    bool Focused() const { return window && SDL_GetKeyboardFocus() == window; }
    SDL_Window* Window() const { return window; }
    bool ContainsScreen(float x, float y) const;
    std::optional<std::size_t> SlotAtScreen(float x, float y) const;
    void Hover(std::optional<std::size_t> slot);
    bool Drop(std::size_t slot, const std::string& material);

private:
    SDL_FRect View() const;
    SDL_FRect Panel() const;
    SDL_FRect Divider() const;
    SDL_FRect SlotClip() const;
    SDL_FRect LODButton() const;
    SDL_FRect Slot(std::size_t i) const;
    SDL_FRect Clear(std::size_t i) const;
    void Frame();
    void ClampScroll();
    void SetResizeCursor(bool active);
    void CancelPointer();
    bool HandleMenu(const SDL_Event& event);
    bool DrawMenu();
    void DestroyThumbnails();
    bool UpdateThumbnails();
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    std::unique_ptr<WorldRenderer> preview;
    MeshInstance instance;
    MeshInstance vertexInstance; // Private, untextured preview; never changes the asset.
    TextureCatalog* catalog = nullptr;
    std::string path, status;
    CameraView camera;
    double yaw = 0, pitch = 0, distance = 3;
    float scroll = 0;
    float panelWidth = 280, panelDragStart = 280;
    std::optional<float> panelGrab;
    SDL_Cursor* resizeCursor = nullptr;
    enum class ViewMode { Lit, VertexColors };
    ViewMode viewMode = ViewMode::Lit;
    bool menuOpen = false;
    int menuRow = 0;
    int previewLOD = -1; // Auto, then each authored LOD.
    std::optional<SDL_FPoint> orbit;
    std::optional<std::size_t> hover;
    std::vector<MaterialEditor> materialPreviews;
    std::vector<SDL_Texture*> thumbnails;
    unsigned materialRevision = 0;
    std::uint64_t assignmentRevision = 0;
    bool dirty = true, imagesDirty = true;
};
