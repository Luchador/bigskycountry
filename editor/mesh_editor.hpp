#pragma once
#include "material_editor.hpp"
#include "world_renderer.hpp"
#include "free_camera.hpp"
#include "transform_gizmo.hpp"
#include "mesh_collision.hpp"
#include <optional>

class MeshAOEditor;

#ifdef _WIN32
class WindowsMeshControls;
#endif

// Modeless, owned utility window. Material definitions/textures stay shared assets.
class MeshEditorWindow
{
public:
    MeshEditorWindow();
    ~MeshEditorWindow();
    bool Open(SDL_Window* owner, TextureCatalog& catalog, const std::string& path);
    void Close();
    bool Handle(const SDL_Event& event);
    bool Draw();
    bool Focused() const;
#ifdef _WIN32
    // Called by the editor's existing Windows message hook; true means consumed.
    bool HandleNativeMessage(void* message);
#endif
    SDL_Window* Window() const { return window; }
    bool ContainsScreen(float x, float y) const;
    std::optional<std::size_t> SlotAtScreen(float x, float y) const;
    void Hover(std::optional<std::size_t> slot);
    bool Drop(std::size_t slot, const std::string& material);

private:
    void OpenAO();
    void UpdateCamera();
    void CollisionCommand(int command);
    void SetCollisionMode(MeshCollisionMode mode);
    bool CollisionKey(SDL_Keycode key, SDL_Keymod mods);
    bool CollisionPointerDown(SDL_FPoint point);
    void CollisionPointerMove(SDL_FPoint point);
    void FinishCollisionDrag(bool commit);
    bool CommitCollision(const MeshCollisionData& next, bool history = true);
    void SyncCollision();
    void SelectShape(std::optional<std::size_t> index);
    std::size_t RowCount() const;
    std::string ShapeLabel(std::size_t index) const;
    std::optional<GizmoPlacement> CollisionGizmoPlacement() const;
    void DrawCollision();
    TransformGizmo collisionGizmo;
    MeshCollisionData collisionDraft;
    MeshCollision::Snapshot collisionSnapshot;
    std::vector<MeshCollisionData> collisionUndo, collisionRedo;
    std::optional<std::size_t> selectedShape;
    std::optional<int> collisionHover;
    std::optional<GizmoDrag> collisionDrag;
    ObjectTransform collisionDragStart;
    bool showCollision = false;
    std::unique_ptr<MeshAOEditor> aoEditor;
    std::shared_ptr<const MeshAOData> aoSnapshot;
#ifdef _WIN32
    friend class WindowsMeshControls;
    std::unique_ptr<WindowsMeshControls> controls;
#endif
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
    SDL_Window* viewportWindow = nullptr; // Native child on Windows, otherwise window.
    SDL_Renderer* renderer = nullptr;
    std::unique_ptr<WorldRenderer> preview;
    MeshInstance instance;
    MeshInstance vertexInstance; // Private, untextured preview; never changes the asset.
    TextureCatalog* catalog = nullptr;
    std::string path, status;
    FreeCamera camera;
    double yaw = 0, pitch = 0, distance = 3;
    float scroll = 0;
    float panelWidth = 280, panelDragStart = 280;
    std::optional<float> panelGrab;
    SDL_Cursor* resizeCursor = nullptr;
    enum class ViewMode { Lit, VertexColors, VertexAlpha };
    void SetViewMode(ViewMode mode);
    ViewMode viewMode = ViewMode::Lit;
    bool menuOpen = false;
    int activeMenu = 0; // View, Tools, Collision.
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
