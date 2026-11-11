#pragma once

struct SDL_Window;
struct SDL_Renderer;
union SDL_Event;
class WorldDocument;
struct RoadGeometry;
class FreeCamera;
class MeshAssets;
class WorldRenderer;

enum class ApplicationMode
{
    Game,
    Editor
};

enum class EditorEvent { Pass, Handled, Error };

class EditorTool
{
public:
    virtual ~EditorTool() = default;
    virtual SDL_Window* CreateRenderWindow(SDL_Window* frame) { return frame; }
    virtual bool ViewportFocused() const { return true; }
    virtual bool Initialize(SDL_Window* window, MeshAssets& models) = 0;
    virtual void SubmitGizmos(WorldRenderer& renderer, const WorldDocument& world, const FreeCamera& camera) const = 0;
    virtual bool DraggingGizmo() const = 0;
    virtual const RoadGeometry* SelectedGeometry(const WorldDocument& world) const = 0;
    virtual bool WireframeEnabled() const { return false; }
    virtual bool IsPlanView() const = 0;
    virtual bool SetContentViewport(SDL_Renderer* renderer) = 0;
    virtual EditorEvent HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event, WorldDocument& world, FreeCamera& camera) = 0;
    virtual bool Render(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera) = 0;
};

int RunApplication(ApplicationMode mode, const char* worldFile = nullptr, EditorTool* roadTool = nullptr);
