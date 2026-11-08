#pragma once

struct SDL_Window;
struct SDL_Renderer;
union SDL_Event;
class WorldDocument;
class FreeCamera;

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
    virtual bool Initialize(SDL_Window* window) = 0;
    virtual bool IsPlanView() const = 0;
    virtual EditorEvent HandleEvent(SDL_Window* window, SDL_Renderer* renderer, const SDL_Event& event,
                                    WorldDocument& world, FreeCamera& camera) = 0;
    virtual bool Render(SDL_Renderer* renderer, const WorldDocument& world, const FreeCamera& camera) = 0;
};

int RunApplication(ApplicationMode mode, const char* worldFile = nullptr, EditorTool* roadTool = nullptr);
