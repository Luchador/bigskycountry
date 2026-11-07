#pragma once

struct SDL_Window;
struct SDL_Renderer;
class RoadMap;
class FreeCamera;

enum class ApplicationMode
{
    Game,
    Editor
};

enum class ToolResult { Closed, Quit, Error };

class EditorTool
{
public:
    virtual ~EditorTool() = default;
    virtual ToolResult Run(SDL_Window* window, SDL_Renderer* renderer, RoadMap& map, FreeCamera& camera) = 0;
};

int RunApplication(ApplicationMode mode, const char* roadFile = nullptr, EditorTool* roadTool = nullptr);
