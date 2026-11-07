#include "road_editor.hpp"

#include <SDL3/SDL_main.h>

int main(int argc, char** argv)
{
    RoadEditor roads;
    return RunApplication(ApplicationMode::Editor, argc > 1 ? argv[1] : nullptr, &roads);
}
