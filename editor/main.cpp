#include "road_editor.hpp"
#include "world_state.hpp"

#include <SDL3/SDL_main.h>

int main(int argc, char** argv)
{
    g_WorldState.SetTimeOfDayMinutes(12 * 60);
    RoadEditor roads;
    return RunApplication(ApplicationMode::Editor, argc > 1 ? argv[1] : nullptr, &roads);
}
