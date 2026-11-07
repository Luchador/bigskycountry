#include "application.hpp"

#include <SDL3/SDL_main.h>

int main(int argc, char** argv)
{
    return RunApplication(ApplicationMode::Game, argc > 1 ? argv[1] : nullptr);
}
