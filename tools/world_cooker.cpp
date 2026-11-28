#include "world_package.hpp"
#include <SDL3/SDL_main.h>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) == "--inspect")
    {
        WorldPackage::Reader reader;
        if (argc != 3 || !reader.Open(argv[2])) { std::cerr << "Inspect failed: " << SDL_GetError() << '\n'; return 1; }
        std::uint64_t near = 0, far = 0;
        for (std::size_t i = 0; i < reader.Index().cells.size(); ++i)
        {
            WorldPackage::Cell cell;
            MeshInstance collision;
            if (!reader.ReadCell(i, cell) || !reader.ReadCollision(i, collision)) { std::cerr << SDL_GetError() << '\n'; return 1; }
            near += reader.Index().cells[i].nearTriangles; far += reader.Index().cells[i].distantTriangles;
        }
        std::cout << reader.Index().entities.size() << " entities, " << reader.Index().cells.size() << " cells, "
                  << reader.Index().assets.size() << " shared asset dependencies\nNear: " << near << "; distant: " << far << " triangles (ties are instanced separately)\n";
        return 0;
    }
    if (argc > 3 || (argc > 1 && std::string(argv[1]) == "--help"))
    {
        std::cout << "BigSkyWorldCooker [world.wld [output.bscworld]]\nBigSkyWorldCooker --inspect output.bscworld\n"
                     "Without arguments, cooks the default editor world. Cooking rebuilds changed assets,\n"
                     "then writes the world package and saves persistent IDs to the source world.\n";
        return argc > 3 ? 1 : 0;
    }
    WorldDocument world(AssetRoot::Project);
    if (!world.Open(argc > 1 ? argv[1] : nullptr)) { std::cerr << SDL_GetError() << '\n'; return 1; }
    const auto file = argc > 2 ? std::string(argv[2]) : WorldPackage::DefaultPath(world);
    if (!WorldPackage::Cook(world, file)) { std::cerr << SDL_GetError() << '\n'; return 1; }
    std::cout << "Cooked world: " << file << '\n';
    return 0;
}
