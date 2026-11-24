#include "dds_texture.hpp"
#include "texture_loader.hpp"
#include <SDL3/SDL_main.h>
#include <cstdio>
#include <string_view>

// Share the editor's coverage filter; the platform encoder only compresses
// these levels, without regenerating them or changing the source texture.
int main(int argc, char** argv)
{
    if (argc != 4 || (std::string_view(argv[3]) != "color" && std::string_view(argv[3]) != "opacity"))
    {
        std::fprintf(stderr, "Usage: BigSkyFoliageMips input output.dds color|opacity\n");
        return 1;
    }
    SDL_Surface* image = TextureLoader::LoadImageFile(argv[1]);
    DdsTexture mips;
    const auto format = std::string_view(argv[3]) == "color" ? DdsTexture::Format::BaseColor : DdsTexture::Format::Scalar;
    const bool saved = image && mips.FromImage(image, format, true) && mips.SaveMipSource(argv[2]);
    if (!saved) std::fprintf(stderr, "%s: %s\n", argv[1], SDL_GetError());
    SDL_DestroySurface(image);
    return saved ? 0 : 1;
}
