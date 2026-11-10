#pragma once

#include <SDL3/SDL_render.h>
#include <array>

// Editor chrome. World materials and viewport selection colors have their own semantics.
namespace EditorTheme
{
    inline constexpr SDL_Color chrome{29, 29, 29, 255};
    inline constexpr SDL_Color panel{48, 48, 48, 255};
    inline constexpr SDL_Color list{40, 40, 40, 255};
    inline constexpr SDL_Color alternateRow{44, 44, 44, 255};
    inline constexpr SDL_Color header{53, 53, 53, 255};
    inline constexpr SDL_Color field{36, 36, 36, 255};
    inline constexpr SDL_Color preview{24, 24, 24, 255};
    inline constexpr SDL_Color button{84, 84, 84, 255};
    inline constexpr SDL_Color hover{98, 98, 98, 255};
    inline constexpr SDL_Color pressed{61, 61, 61, 255};
    inline constexpr SDL_Color border{27, 27, 27, 255};
    inline constexpr SDL_Color text{232, 232, 232, 255};
    inline constexpr SDL_Color secondary{190, 190, 190, 255};
    inline constexpr SDL_Color disabled{148, 148, 148, 255};
    inline constexpr SDL_Color selected{54, 89, 132, 255};
    inline constexpr SDL_Color accent{94, 146, 207, 255};
    inline constexpr SDL_Color planBackground{63, 63, 63, 255};
    inline constexpr SDL_Color checkerLight{123, 123, 123, 255};
    inline constexpr SDL_Color checkerDark{83, 83, 83, 255};
    inline constexpr SDL_Color grid{82, 82, 82, 255};
    inline constexpr std::array<SDL_Color, 3> axes{{{240, 130, 124, 255}, {150, 206, 135, 255}, {131, 177, 239, 255}}};

    inline bool SetColor(SDL_Renderer* renderer, SDL_Color color)
    {
        return SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    }
}
