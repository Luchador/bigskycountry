#include "sdl_transform_panel.hpp"

#include <algorithm>

namespace
{
    SDL_FRect Cell(int width, int field)
    {
        const float w = std::max(1.0f, (width - 86) / 3.0f);
        return {78 + (field % 3) * w, 41.0f + (field / 3) * 22, w - 2, 19};
    }
}

bool SdlTransformPanel::Draw(SDL_Renderer* r, int width, const TransformFields& fields) const
{
    const SDL_FRect background{0, 0, static_cast<float>(width), height};
    if (!SDL_SetRenderDrawColor(r, 24, 32, 38, 255) || !SDL_RenderFillRect(r, &background) ||
        !SDL_SetRenderDrawColor(r, 221, 228, 234, 255) ||
        !SDL_RenderDebugTextFormat(r, 8, 8, "Transform | %s", fields.Title().c_str())) return false;
    const char* labels[] = {"Loc (m)", "Rot (deg)", "Scale"};

    for (int i = 0; i < 9; ++i)
    {
        const SDL_FRect cell = Cell(width, i);

        if (!SDL_SetRenderDrawColor(r, 185, 197, 205, 255)) 
        {
            return false;
        }

        if (i < 3 && !SDL_RenderDebugTextFormat(r, cell.x + cell.w / 2 - 4, 26, "%c", 'X' + i)) 
        {
            return false;
        }

        if (i % 3 == 0 && !SDL_RenderDebugText(r, 6, cell.y + 5, labels[i / 3])) 
        {
            return false;
        }

        const bool active = fields.target == target && i == field;
        const bool enabled = fields.CanEdit(i);

        if (!SDL_SetRenderDrawColor(r, active ? 44 : 15, active ? 79 : 23, active ? 84 : 28, 255) || !SDL_RenderFillRect(r, &cell)) 
        {
            return false;
        }

        if (enabled)
        {
            std::string text = active ? buffer : fields.Text(i);
            const auto visible = static_cast<std::size_t>(std::max(1.0f, (cell.w - 4) / 8));
            std::size_t start = active && caret >= visible ? caret - visible + 1 : 0;
            text = text.substr(start, visible);

            if (!SDL_SetRenderDrawColor(r, 221, 228, 234, 255) || !SDL_RenderDebugText(r, cell.x + 2, cell.y + 5, text.c_str())) 
            {
                return false;
            }

            if (active)
            {
                const float x = cell.x + 2 + (caret - start) * 8;
                if (!SDL_RenderLine(r, x, cell.y + 3, x, cell.y + 15)) 
                {
                    return false;
                }
            }
        }
    }

    return true;
}

void SdlTransformPanel::Blur(SDL_Window* window)
{
    if (Focused()) 
    {
        SDL_StopTextInput(window);
    }

    field = -1;
}

void SdlTransformPanel::Click(SDL_Window* window, SDL_FPoint pos, int width, const TransformFields& fields)
{
    Blur(window);

    for (int i = 0; i < 9; ++i)
    {
        const auto rect = Cell(width, i);

        if (!fields.CanEdit(i) || !SDL_PointInRectFloat(&pos, &rect)) 
        {
            continue;
        }

        target = fields.target;
        field = i;
        buffer = fields.Text(i);
        selectAll = true;
        dirty = false;
        caret = buffer.size();
        SDL_StartTextInput(window);
    
        return;
    }
}

bool SdlTransformPanel::Key(SDL_Window* window, const SDL_Event& event, std::optional<TransformEdit>& commit)
{
    if (!Focused()) 
    {
        return false;
    }

    const auto insert = [&](std::string_view text)
    {
        if (selectAll)
        {
            buffer.clear();
            caret = 0;
            selectAll = false;
        }

        if (buffer.size() + text.size() <= 120)
        {
            dirty = true;
            buffer.insert(caret, text);
            caret += text.size();
        }
    };

    if (event.type == SDL_EVENT_TEXT_INPUT)
    {
        insert(event.text.text);
        return true;
    }

    if (event.type == SDL_EVENT_KEY_UP) 
    {
        return true;
    }

    if (event.type != SDL_EVENT_KEY_DOWN) 
    {
        return false;
    }

    switch (event.key.key)
    {
    case SDLK_RETURN:
    case SDLK_KP_ENTER:
        if (dirty) 
        {
            commit = TransformEdit{target, field, buffer};
        }
        Blur(window);
        break;
    case SDLK_ESCAPE:
        Blur(window);
        break;
    case SDLK_A:
        if (event.key.mod & SDL_KMOD_CTRL) selectAll = true;
        break;
    case SDLK_V:
        if (event.key.mod & SDL_KMOD_CTRL)
        {
            char* text = SDL_GetClipboardText();
            if (text)
            {
                insert(text);
                SDL_free(text);
            }
        }
        break;
    case SDLK_BACKSPACE:
    case SDLK_DELETE:
        dirty = true;
        if (selectAll) 
        {
            insert("");
        }
        else if (event.key.key == SDLK_BACKSPACE && caret) 
        {
            buffer.erase(--caret, 1);
        }
        else if (event.key.key == SDLK_DELETE && caret < buffer.size()) 
        {
            buffer.erase(caret, 1);
        }
        break;
    case SDLK_LEFT:
        if (caret) 
        {
            --caret;
        }
        selectAll = false;
        break;
    case SDLK_RIGHT:
        if (caret < buffer.size()) 
        {
            ++caret;
        }
        selectAll = false;
        break;
    case SDLK_HOME:
        caret = 0;
        selectAll = false;
        break;
    case SDLK_END:
        caret = buffer.size();
        selectAll = false;
        break;
    }
    return true;
}
