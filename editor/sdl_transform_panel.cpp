#include "sdl_transform_panel.hpp"

#include "editor_theme.hpp"
#include "marker.hpp"
#include <utility>
#include <algorithm>

namespace
{
    constexpr SDL_FRect toggle{6, 3, 218, 19};
    SDL_FRect Cell(int width, int field, bool geographic)
    {
        const float w = std::max(1.0f, (width - 86) / 3.0f);
        if (geographic)
        {
            const float end = width - 8.0f;
            const float heightWidth = std::max(56.0f, w);
            if (field == TransformFields::geographicLocation) return {7, 35, end - heightWidth - 11, 19};
            if (field == 1) return {end - heightWidth, 35, heightWidth, 19};
        }
        return {78 + (field % 3) * w, (geographic ? 45.0f : 41.0f) + (field / 3) * 22, w - 2, 19};
    }
}

bool SdlTransformPanel::Draw(SDL_Renderer* r, int width, const TransformFields& fields) const
{
    const SDL_FRect background{0, 0, static_cast<float>(width), height};
    if (!EditorTheme::SetColor(r, EditorTheme::panel) || !SDL_RenderFillRect(r, &background) ||
        !EditorTheme::SetColor(r, EditorTheme::text) ||
        !SDL_RenderDebugText(r, 26, 8, "Use Geographic Coordinates")) return false;
    const SDL_FRect box{8, 6, 13, 13};
    if (!EditorTheme::SetColor(r, geographic ? EditorTheme::selected : EditorTheme::field) ||
        !SDL_RenderFillRect(r, &box) || !EditorTheme::SetColor(r, EditorTheme::secondary) ||
        !SDL_RenderRect(r, &box)) return false;
    if (geographic)
    {
        if (!SDL_RenderLine(r, 10, 12, 13, 15) || !SDL_RenderLine(r, 13, 15, 19, 8) ||
            !SDL_RenderDebugText(r, 7, 24, "Latitude, longitude") ||
            !SDL_RenderDebugText(r, Cell(width, 1, true).x, 24, "Elev (m)")) return false;
    }
    const char* labels[] = {"Loc (m)", "Rot (deg)", "Scale"};

    for (int axis = 0; axis < 3; ++axis)
    {
        const auto cell = Cell(width, axis + 3, geographic);
        if (!EditorTheme::SetColor(r, EditorTheme::axes[axis]) ||
            !SDL_RenderDebugTextFormat(r, cell.x + cell.w / 2 - 4, geographic ? 57 : 26, "%c", 'X' + axis)) return false;
    }
    for (int i = 0; i <= (geographic ? TransformFields::geographicLocation : 8); ++i)
    {
        if (geographic && (i == 0 || i == 2)) continue;
        const SDL_FRect cell = Cell(width, i, geographic);
        if (i < 9 && i % 3 == 0 && (!EditorTheme::SetColor(r, EditorTheme::secondary) ||
                           !SDL_RenderDebugText(r, 6, cell.y + 5, labels[i / 3])))
        {
            return false;
        }

        if (!DrawField(r, cell, fields.target, i, fields.Text(i), fields.CanEdit(i))) return false;
    }

    return true;
}

bool SdlTransformPanel::DrawField(SDL_Renderer* r, SDL_FRect cell, const TransformTarget& selection, int id,
                                  const std::string& value, bool enabled) const
{
    const bool active = selection == target && id == field;
    if (!EditorTheme::SetColor(r, active ? EditorTheme::selected : EditorTheme::field) || !SDL_RenderFillRect(r, &cell)) return false;
    const auto& text = active ? buffer : value;
    const size_t visible = static_cast<size_t>(std::max(1.0f, (cell.w - 4) / 8));
    const size_t column = active ? SDL_utf8strnlen(text.data(), caret) : 0;
    const size_t start = column >= visible ? column - visible + 1 : 0;
    const char* line = text.c_str();
    for (size_t i = 0; i < start; ++i) SDL_StepUTF8(&line, nullptr);
    const char* end = line;
    for (size_t i = 0; i < visible && *end; ++i) SDL_StepUTF8(&end, nullptr);
    if (!EditorTheme::SetColor(r, enabled ? EditorTheme::text : EditorTheme::disabled) ||
        !SDL_RenderDebugText(r, cell.x + 2, cell.y + 5, std::string(line, end).c_str())) return false;
    const float x = cell.x + 2 + (column - start) * 8;
    return !active || SDL_RenderLine(r, x, cell.y + 3, x, cell.y + 15);
}

void SdlTransformPanel::StartEdit(SDL_Window* window, TransformTarget selection, int id, std::string value)
{
    target = selection;
    field = id;
    initialText = value;
    buffer = std::move(value);
    selectAll = true;
    dirty = false;
    caret = buffer.size();
    SDL_StartTextInput(window);
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

    if (SDL_PointInRectFloat(&pos, &toggle))
    {
        geographic = !geographic;
        return;
    }
    for (int i = 0; i <= (geographic ? TransformFields::geographicLocation : 8); ++i)
    {
        if (geographic && (i == 0 || i == 2)) continue;
        const auto rect = Cell(width, i, geographic);

        if (!fields.CanEdit(i) || !SDL_PointInRectFloat(&pos, &rect)) 
        {
            continue;
        }

        StartEdit(window, fields.target, i, fields.Text(i));
        return;
    }
}

bool SdlTransformPanel::Key(SDL_Window* window, const SDL_Event& event, std::optional<InspectorEdit>& commit)
{
    if (!Focused()) 
    {
        return false;
    }

    const auto insert = [&](std::string_view text)
    {
        const auto current = selectAll ? 0 : SDL_utf8strlen(buffer.c_str());
        const auto limit = field == InspectorEdit::markerText ? Marker::maxCharacters : 120;
        if (current + SDL_utf8strnlen(text.data(), text.size()) > limit) return;
        if (selectAll)
        {
            buffer.clear();
            caret = 0;
            selectAll = false;
        }
        dirty = true;
        buffer.insert(caret, text);
        caret += text.size();
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

    const auto neighbor = [&](bool forward)
    {
        const char* p = buffer.data() + caret;
        if (forward) SDL_StepUTF8(&p, nullptr);
        else SDL_StepBackUTF8(buffer.data(), &p);
        return static_cast<size_t>(p - buffer.data());
    };
    switch (event.key.key)
    {
    case SDLK_RETURN:
    case SDLK_KP_ENTER:
        if (dirty) 
        {
            commit = InspectorEdit{target, field, buffer};
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
                if (field == TransformFields::geographicLocation && std::string_view(text).find(',') != std::string_view::npos)
                    selectAll = true;
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
            const auto prev = neighbor(false);
            buffer.erase(prev, caret - prev);
            caret = prev;
        }
        else if (event.key.key == SDLK_DELETE && caret < buffer.size()) 
        {
            buffer.erase(caret, neighbor(true) - caret);
        }
        break;
    case SDLK_LEFT:
        if (caret) 
        {
            caret = neighbor(false);
        }
        selectAll = false;
        break;
    case SDLK_RIGHT:
        if (caret < buffer.size()) 
        {
            caret = neighbor(true);
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
