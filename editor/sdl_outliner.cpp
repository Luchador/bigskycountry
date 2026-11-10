#include "sdl_outliner.hpp"
#include "editor_theme.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    constexpr int header = 46, rowHeight = 20;
    bool Fill(SDL_Renderer* r, float x, float y, float w, float h, SDL_Color c)
    {
        SDL_FRect rect{x, y, w, h};
        return SDL_SetRenderDrawColor(r, c.r, c.g, c.b, 255) && SDL_RenderFillRect(r, &rect);
    }
    bool Text(SDL_Renderer* r, float x, float y, std::string_view s, SDL_Color c)
    {
        return SDL_SetRenderDrawColor(r, c.r, c.g, c.b, 255) && SDL_RenderDebugText(r, x, y, std::string(s).c_str());
    }
    int Limit(int height, const SceneOutliner& m)
    {
        return std::max(0, static_cast<int>(m.Rows().size()) - std::max(1, (height - header) / rowHeight));
    }
    int Index(const SceneOutliner& m, SceneKey key)
    {
        const auto& rows = m.Rows();
        const auto it = std::find_if(rows.begin(), rows.end(), [&](const auto& row) { return row.key == key; });
        return it == rows.end() ? -1 : static_cast<int>(it - rows.begin());
    }
    bool Folder(const SceneOutliner& m)
    {
        return m.Selection().size() == 1 && m.Selection().begin()->kind == SceneKind::Folder;
    }
    int Indent(int depth, int width)
    {
        return 5 + std::min(depth * 13, std::max(0, width * 2 / 3 - 54));
    }
} // namespace

void SdlOutliner::Blur(SDL_Window* window)
{
    if (edit)
    {
        SDL_StopTextInput(window);
    }
    edit.reset();
    focused = dragging = singleOnRelease = false;
    previousClick.reset();
    pressed.reset();
    drop.reset();
}

void SdlOutliner::StartName(SDL_Window* window, bool create, const SceneOutliner& m)
{
    if (!create && !Folder(m))
    {
        return;
    }
    const int parent = m.NewFolderParent();
    const SceneKey key = create ? (parent ? SceneKey{SceneKind::Folder, parent} : SceneKey{}) : *m.Selection().begin();
    edit = OutlineCommand{create ? OutlineOp::Create : OutlineOp::Rename, key};
    if (create)
    {
        text = m.NewFolderName();
    }
    else
    {
        text = m.Rows()[Index(m, key)].label;
    }
    selectText = true;
    SDL_StartTextInput(window);
}

bool SdlOutliner::Key(SDL_Window* window, const SDL_Event& e, const SceneOutliner& m,
                      std::optional<OutlineCommand>& cmd)
{
    if (!focused)
    {
        return false;
    }
    if (e.type == SDL_EVENT_WINDOW_FOCUS_LOST)
    {
        Blur(window);
        return false;
    }
    if (edit && e.type == SDL_EVENT_TEXT_INPUT)
    {
        if (selectText)
        {
            text.clear();
        }
        selectText = false;
        if (text.size() + std::char_traits<char>::length(e.text.text) <= 255)
        {
            text += e.text.text;
        }
        return true;
    }
    if (e.type == SDL_EVENT_KEY_UP)
    {
        return true;
    }
    if (e.type != SDL_EVENT_KEY_DOWN)
    {
        return false;
    }
    const auto k = e.key.key;
    if (edit)
    {
        if (k == SDLK_RETURN || k == SDLK_KP_ENTER || k == SDLK_ESCAPE)
        {
            if (k != SDLK_ESCAPE && !text.empty())
            {
                edit->name = text;
                cmd = std::move(edit);
            }
            edit.reset();
            SDL_StopTextInput(window);
        }
        else if (k == SDLK_A && (e.key.mod & SDL_KMOD_CTRL))
        {
            selectText = true;
        }
        else if (k == SDLK_BACKSPACE || k == SDLK_DELETE)
        {
            if (selectText)
            {
                text.clear();
            }
            else if (k == SDLK_BACKSPACE && !text.empty())
            {
                auto i = text.size() - 1;
                while (i && (static_cast<unsigned char>(text[i]) & 0xC0) == 0x80)
                {
                    --i;
                }
                text.resize(i);
            }
            selectText = false;
        }
        else if (k == SDLK_V && (e.key.mod & SDL_KMOD_CTRL))
        {
            char* clip = SDL_GetClipboardText();
            if (clip)
            {
                if (selectText)
                {
                    text.clear();
                }
                selectText = false;
                if (text.size() + std::char_traits<char>::length(clip) <= 255)
                {
                    text += clip;
                }
                SDL_free(clip);
            }
        }
        return true;
    }
    if ((e.key.mod & SDL_KMOD_CTRL) && (k == SDLK_Z || k == SDLK_Y || k == SDLK_S))
    {
        return false;
    }
    if (!m.Rows().empty() && (k == SDLK_UP || k == SDLK_DOWN))
    {
        const int current = m.Selection().empty() ? -1 : Index(m, *m.Selection().begin());
        const int next = std::clamp(current + (k == SDLK_UP ? -1 : 1), 0, static_cast<int>(m.Rows().size()) - 1);
        anchor = m.Rows()[next].key;
        cmd = OutlineCommand{OutlineOp::Select, {}, {*anchor}};
        return true;
    }
    if (m.Selection().size() == 1 && (k == SDLK_LEFT || k == SDLK_RIGHT))
    {
        const int i = Index(m, *m.Selection().begin());
        if (i >= 0 && m.Rows()[i].branch && (k == SDLK_LEFT) == m.Rows()[i].expanded)
        {
            cmd = OutlineCommand{OutlineOp::Expand, m.Rows()[i].key};
        }
        return true;
    }
    if (!e.key.repeat && k == SDLK_F2)
    {
        StartName(window, false, m);
    }
    if (!e.key.repeat && k == SDLK_DELETE && m.Selection().size() == 1 &&
        (Folder(m) || m.Selection().begin()->kind == SceneKind::Object))
    {
        cmd = OutlineCommand{OutlineOp::Remove, *m.Selection().begin()};
    }
    if (k == SDLK_ESCAPE)
    {
        Blur(window);
    }
    return true;
}

void SdlOutliner::Pointer(SDL_Window* window, const SDL_Event& e, SDL_FPoint p, int width, int height,
                          const SceneOutliner& m, std::optional<OutlineCommand>& cmd)
{
    const auto& rows = m.Rows();
    const bool inside = p.x >= 0 && p.x < width && p.y >= 0 && p.y < height;
    if (e.type == SDL_EVENT_MOUSE_WHEEL)
    {
        const float delta = e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -e.wheel.y : e.wheel.y;
        scroll = std::clamp(scroll - static_cast<int>(std::round(3 * delta)), 0, Limit(height, m));
        return;
    }
    if (pressed && e.type == SDL_EVENT_MOUSE_MOTION && std::hypot(p.x - start.x, p.y - start.y) >= 4)
    {
        dragging = true;
        if (inside && p.y >= header)
        {
            if (p.y < header + 6)
            {
                scroll = std::max(0, scroll - 1);
            }
            if (p.y > height - 7)
            {
                scroll = std::min(Limit(height, m), scroll + 1);
            }
        }
    }
    const int i = inside && p.y >= header ? scroll + static_cast<int>((p.y - header) / rowHeight) : -1;
    const OutlineRow* row = i >= 0 && i < static_cast<int>(rows.size()) ? &rows[i] : nullptr;
    if (dragging)
    {
        drop.reset();
        if (row && (row->key.kind == SceneKind::World || row->key.kind == SceneKind::Folder))
        {
            drop = row->key;
        }
    }
    if (e.type == SDL_EVENT_MOUSE_BUTTON_UP && e.button.button == SDL_BUTTON_LEFT)
    {
        if (dragging && drop)
        {
            cmd = OutlineCommand{OutlineOp::Move, *drop, m.Selection()};
        }
        else if (singleOnRelease && !dragging && pressed && row && row->key == *pressed)
        {
            cmd = OutlineCommand{OutlineOp::Select, {}, {*pressed}};
        }
        pressed.reset();
        drop.reset();
        dragging = singleOnRelease = false;
        return;
    }
    if (e.type != SDL_EVENT_MOUSE_BUTTON_DOWN || e.button.button != SDL_BUTTON_LEFT || !inside)
    {
        return;
    }
    focused = true;
    if (edit)
    {
        edit.reset();
        SDL_StopTextInput(window);
    }
    if (p.y < 25)
    {
        if (p.x >= width - 52 && Folder(m))
        {
            cmd = OutlineCommand{OutlineOp::Remove, *m.Selection().begin()};
        }
        else if (p.x >= width - 110 && p.x < width - 52)
        {
            StartName(window, false, m);
        }
        else if (p.x >= width - 202 && p.x < width - 110)
        {
            StartName(window, true, m);
        }
        return;
    }
    if (!row)
    {
        if (p.y >= header)
        {
            cmd = OutlineCommand{OutlineOp::Select};
        }
        return;
    }
    const bool doubleClick = e.button.clicks == 2 && previousClick == row->key;
    previousClick = row->key;
    if (row->branch && (p.x < Indent(row->depth, width) + 12 || doubleClick))
    {
        cmd = OutlineCommand{OutlineOp::Expand, row->key};
        return;
    }
    const auto mods = SDL_GetModState();
    auto sel = m.Selection();
    const int a = anchor ? Index(m, *anchor) : -1;
    if ((mods & SDL_KMOD_SHIFT) && a >= 0)
    {
        if (!(mods & SDL_KMOD_CTRL))
        {
            sel.clear();
        }
        for (int j = std::min(a, i); j <= std::max(a, i); ++j)
        {
            sel.insert(rows[j].key);
        }
    }
    else if (mods & SDL_KMOD_CTRL)
    {
        if (!sel.erase(row->key))
        {
            sel.insert(row->key);
        }
        anchor = row->key;
    }
    else
    {
        singleOnRelease = sel.size() > 1 && sel.contains(row->key);
        if (!singleOnRelease)
        {
            sel = {row->key};
        }
        anchor = row->key;
    }
    if (sel.contains(row->key))
    {
        pressed = row->key;
        start = p;
    }
    cmd = OutlineCommand{OutlineOp::Select, {}, std::move(sel)};
}

bool SdlOutliner::Draw(SDL_Renderer* r, const SceneOutliner& m, int width, int height)
{
    constexpr auto ink = EditorTheme::text, muted = EditorTheme::secondary;
    if (!Fill(r, 0, 0, width, height, EditorTheme::list) || !Fill(r, 0, 0, width, 25, EditorTheme::header) ||
        !Text(r, 7, 8, "Outliner", ink) || !Text(r, width - 198, 8, "New folder", ink) ||
        !Text(r, width - 108, 8, "Rename", Folder(m) ? ink : muted) ||
        !Text(r, width - 51, 8, "Delete", Folder(m) ? ink : muted))
    {
        return false;
    }
    const int split = width * 2 / 3;
    if (!Fill(r, 0, 25, width, 21, EditorTheme::chrome))
    {
        return false;
    }
    if (edit)
    {
        const int x = edit->op == OutlineOp::Create ? 45 : 70;
        if (!Text(r, 6, 31, edit->op == OutlineOp::Create ? "New:" : "Rename:", muted) ||
            !Fill(r, x, 27, width - x - 4, 16, selectText ? EditorTheme::selected : EditorTheme::field) ||
            !Text(r, x + 3, 31, text, ink))
        {
            return false;
        }
    }
    else if (!Text(r, 7, 31, "Item label", muted) || !Text(r, split + 4, 31, "Type", muted))
    {
        return false;
    }
    const std::optional<SceneKey> single =
        m.Selection().size() == 1 ? std::optional{*m.Selection().begin()} : std::nullopt;
    if (single && lastSingle != single)
    {
        const int i = Index(m, *m.Selection().begin());
        if (i >= 0)
        {
            scroll = std::clamp(scroll, std::max(0, i - std::max(1, (height - header) / rowHeight) + 1), i);
        }
    }
    lastSingle = single;
    scroll = std::clamp(scroll, 0, Limit(height, m));
    for (int i = scroll, y = header; i < static_cast<int>(m.Rows().size()) && y < height; ++i, y += rowHeight)
    {
        const auto& row = m.Rows()[i];
        if (drop == row.key)
        {
            if (!Fill(r, 0, y, width, rowHeight, EditorTheme::hover))
            {
                return false;
            }
        }
        else if (m.Selection().contains(row.key))
        {
            if (!Fill(r, 0, y, width, rowHeight, EditorTheme::selected))
            {
                return false;
            }
        }
        else if (i % 2 && !Fill(r, 0, y, width, rowHeight, EditorTheme::alternateRow))
        {
            return false;
        }
        const int x = Indent(row.depth, width);
        if (row.branch && !Text(r, x, y + 6, row.expanded ? "v" : ">", muted))
        {
            return false;
        }
        if (!Text(r, x + 13, y + 6, row.label.substr(0, std::max(0, (split - x - 15) / 8)), ink) ||
            !Text(r, split + 4, y + 6, row.type, muted))
        {
            return false;
        }
    }
    return true;
}
