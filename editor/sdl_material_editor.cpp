#include "sdl_material_editor.hpp"
#include "editor_theme.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    struct Layout
    {
        int windowWidth = 0, windowHeight = 0;
        SDL_FRect view, type;
        std::array<SDL_FRect, materialSlotCount> slots{}, clear{};
        explicit Layout(SDL_Window* window)
        {
            SDL_GetWindowSize(window, &windowWidth, &windowHeight);
            const float split = float(windowWidth) - 224;
            view = {0, 0, split, float(windowHeight)};
            type = {split + 10, 32, 204, 25};
            const float step = std::min(80.f, (windowHeight - 70.f) / materialSlotCount);
            for (std::size_t i = 0; i < slots.size(); ++i)
            {
                slots[i] = {split + 10, 68 + i * step, 160, step - 5};
                clear[i] = {split + 173, slots[i].y + (step - 27) * .5f, 41, 22};
            }
        }
        std::optional<MaterialSlot> Slot(SDL_FPoint p) const
        {
            for (std::size_t i = 0; i < slots.size(); ++i)
                if (SDL_PointInRectFloat(&p, &slots[i]))
                    return static_cast<MaterialSlot>(i);
            return {};
        }
        SDL_FRect Choice(int i) const { return {type.x, type.y + (1 + i) * type.h, type.w, type.h}; }
    };

    bool Fill(SDL_Renderer* r, SDL_FRect box, SDL_Color color)
    {
        return EditorTheme::SetColor(r, color) && SDL_RenderFillRect(r, &box);
    }
    bool Text(SDL_Renderer* r, float x, float y, const std::string& text, SDL_Color color)
    {
        return EditorTheme::SetColor(r, color) && SDL_RenderDebugText(r, x, y, text.c_str());
    }
}

bool SdlMaterialEditor::Open(SDL_Window* owner, TextureCatalog& catalog, const std::string& path)
{
    if (!editor.Open(catalog, path))
        return false;
    if (!window)
    {
        const auto props = SDL_CreateProperties();
        SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_PARENT_POINTER, owner);
        SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_UTILITY_BOOLEAN, true);
        SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_RESIZABLE_BOOLEAN, true);
        SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_HIGH_PIXEL_DENSITY_BOOLEAN, true);
        SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, 660);
        SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, 460);
        int x, y, w, h;
        SDL_GetWindowPosition(owner, &x, &y);
        SDL_GetWindowSize(owner, &w, &h);
        SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_X_NUMBER, x + std::max(0, (w - 660) * 2 / 3));
        SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_Y_NUMBER, y + std::max(0, (h - 460) / 2));
        window = SDL_CreateWindowWithProperties(props);
        SDL_DestroyProperties(props);
        if (!window)
            return false;
        SDL_SetWindowMinimumSize(window, 440, 350);
        renderer = SDL_CreateRenderer(window, nullptr);
        if (!renderer)
        {
            Close();
            return false;
        }
    }
    SDL_SetWindowTitle(window, ("Material Editor - " + path).c_str());
    SDL_RaiseWindow(window);
    imagesDirty = dirty = true;
    typeMenu = false;
    return true;
}

void SdlMaterialEditor::FinishDrag()
{
    if (orbitPoint || !dragged.empty())
        SDL_CaptureMouse(false);
    orbitPoint.reset();
    dragged.clear();
    hover.reset();
    dirty = true;
}

void SdlMaterialEditor::Close()
{
    FinishDrag();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    sphere = nullptr;
    thumbs.fill(nullptr);
    renderer = nullptr;
    window = nullptr;
}

void SdlMaterialEditor::StartDrag(const std::string& path)
{
    if (!window)
        return;
    FinishDrag();
    dragged = path;
    SDL_CaptureMouse(true);
}

void SdlMaterialEditor::Report(bool success)
{
    if (!success)
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Material Editor", SDL_GetError(), window);
    else
        imagesDirty = true;
    dirty = true;
}

bool SdlMaterialEditor::Handle(const SDL_Event& e)
{
    if (!window)
        return false;
    auto* source = SDL_GetWindowFromEvent(&e);
    const bool own = source == window;
    if (orbitPoint)
    {
        const bool released = e.type == SDL_EVENT_MOUSE_BUTTON_UP && e.button.button == SDL_BUTTON_LEFT;
        if (own && (e.type == SDL_EVENT_MOUSE_MOTION || released))
        {
            if (released || (e.motion.state & SDL_BUTTON_LMASK))
            {
                const SDL_FPoint point = released ? SDL_FPoint{e.button.x, e.button.y} : SDL_FPoint{e.motion.x, e.motion.y};
                const auto view = Layout(window).view;
                if (editor.Orbit(point.x - orbitPoint->x, point.y - orbitPoint->y, std::min(view.w, view.h)))
                    sphereDirty = true;
                orbitPoint = point;
            }
            if (released || !(e.motion.state & SDL_BUTTON_LMASK)) FinishDrag();
            return true;
        }
        if (own && e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE)
        {
            FinishDrag();
            return true;
        }
        if (released || (own && e.type == SDL_EVENT_WINDOW_FOCUS_LOST) || e.type == SDL_EVENT_WILL_ENTER_BACKGROUND)
            FinishDrag();
    }
    if (!dragged.empty())
    {
        if (e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE)
        {
            FinishDrag();
            return true;
        }
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN || e.type == SDL_EVENT_WILL_ENTER_BACKGROUND)
            FinishDrag();
        if (source && (e.type == SDL_EVENT_MOUSE_MOTION ||
                       (e.type == SDL_EVENT_MOUSE_BUTTON_UP && e.button.button == SDL_BUTTON_LEFT)))
        {
            SDL_FPoint p = e.type == SDL_EVENT_MOUSE_MOTION ? SDL_FPoint{e.motion.x, e.motion.y}
                                                            : SDL_FPoint{e.button.x, e.button.y};
            if (!own)
            {
                int sx, sy, wx, wy;
                SDL_GetWindowPosition(source, &sx, &sy);
                SDL_GetWindowPosition(window, &wx, &wy);
                p.x += sx - wx;
                p.y += sy - wy;
            }
            const auto slot = Layout(window).Slot(p);
            if (slot != hover)
            {
                hover = slot;
                dirty = true;
            }
            if (e.type == SDL_EVENT_MOUSE_BUTTON_UP)
            {
                const auto path = dragged;
                FinishDrag();
                if (slot)
                    Report(editor.SetTexture(*slot, path));
            }
            else if (!(e.motion.state & SDL_BUTTON_LMASK))
                FinishDrag();
            return true;
        }
    }
    if (!own)
        return false;
    if (e.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED)
    {
        Close();
        return true;
    }
    if (e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE)
    {
        if (typeMenu)
        {
            typeMenu = false;
            dirty = true;
        }
        else
            Close();
        return true;
    }
    if (e.type == SDL_EVENT_WINDOW_EXPOSED || e.type == SDL_EVENT_WINDOW_RESIZED ||
        e.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED || e.type == SDL_EVENT_WINDOW_RESTORED)
        dirty = true;
    if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_LEFT)
    {
        const SDL_FPoint p{e.button.x, e.button.y};
        const Layout layout(window);
        dirty = true;
        if (typeMenu)
        {
            typeMenu = false;
            for (int i = 0; i < int(materialTypes.size()); ++i)
            {
                const auto choice = layout.Choice(i);
                if (SDL_PointInRectFloat(&p, &choice))
                    Report(editor.SetType(static_cast<MaterialType>(i)));
            }
        }
        else if (SDL_PointInRectFloat(&p, &layout.view))
        {
            orbitPoint = p;
            SDL_CaptureMouse(true);
        }
        else if (SDL_PointInRectFloat(&p, &layout.type))
            typeMenu = true;
        else
            for (std::size_t i = 0; i < layout.clear.size(); ++i)
                if (SDL_PointInRectFloat(&p, &layout.clear[i]) && !editor.Current().textures[i].empty())
                    Report(editor.SetTexture(static_cast<MaterialSlot>(i), {}));
    }
    return true;
}

bool SdlMaterialEditor::UpdateImages(int pixelWidth, int pixelHeight)
{
    auto* pixels = editor.Preview(pixelWidth, pixelHeight);
    if (!pixels) return false;
    if (!sphere || sphere->w != pixelWidth || sphere->h != pixelHeight)
    {
        auto* resized = SDL_CreateTexture(renderer, pixels->format, SDL_TEXTUREACCESS_STREAMING, pixelWidth, pixelHeight);
        if (!resized) return false;
        SDL_DestroyTexture(sphere);
        sphere = resized;
    }
    if (!SDL_UpdateTexture(sphere, nullptr, pixels->pixels, pixels->pitch)) return false;
    sphereDirty = false;
    if (!imagesDirty) return true;
    for (std::size_t i = 0; i < thumbs.size(); ++i)
    {
        SDL_DestroyTexture(thumbs[i]);
        auto* thumb = editor.Thumbnail(i);
        thumbs[i] = thumb ? SDL_CreateTextureFromSurface(renderer, thumb) : nullptr;
    }
    imagesDirty = false;
    return true;
}

bool SdlMaterialEditor::Draw()
{
    if (!window)
        return true;
    if (editor.ImagesChanged())
        imagesDirty = true;
    if (!dirty && !imagesDirty && !sphereDirty)
        return true;
    const Layout layout(window);
    int outputWidth, outputHeight;
    if (!SDL_GetRenderOutputSize(renderer, &outputWidth, &outputHeight)) return false;
    if (layout.view.w <= 0 || layout.windowHeight <= 0 || outputWidth <= 0 || outputHeight <= 0) return true;

    // Controls and mouse input use window coordinates. Shade the preview in drawable pixels.
    const float scaleX = float(outputWidth) / layout.windowWidth;
    const float scaleY = float(outputHeight) / layout.windowHeight;
    const int previewWidth = std::lround(layout.view.w * scaleX);
    if (!SDL_SetRenderScale(renderer, scaleX, scaleY)) return false;
    if (imagesDirty || sphereDirty || !sphere || sphere->w != previewWidth || sphere->h != outputHeight)
    {
        if (!UpdateImages(previewWidth, outputHeight)) return false;
    }

    using namespace EditorTheme;
    if (!EditorTheme::SetColor(renderer, panel) || !SDL_RenderClear(renderer) ||
        !sphere || !SDL_RenderTexture(renderer, sphere, nullptr, &layout.view) ||
        !Text(renderer, layout.type.x, 13, "Material type", text) || !Fill(renderer, layout.type, button) ||
        !Text(renderer, layout.type.x + 6, layout.type.y + 8,
              materialTypes[static_cast<std::size_t>(editor.Current().type)].label, text) ||
        !Text(renderer, layout.type.x + 184, layout.type.y + 8, "v", text))
        return false;
    for (std::size_t i = 0; i < thumbs.size(); ++i)
    {
        const auto r = layout.slots[i], c = layout.clear[i];
        const float sz = std::min(48.f, r.h - 8);
        const SDL_FRect image{r.x + 4, r.y + (r.h - sz) / 2, sz, sz};
        if (!Fill(renderer, r, hover == static_cast<MaterialSlot>(i) ? selected : list) ||
            !Fill(renderer, image, preview) || (thumbs[i] && !SDL_RenderTexture(renderer, thumbs[i], nullptr, &image)))
            return false;
        const auto& path = editor.Current().textures[i];
        const float x = image.x + sz + 6;
        const SDL_Rect clip{int(x), int(r.y), int(r.x + r.w - x), int(r.h)};
        if (!SDL_SetRenderClipRect(renderer, &clip) ||
            !Text(renderer, x, r.y + r.h / 2 - 10, materialSlotLabels[i], text) ||
            !Text(renderer, x, r.y + r.h / 2 + 5, path.empty() ? "Drop texture" : path.substr(path.rfind('/') + 1),
                  secondary) ||
            !SDL_SetRenderClipRect(renderer, nullptr))
            return false;
        if (!path.empty() && (!Fill(renderer, c, button) || !Text(renderer, c.x + 1, c.y + 7, "Clear", text)))
            return false;
    }
    if (typeMenu)
        for (int i = 0; i < int(materialTypes.size()); ++i)
        {
            const auto choice = layout.Choice(i);
            if (!Fill(renderer, choice, chrome) ||
                !Text(renderer, choice.x + 6, choice.y + 8, materialTypes[i].label, text))
                return false;
        }
    dirty = false;
    return SDL_RenderPresent(renderer);
}
