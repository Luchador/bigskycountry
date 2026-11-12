#pragma once
#include "material_editor.hpp"
#include <optional>

class SdlMaterialEditor
{
public:
    bool Open(SDL_Window* owner, TextureCatalog& catalog, const std::string& path);
    void StartDrag(const std::string& path);
    bool Handle(const SDL_Event& event);
    bool Draw();
    bool Focused() const { return window && SDL_GetKeyboardFocus() == window; }

private:
    void Close();
    void FinishDrag();
    void AdjustSlider(float x, bool commit);
    bool UpdateImages(int pixelWidth, int pixelHeight);
    void Report(bool success);
    MaterialEditor editor;
    SDL_Window* window = nullptr; // Owned utility window, also destroyed when its parent closes.
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* sphere = nullptr;
    std::array<SDL_Texture*, materialSlotCount> thumbs{};
    bool dirty = false, imagesDirty = false, sphereDirty = false, typeMenu = false;
    std::optional<SDL_FPoint> orbitPoint;
    std::optional<int> activeSlider; // RGB channels followed by the translucency multiplier.
    std::string dragged;
    std::optional<MaterialSlot> hover;
};
