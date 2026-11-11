#pragma once

#include "transform_inspector.hpp"

class SdlTransformPanel
{
public:
    static constexpr int height = 112;
    bool Focused() const { return field >= 0; }
    void Blur(SDL_Window* window);
    void Click(SDL_Window* window, SDL_FPoint pos, int width, const TransformFields& fields);
    bool Key(SDL_Window* window, const SDL_Event& event, std::optional<InspectorEdit>& commit);
    void StartEdit(SDL_Window* window, TransformTarget selection, int id, std::string value);
    bool DrawField(SDL_Renderer* renderer, SDL_FRect cell, const TransformTarget& selection, int id,
                   const std::string& value, bool enabled) const;
    bool Draw(SDL_Renderer* renderer, int width, const TransformFields& fields) const;

private:
    bool geographic = false;
    int field = -1;
    TransformTarget target;
    std::string buffer;
    bool selectAll = false, dirty = false;
    std::size_t caret = 0;
};
