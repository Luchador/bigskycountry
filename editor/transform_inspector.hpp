#pragma once

#include "mesh.hpp"
#include <optional>
#include <string_view>

class WorldDocument;

enum class SelectionKind { None, Road, Point, Object, Terrain };
struct TransformTarget
{
    SelectionKind kind = SelectionKind::None;
    int index = -1;
    int point = -1;
    bool operator==(const TransformTarget&) const = default;
};

struct TransformFields
{
    TransformTarget target;
    ObjectTransform value;
    bool CanEdit(int field) const;
    double DisplayValue(int field) const;
    std::string Text(int field) const;
    std::string Title() const;
    bool operator==(const TransformFields&) const = default;
};

struct TransformEdit
{
    TransformTarget target;
    int field;
    std::string text;
};

TransformFields Inspect(const WorldDocument& world, TransformTarget target);
bool ApplyTransform(WorldDocument& world, const TransformFields& fields, int field, double value);
std::optional<double> ParseTransformValue(std::string_view text);
