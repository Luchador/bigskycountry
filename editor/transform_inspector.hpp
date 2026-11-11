#pragma once

#include "mesh.hpp"
#include "scene_hierarchy.hpp"
#include <optional>
#include <string_view>

class WorldDocument;
struct Road;

enum class SelectionKind { None, Road, Point, Object, Terrain, Intersection, Anchor, RoadGroup };
struct TransformTarget
{
    SelectionKind kind = SelectionKind::None;
    int index = -1;
    int point = -1;
    std::set<SceneKey> members; // Identifies the exact group, including for queued inspector edits.
    bool YawOnly() const
    {
        return kind == SelectionKind::Intersection ||
            (kind == SelectionKind::RoadGroup && members.lower_bound({SceneKind::Intersection, 0}) != members.end());
    }
    bool operator==(const TransformTarget&) const = default;
};

struct TransformFields
{
    TransformTarget target;
    ObjectTransform value;
    std::string selectionLabel;
    bool roadBound = false;
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
bool SetPlacementTransform(WorldDocument& world, TransformTarget target, const ObjectTransform& transform);
bool ValidRoadSpan(const Road& road, Vector3 a, Vector3 b);
bool ApplyTransform(WorldDocument& world, const TransformFields& fields, int field, double value);
std::optional<double> ParseTransformValue(std::string_view text);
