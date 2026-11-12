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
    static constexpr int geographicLocation = 9; // A single latitude/longitude edit affecting X and Z.
    bool roadBound = false;
    bool CanEdit(int field) const;
    double DisplayValue(int field) const;
    std::string Text(int field) const;
    std::optional<ObjectTransform> Parse(int field, std::string_view text) const;
    bool operator==(const TransformFields&) const = default;
};

struct InspectorEdit
{
    static constexpr int markerText = TransformFields::geographicLocation + 1;
    static constexpr int markerTextColor = markerText + 1;
    static constexpr int roadNames = markerTextColor + 1;
    TransformTarget target;
    int field;
    std::string text;
};

TransformFields Inspect(const WorldDocument& world, TransformTarget target);
bool SetPlacementTransform(WorldDocument& world, TransformTarget target, const ObjectTransform& transform);
bool ValidRoadSpan(const Road& road, Vector3 a, Vector3 b);
bool ApplyTransform(WorldDocument& world, const TransformFields& fields, int field, const ObjectTransform& desired);
std::optional<double> ParseTransformValue(std::string_view text);
