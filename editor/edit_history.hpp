#pragma once

#include "road.hpp"
#include "transform_inspector.hpp"

#include <set>
#include <span>
#include <string>

// Editable tool state. Camera, view, hover and generated geometry stay out of history.
struct RoadEditorState
{
    RoadStyle defaults;
    std::optional<Road> draft;
    int selected = -1;
    int point = -1; // Active point for the inspector and gizmo.
    std::set<int> additionalPoints;
    bool drawing = false;
    std::set<int> segments; // Indices of selected spans in the active road.

    TransformTarget meshSelection;
    TransformTarget Target() const
    {
        if (selected < 0) return meshSelection;
        return {point < 0 ? SelectionKind::Road : SelectionKind::Point, selected, point};
    }

    bool PointSelected(int index) const
    {
        return index == point || additionalPoints.contains(index);
    }

    void PickPoint(int roadIndex, int index, bool add, bool subtract);

    void SelectPoint(int roadIndex, int pointIndex)
    {
        selected = roadIndex;
        point = pointIndex;
        additionalPoints.clear();
        segments.clear();
        meshSelection = {};
    }
};

struct EditSnapshot
{
    EditSnapshot(const RoadEditorState& editor, const Road* affectedRoad = nullptr);
    EditSnapshot(const RoadEditorState& editor, std::span<const Road> affectedRoads);
    TransformTarget meshSelection;
    std::optional<ObjectTransform> meshTransform;
    struct TerrainPlacement
    {
        std::string modelPath;
        ObjectTransform transform;
        bool operator==(const TerrainPlacement&) const = default;
    };
    std::optional<TerrainPlacement> addedTerrain;
    bool operator==(const EditSnapshot&) const = default;

    std::string editorData;
    std::vector<std::string> roadData; // Source records for one contiguous range.
    std::set<int> additionalPoints;
    std::set<int> segments;
};

class EditHistory
{
public:
    enum class Result { Unavailable, ToolChanged, MapChanged, Failed };

    bool CanUndo() const { return cursor != 0; }
    bool CanRedo() const { return cursor < actions.size(); }
    bool Record(int roadIndex, EditSnapshot before, EditSnapshot after);
    void ReplaceSelectedRoad(WorldDocument& world, RoadEditorState& editor, std::vector<Road> replacements);
    Result Undo(WorldDocument& world, RoadEditorState& editor);
    Result Redo(WorldDocument& world, RoadEditorState& editor);

private:
    struct Action
    {
        int roadIndex; // -1 for a tool or draft edit.
        EditSnapshot before;
        EditSnapshot after;
    };
    static Result Restore(WorldDocument& world, RoadEditorState& editor, int index,
                          const EditSnapshot& from, const EditSnapshot& to);
    std::vector<Action> actions;
    std::size_t cursor = 0; // Number of applied actions.
};
