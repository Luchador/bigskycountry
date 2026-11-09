#pragma once

#include "road.hpp"
#include "transform_inspector.hpp"

#include <set>
#include <string>

// Editable tool state. Camera, view, hover and generated geometry stay out of history.
struct RoadEditorState
{
    RoadStyle defaults;
    std::optional<Road> draft;
    int selected = -1;
    int point = -1;
    bool drawing = false;
    std::set<int> segments; // Indices of selected spans in the active road.

    TransformTarget meshSelection;
    TransformTarget Target() const
    {
        if (selected < 0) return meshSelection;
        return {point < 0 ? SelectionKind::Road : SelectionKind::Point, selected, point};
    }

    void SelectPoint(int roadIndex, int pointIndex)
    {
        selected = roadIndex;
        point = pointIndex;
        segments.clear();
        meshSelection = {};
    }
};

struct EditSnapshot
{
    EditSnapshot(const RoadEditorState& editor, const Road* affectedRoad = nullptr);
    TransformTarget meshSelection;
    std::optional<ObjectTransform> meshTransform;
    bool operator==(const EditSnapshot&) const = default;

    std::string editorData;
    std::optional<std::string> roadData;
    std::set<int> segments;
};

class EditHistory
{
public:
    enum class Result { Unavailable, ToolChanged, MapChanged };

    bool CanUndo() const { return cursor != 0; }
    bool CanRedo() const { return cursor < actions.size(); }
    bool Record(int roadIndex, EditSnapshot before, EditSnapshot after);
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
