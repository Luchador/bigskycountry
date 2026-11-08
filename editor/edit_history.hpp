#pragma once

#include "road.hpp"

// Editable tool state. Camera, view, hover and generated geometry stay out of history.
struct RoadEditorState
{
    RoadStyle defaults;
    std::optional<Road> draft;
    int selected = -1;
    int point = -1;
    bool drawing = false;
};

struct EditSnapshot
{
    EditSnapshot(const RoadEditorState& editor, const Road* affectedRoad = nullptr);
    bool operator==(const EditSnapshot&) const = default;

    std::string editorData;
    std::optional<std::string> roadData;
};

class EditHistory
{
public:
    enum class Result { Unavailable, ToolChanged, MapChanged };

    bool CanUndo() const { return cursor != 0; }
    bool CanRedo() const { return cursor < actions.size(); }
    bool Record(int roadIndex, EditSnapshot before, EditSnapshot after);
    Result Undo(RoadMap& map, RoadEditorState& editor);
    Result Redo(RoadMap& map, RoadEditorState& editor);

private:
    struct Action
    {
        int roadIndex; // -1 for a tool or draft edit.
        EditSnapshot before;
        EditSnapshot after;
    };
    static Result Restore(RoadMap& map, RoadEditorState& editor, int index,
                          const EditSnapshot& from, const EditSnapshot& to);
    std::vector<Action> actions;
    std::size_t cursor = 0; // Number of applied actions.
};
