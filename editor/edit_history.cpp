#include "edit_history.hpp"
#include "road_io.hpp"

#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>

namespace
{
    // Session-local payload. Keep the source points and settings, never the generated meshes.
    std::ostringstream Writer()
    {
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << std::setprecision(std::numeric_limits<double>::max_digits10);
        return out;
    }

    std::istringstream Reader(const std::string& data)
    {
        std::istringstream in(data);
        in.imbue(std::locale::classic());
        in.exceptions(std::ios::failbit | std::ios::badbit);
        return in;
    }
}

EditSnapshot::EditSnapshot(const RoadEditorState& editor, const Road* affectedRoad)
{
    auto out = Writer();
    RoadIO::WriteStyle(out, editor.defaults);
    out << editor.selected << ' ' << editor.point << ' ' << editor.drawing << ' ' << editor.draft.has_value() << '\n';
    if (editor.draft)
    {
        RoadIO::WriteRoad(out, *editor.draft);
    }
    editorData = std::move(out).str();
    if (affectedRoad)
    {
        auto road = Writer();
        RoadIO::WriteRoad(road, *affectedRoad);
        roadData = std::move(road).str();
    }
}

bool EditHistory::Record(int roadIndex, EditSnapshot before, EditSnapshot after)
{
    if (before == after)
    {
        return false;
    }
    actions.erase(actions.begin() + cursor, actions.end());
    actions.push_back({roadIndex, std::move(before), std::move(after)});
    cursor = actions.size();
    return true;
}

EditHistory::Result EditHistory::Restore(std::vector<Road>& roads, RoadEditorState& editor, int index,
                                         const EditSnapshot& from, const EditSnapshot& to)
{
    RoadEditorState restored;
    auto in = Reader(to.editorData);
    RoadIO::ReadStyle(in, restored.defaults);
    bool hasDraft;
    in >> restored.selected >> restored.point >> restored.drawing >> hasDraft;
    if (hasDraft)
    {
        restored.draft = RoadIO::ReadRoad(in);
    }

    // Insert/delete at the original index to preserve draw order and selection through replay.
    if (to.roadData)
    {
        auto payload = Reader(*to.roadData);
        Road road = RoadIO::ReadRoad(payload);
        road.Rebuild();
        if (from.roadData)
        {
            roads.at(index) = std::move(road);
        }
        else
        {
            roads.insert(roads.begin() + index, std::move(road));
        }
    }
    else if (from.roadData)
    {
        roads.erase(roads.begin() + index);
    }
    editor = std::move(restored);
    return index < 0 ? Result::ToolChanged : Result::MapChanged;
}

EditHistory::Result EditHistory::Undo(std::vector<Road>& roads, RoadEditorState& editor)
{
    if (!CanUndo())
    {
        return Result::Unavailable;
    }
    const Action& action = actions[cursor - 1];
    const Result result = Restore(roads, editor, action.roadIndex, action.after, action.before);
    --cursor;
    return result;
}

EditHistory::Result EditHistory::Redo(std::vector<Road>& roads, RoadEditorState& editor)
{
    if (!CanRedo())
    {
        return Result::Unavailable;
    }
    const Action& action = actions[cursor];
    const Result result = Restore(roads, editor, action.roadIndex, action.before, action.after);
    ++cursor;
    return result;
}
