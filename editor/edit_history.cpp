#include "edit_history.hpp"

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

    void WriteStyle(std::ostream& out, const RoadStyle& style)
    {
        out << style.GetLaneCount() << ' ' << style.curved << ' ' << style.leftShoulder << ' '
            << style.rightShoulder << ' ' << static_cast<int>(style.threeLaneLayout) << ' '
            << style.divided << ' ' << style.dividedGap << '\n';
    }

    void ReadStyle(std::istream& in, RoadStyle& style)
    {
        int lanes, layout;
        in >> lanes >> style.curved >> style.leftShoulder >> style.rightShoulder >> layout
           >> style.divided >> style.dividedGap;
        style.SetLaneCount(lanes);
        style.threeLaneLayout = static_cast<ThreeLaneLayout>(layout);
    }

    void WriteRoad(std::ostream& out, const Road& road)
    {
        WriteStyle(out, road.style);
        out << road.points.size() << '\n';
        for (const RoadPoint& p : road.points)
        {
            out << p.position.x << ' ' << p.position.y << ' ' << p.position.z << ' '
                << p.heading.has_value() << ' ' << p.heading.value_or(0.0) << '\n';
        }
    }

    Road ReadRoad(std::istream& in)
    {
        Road road;
        ReadStyle(in, road.style);
        std::size_t count;
        in >> count;
        road.points.resize(count);
        for (RoadPoint& p : road.points)
        {
            bool hasHeading;
            double heading;
            in >> p.position.x >> p.position.y >> p.position.z >> hasHeading >> heading;
            if (hasHeading)
            {
                p.heading = heading;
            }
        }
        return road;
    }
}

EditSnapshot::EditSnapshot(const RoadEditorState& editor, const Road* affectedRoad)
{
    auto out = Writer();
    WriteStyle(out, editor.defaults);
    out << editor.selected << ' ' << editor.point << ' ' << editor.drawing << ' ' << editor.draft.has_value() << '\n';
    if (editor.draft)
    {
        WriteRoad(out, *editor.draft);
    }
    editorData = std::move(out).str();
    if (affectedRoad)
    {
        auto road = Writer();
        WriteRoad(road, *affectedRoad);
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

EditHistory::Result EditHistory::Restore(RoadMap& map, RoadEditorState& editor, int index,
                                         const EditSnapshot& from, const EditSnapshot& to)
{
    RoadEditorState restored;
    auto in = Reader(to.editorData);
    ReadStyle(in, restored.defaults);
    bool hasDraft;
    in >> restored.selected >> restored.point >> restored.drawing >> hasDraft;
    if (hasDraft)
    {
        restored.draft = ReadRoad(in);
    }

    // Insert/delete at the original index to preserve draw order and selection through replay.
    if (to.roadData)
    {
        auto payload = Reader(*to.roadData);
        Road road = ReadRoad(payload);
        road.Rebuild();
        if (from.roadData)
        {
            map.roads.at(index) = std::move(road);
        }
        else
        {
            map.roads.insert(map.roads.begin() + index, std::move(road));
        }
    }
    else if (from.roadData)
    {
        map.roads.erase(map.roads.begin() + index);
    }
    editor = std::move(restored);
    return index < 0 ? Result::ToolChanged : Result::MapChanged;
}

EditHistory::Result EditHistory::Undo(RoadMap& map, RoadEditorState& editor)
{
    if (!CanUndo())
    {
        return Result::Unavailable;
    }
    const Action& action = actions[cursor - 1];
    const Result result = Restore(map, editor, action.roadIndex, action.after, action.before);
    --cursor;
    return result;
}

EditHistory::Result EditHistory::Redo(RoadMap& map, RoadEditorState& editor)
{
    if (!CanRedo())
    {
        return Result::Unavailable;
    }
    const Action& action = actions[cursor];
    const Result result = Restore(map, editor, action.roadIndex, action.before, action.after);
    ++cursor;
    return result;
}
