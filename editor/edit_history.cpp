#include "edit_history.hpp"
#include "road_io.hpp"
#include "world_document.hpp"

#include <algorithm>
#include <iomanip>
#include <iterator>
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

    void ReplaceRoadRange(std::vector<Road>& roads, int index, std::size_t count, std::vector<Road> replacements)
    {
        if (count == replacements.size())
        {
            std::move(replacements.begin(), replacements.end(), roads.begin() + index);
            return;
        }
        roads.reserve(roads.size() - count + replacements.size());
        const auto first = roads.begin() + index;
        roads.erase(first, first + count);
        roads.insert(roads.begin() + index, std::make_move_iterator(replacements.begin()),
                     std::make_move_iterator(replacements.end()));
    }
}

void RoadEditorState::PickPoint(int roadIndex, int index, bool add, bool subtract)
{
    if (subtract)
    {
        if (roadIndex != selected || point < 0)
        {
            return;
        }
        if (index == point)
        {
            point = additionalPoints.empty() ? -1 : *additionalPoints.rbegin();
            additionalPoints.erase(point);
        }
        else
        {
            additionalPoints.erase(index);
        }
        return;
    }
    if (!add || roadIndex != selected || point < 0)
    {
        SelectPoint(roadIndex, index);
        return;
    }
    if (point != index)
    {
        additionalPoints.insert(point);
        additionalPoints.erase(index);
        point = index;
    }
}

EditSnapshot::EditSnapshot(const RoadEditorState& editor, const Road* affectedRoad)
    : EditSnapshot(editor, affectedRoad ? std::span(affectedRoad, 1) : std::span<const Road>{})
{
}

EditSnapshot::EditSnapshot(const RoadEditorState& editor, std::span<const Road> affectedRoads)
    : meshSelection(editor.meshSelection), additionalPoints(editor.additionalPoints), segments(editor.segments)
{
    auto out = Writer();
    RoadIO::WriteStyle(out, editor.defaults);
    out << editor.selected << ' ' << editor.point << ' ' << editor.drawing << ' ' << editor.draft.has_value() << '\n';
    if (editor.draft)
    {
        RoadIO::WriteRoad(out, *editor.draft);
    }
    editorData = std::move(out).str();
    for (const Road& source : affectedRoads)
    {
        auto road = Writer();
        RoadIO::WriteRoad(road, source);
        roadData.push_back(std::move(road).str());
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

void EditHistory::ReplaceSelectedRoad(WorldDocument& world, RoadEditorState& editor, std::vector<Road> replacements)
{
    const int index = editor.selected;
    EditSnapshot before(editor, &world.roads.at(index));
    const std::size_t count = replacements.size();
    ReplaceRoadRange(world.roads, index, 1, std::move(replacements));
    editor.SelectPoint(count ? index : -1, -1);
    EditSnapshot after(editor, std::span(world.roads).subspan(index, count));
    Record(index, std::move(before), std::move(after));
}

EditHistory::Result EditHistory::Restore(WorldDocument& world, RoadEditorState& editor, int index,
                                         const EditSnapshot& from, const EditSnapshot& to)
{
    auto& roads = world.roads;
    RoadEditorState restored;
    restored.meshSelection = to.meshSelection;
    restored.segments = to.segments;
    restored.additionalPoints = to.additionalPoints;
    auto in = Reader(to.editorData);
    RoadIO::ReadStyle(in, restored.defaults);
    bool hasDraft;
    in >> restored.selected >> restored.point >> restored.drawing >> hasDraft;
    if (hasDraft)
    {
        restored.draft = RoadIO::ReadRoad(in);
    }

    if (to.addedTerrain && !world.PlaceTerrain(to.addedTerrain->modelPath, to.addedTerrain->transform))
    {
        return Result::Failed;
    }
    if (from.addedTerrain && !world.RemoveTerrain(from.meshSelection.index))
    {
        return Result::Failed;
    }
    if (!to.roadData.empty() || !from.roadData.empty())
    {
        std::vector<Road> replacements;
        replacements.reserve(to.roadData.size());
        for (const auto& record : to.roadData)
        {
            auto payload = Reader(record);
            Road road = RoadIO::ReadRoad(payload);
            road.Rebuild();
            replacements.push_back(std::move(road));
        }
        ReplaceRoadRange(roads, index, from.roadData.size(), std::move(replacements));
    }
    if (to.meshTransform)
    {
        const auto target = to.meshSelection;
        if (target.kind == SelectionKind::Object) world.Objects().at(target.index)->SetTransform(*to.meshTransform);
        else world.Terrain().at(target.index)->SetTransform(*to.meshTransform);
    }
    editor = std::move(restored);
    const bool mapChanged = index >= 0 || to.meshTransform || from.addedTerrain || to.addedTerrain;
    return mapChanged ? Result::MapChanged : Result::ToolChanged;
}

EditHistory::Result EditHistory::Undo(WorldDocument& world, RoadEditorState& editor)
{
    if (!CanUndo())
    {
        return Result::Unavailable;
    }
    const Action& action = actions[cursor - 1];
    const Result result = Restore(world, editor, action.roadIndex, action.after, action.before);
    if (result != Result::Failed)
    {
        --cursor;
    }
    return result;
}

EditHistory::Result EditHistory::Redo(WorldDocument& world, RoadEditorState& editor)
{
    if (!CanRedo())
    {
        return Result::Unavailable;
    }
    const Action& action = actions[cursor];
    const Result result = Restore(world, editor, action.roadIndex, action.before, action.after);
    if (result != Result::Failed)
    {
        ++cursor;
    }
    return result;
}
