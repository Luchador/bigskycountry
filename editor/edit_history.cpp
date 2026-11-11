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

std::set<SceneKey> RoadEditorState::SelectedRows() const
{
    if (!outlineSelection.empty()) return outlineSelection;
    if (selected >= 0) return {{SceneKind::Road, selected}};
    if (meshSelection.kind == SelectionKind::Intersection || meshSelection.kind == SelectionKind::Anchor)
        return {{SceneKind::Intersection, meshSelection.index}};
    if (meshSelection.kind == SelectionKind::Object) return {{SceneKind::Object, meshSelection.index}};
    if (meshSelection.kind == SelectionKind::Terrain) return {{SceneKind::Terrain, meshSelection.index}};
    return {};
}

void RoadEditorState::SelectRows(const std::set<SceneKey>& keys)
{
    drawing = false;
    SelectPoint(-1, -1);
    if (keys.size() == 1)
    {
        const auto key = *keys.begin();
        if (key.kind == SceneKind::Road) { selected = key.index; return; }
        if (key.kind == SceneKind::Intersection) { meshSelection = {SelectionKind::Intersection, key.index}; return; }
        if (key.kind == SceneKind::Object) { meshSelection = {SelectionKind::Object, key.index}; return; }
        if (key.kind == SceneKind::Terrain) { meshSelection = {SelectionKind::Terrain, key.index}; return; }
    }
    outlineSelection = keys;
}

void RoadEditorState::PickPoint(int roadIndex, int index, bool add, bool subtract)
{
    anchorSelection.reset();
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
    : meshSelection(editor.meshSelection), anchorSelection(editor.anchorSelection), outlineSelection(editor.outlineSelection), additionalPoints(editor.additionalPoints), segments(editor.segments)
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

bool EditHistory::Record(int roadIndex, EditSnapshot before, EditSnapshot after, std::optional<ObjectEdit> object)
{
    if (!object && before == after)
    {
        return false;
    }
    actions.erase(actions.begin() + cursor, actions.end());
    actions.push_back({roadIndex, std::move(before), std::move(after), std::move(object)});
    cursor = actions.size();
    return true;
}

void EditHistory::ReplaceSelectedRoad(WorldDocument& world, RoadEditorState& editor, std::vector<Road> replacements)
{
    const int index = editor.selected;
    EditSnapshot before(editor, &world.roads.at(index));
    before.roadFolder = world.hierarchy.Parent({SceneKind::Road, index});
    const std::size_t count = replacements.size();
    world.hierarchy.ReplaceRange(SceneKind::Road, index, 1, static_cast<int>(count), before.roadFolder);
    ReplaceRoadRange(world.roads, index, 1, std::move(replacements));
    editor.SelectPoint(count ? index : -1, -1);
    EditSnapshot after(editor, std::span(world.roads).subspan(index, count));
    after.roadFolder = before.roadFolder;
    Record(index, std::move(before), std::move(after));
}

EditHistory::Result EditHistory::Restore(WorldDocument& world, RoadEditorState& editor, Action& action, bool undo)
{
    const int index = action.roadIndex;
    const EditSnapshot& from = undo ? action.after : action.before;
    const EditSnapshot& to = undo ? action.before : action.after;
    auto& roads = world.roads;
    RoadEditorState restored;
    restored.anchorSelection = to.anchorSelection;
    restored.meshSelection = to.meshSelection;
    restored.outlineSelection = to.outlineSelection;
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

    if (action.object)
    {
        auto& edit = *action.object;
        if (edit.removed)
        {
            if (!world.InsertObject(edit.index, std::move(edit.removed), edit.folder)) return Result::Failed;
        }
        else
        {
            edit.removed = world.TakeObject(edit.index);
            if (!edit.removed) return Result::Failed;
        }
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
        if (from.roadData.size() != replacements.size())
        {
            world.hierarchy.ReplaceRange(SceneKind::Road, index, static_cast<int>(from.roadData.size()),
                                         static_cast<int>(replacements.size()), to.roadFolder);
        }
        ReplaceRoadRange(roads, index, from.roadData.size(), std::move(replacements));
    }
    if (to.junction)
    {
        const auto& [id, value] = *to.junction;
        if (value) world.intersections.insert_or_assign(id, *value);
        else world.intersections.erase(id);
        // Deleting a junction leaves the roads in place. Undo restores their endpoint links.
        for (const auto& end : value ? to.detachedEnds : from.detachedEnds)
        {
            auto& points = roads.at(end.road).points;
            (end.first ? points.front() : points.back()).anchor = value ? std::optional{end.anchor} : std::nullopt;
        }
    }
    if (to.objectInvincible) world.Objects().at(to.meshSelection.index)->invincible = *to.objectInvincible;
    if (to.terrainMaterial)
    {
        const auto& target = *to.terrainMaterial;
        if (!world.Contains({SceneKind::Terrain, target.index})) return Result::Failed;
        world.Terrain()[target.index]->material = target.path;
    }
    if (to.meshTransform && !SetPlacementTransform(world, to.meshSelection, *to.meshTransform))
    {
        return Result::Failed;
    }
    if (to.hierarchy) world.hierarchy = *to.hierarchy;
    editor = std::move(restored);
    const bool mapChanged = to.junction || to.objectInvincible || action.object || index >= 0 || to.meshTransform || from.addedTerrain || to.addedTerrain || to.terrainMaterial || to.hierarchy;
    return mapChanged ? Result::MapChanged : Result::ToolChanged;
}

EditHistory::Result EditHistory::Undo(WorldDocument& world, RoadEditorState& editor)
{
    if (!CanUndo())
    {
        return Result::Unavailable;
    }
    const Result result = Restore(world, editor, actions[cursor - 1], true);
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
    const Result result = Restore(world, editor, actions[cursor], false);
    if (result != Result::Failed)
    {
        ++cursor;
    }
    return result;
}
