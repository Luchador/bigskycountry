#include "edit_history.hpp"
#include "properties_inspector.hpp"
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

bool RoadEditorState::GeometrySelected(SceneKey key) const
{
    if (key.kind == SceneKind::Road && key.index == selected) return true;
    if (key.kind == SceneKind::Intersection && meshSelection == TransformTarget{SelectionKind::Intersection, key.index}) return true;
    return outlineSelection.contains(key);
}

bool IsRoadGroup(const std::set<SceneKey>& keys)
{
    return keys.size() > 1 && std::all_of(keys.begin(), keys.end(), [](SceneKey key)
    {
        return key.kind == SceneKind::Road || key.kind == SceneKind::Intersection;
    });
}

void RoadEditorState::PickElement(SceneKey key, bool add, bool subtract)
{
    if (subtract && !GeometrySelected(key)) return;
    auto keys = SelectedRows();
    if (subtract) keys.erase(key);
    else
    {
        if (!add) keys.clear();
        std::erase_if(keys, [](SceneKey k) { return k.kind != SceneKind::Road && k.kind != SceneKind::Intersection; });
        keys.insert(key);
    }
    SelectRows(keys);
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

void RoadProperties::Apply(Road& road) const
{
    if (road.NameList() != names) road.SetNameList(names);
    road.style.material = style.material;
    if (road.style != style || road.paintMarkings != paintMarkings)
    {
        road.style = style;
        road.paintMarkings = paintMarkings;
        road.Rebuild();
    }
}

bool EditHistory::Action::ChangesWorld() const
{
    return roadIndex >= 0 || !after.roadProperties.empty() || object || after.meshTransform || after.roadGroup || after.junction ||
           after.objectInvincible || after.markerText || after.markerTextColor || after.hierarchy || after.placedMaterial ||
           before.addedTerrain || after.addedTerrain;
}

bool EditHistory::Record(int roadIndex, EditSnapshot before, EditSnapshot after, std::optional<ObjectEdit> object)
{
    if (!object && before == after)
    {
        return false;
    }
    Action action{roadIndex, std::move(before), std::move(after), std::move(object)};
    action.worldRevision = action.ChangesWorld() ? ++lastWorldRevision : WorldRevision();
    actions.erase(actions.begin() + cursor, actions.end());
    actions.push_back(std::move(action));
    cursor = actions.size();
    return true;
}

bool EditHistory::ReplaceRoad(WorldDocument& world, RoadEditorState& editor, int index,
                              std::vector<Road> replacements, const Intersection* inserted)
{
    EditSnapshot before(editor, &world.roads.at(index));
    before.roadFolder = world.hierarchy.Parent({SceneKind::Road, index});
    for (std::size_t i = 0; i < replacements.size(); ++i)
    {
        replacements[i].id = i == 0 ? world.roads[index].id : world.AllocateRoadId();
        if (i && !replacements[i].id) return false;
    }
    int junctionId = 0;
    if (inserted)
    {
        junctionId = world.CreateIntersection(inserted->position);
        if (!junctionId) return false;
        world.intersections.at(junctionId) = *inserted;
        before.junction = EditSnapshot::Junction{junctionId, {}};
        // The planner marks each new attachment with ID zero. A far end can already belong to another junction.
        for (Road& piece : replacements)
        {
            for (RoadPoint* end : {&piece.points.front(), &piece.points.back()})
            {
                if (end->anchor && end->anchor->intersectionId == 0)
                    end->anchor->intersectionId = junctionId;
            }
        }
        if (before.roadFolder)
        {
            before.hierarchy = world.hierarchy;
            world.hierarchy.Move({{SceneKind::Intersection, junctionId}}, before.roadFolder);
        }
    }
    std::map<int, TerrainBoundary> newBoundaries;
    if (inserted || replacements.size() > 1)
    {
        for (int i = 0; i < static_cast<int>(world.Terrain().size()); ++i)
        {
            auto& boundary = world.Terrain()[i]->boundary;
            if (!boundary || !boundary->roads.contains(world.roads[index].id)) continue;
            before.splitBoundaries.emplace(i, *boundary);
            for (const Road& piece : replacements) boundary->roads.insert(piece.id);
            if (inserted) boundary->intersections.insert(junctionId);
            newBoundaries.emplace(i, *boundary);
        }
    }
    const std::size_t count = replacements.size();
    world.hierarchy.ReplaceRange(SceneKind::Road, index, 1, static_cast<int>(count), before.roadFolder);
    ReplaceRoadRange(world.roads, index, 1, std::move(replacements));
    if (inserted) editor.SelectRows({{SceneKind::Intersection, junctionId}});
    else editor.SelectPoint(count ? index : -1, -1);
    EditSnapshot after(editor, std::span(world.roads).subspan(index, count));
    after.roadFolder = before.roadFolder;
    after.splitBoundaries = std::move(newBoundaries);
    if (inserted) after.junction = EditSnapshot::Junction{junctionId, *inserted};
    if (before.hierarchy) after.hierarchy = world.hierarchy;
    return Record(index, std::move(before), std::move(after));
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
    if (to.addedTerrain)
    {
        const auto& placement = *to.addedTerrain;
        auto* chunk = placement.boundary ? world.PlaceTerrainFill(*placement.boundary, true) :
                                           world.PlaceTerrain(placement.modelPath, placement.transform);
        if (!chunk) return Result::Failed;
        chunk->material = placement.material;
        world.MoveLastTerrainTo(to.meshSelection.index);
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
    for (const auto& [i, boundary] : to.splitBoundaries) world.Terrain().at(i)->boundary = boundary;
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
    for (const auto& [i, properties] : to.roadProperties) properties.Apply(roads.at(i));
    if (to.roadGroup) to.roadGroup->Restore(world);
    if (to.markerText || to.markerTextColor)
    {
        auto* marker = dynamic_cast<Marker*>(world.Objects().at(to.meshSelection.index).get());
        if (!marker || (to.markerText && !marker->SetText(*to.markerText))) return Result::Failed;
        if (to.markerTextColor) marker->textColor = *to.markerTextColor;
    }
    if (to.objectInvincible) world.Objects().at(to.meshSelection.index)->invincible = *to.objectInvincible;
    if (to.placedMaterial)
    {
        auto* path = FindPlacedMaterial(world, to.placedMaterial->target);
        if (!path) return Result::Failed;
        *path = to.placedMaterial->path;
    }
    if (to.meshTransform && !SetPlacementTransform(world, to.meshSelection, *to.meshTransform))
    {
        return Result::Failed;
    }
    if (to.hierarchy) world.hierarchy = *to.hierarchy;
    editor = std::move(restored);
    return action.ChangesWorld() ? Result::MapChanged : Result::ToolChanged;
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
