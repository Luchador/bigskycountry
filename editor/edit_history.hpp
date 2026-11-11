#pragma once

#include "world_document.hpp"
#include "bscobject.hpp"
#include "scene_hierarchy.hpp"
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

    std::optional<IntersectionAnchor> anchorSelection; // Secondary selection paired with an active road endpoint.
    TransformTarget meshSelection;
    std::set<SceneKey> outlineSelection; // Folder, static mesh, or multiple rows.
    std::set<SceneKey> SelectedRows() const;
    void SelectRows(const std::set<SceneKey>& keys);
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
        anchorSelection.reset();
        selected = roadIndex;
        point = pointIndex;
        additionalPoints.clear();
        segments.clear();
        meshSelection = {};
        outlineSelection.clear();
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
    struct TerrainMaterial
    {
        int index;
        std::string path;
        bool operator==(const TerrainMaterial&) const = default;
    };
    std::optional<TerrainMaterial> terrainMaterial;
    struct Junction
    {
        int id;
        std::optional<Intersection> value; // Empty on the unplaced side of a create/delete.
        bool operator==(const Junction&) const = default;
    };
    std::optional<Junction> junction;
    std::vector<WorldDocument::ConnectedEnd> detachedEnds; // Restore these links when undoing a junction deletion.
    std::optional<IntersectionAnchor> anchorSelection;
    std::optional<bool> objectInvincible;
    std::optional<SceneHierarchy> hierarchy;
    std::set<SceneKey> outlineSelection;
    bool operator==(const EditSnapshot&) const = default;

    std::string editorData;
    int roadFolder = 0; // All sections of a split inherit the original road's folder.
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
    struct ObjectEdit
    {
        int index, folder;
        // Exactly one owner: the world while placed, this record while removed.
        std::unique_ptr<BSCObject> removed;
    };
    bool Record(int roadIndex, EditSnapshot before, EditSnapshot after, std::optional<ObjectEdit> object = {});
    void ReplaceSelectedRoad(WorldDocument& world, RoadEditorState& editor, std::vector<Road> replacements);
    Result Undo(WorldDocument& world, RoadEditorState& editor);
    Result Redo(WorldDocument& world, RoadEditorState& editor);

private:
    struct Action
    {
        int roadIndex; // -1 for a tool or draft edit.
        EditSnapshot before;
        EditSnapshot after;
        std::optional<ObjectEdit> object;
    };
    static Result Restore(WorldDocument& world, RoadEditorState& editor, Action& action, bool undo);
    std::vector<Action> actions;
    std::size_t cursor = 0; // Number of applied actions.
};
