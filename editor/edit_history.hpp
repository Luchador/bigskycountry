#pragma once

#include "world_document.hpp"
#include "bscobject.hpp"
#include "marker.hpp"
#include "scene_hierarchy.hpp"
#include "transform_inspector.hpp"
#include "road_group_transform.hpp"

#include <cstdint>
#include <iterator>
#include <limits>
#include <ranges>
#include <set>
#include <span>
#include <string>

bool IsRoadGroup(const std::set<SceneKey>& keys);

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
    bool GeometrySelected(SceneKey key) const;
    bool RoadGroupSelected() const { return IsRoadGroup(outlineSelection); }
    auto SelectedRoadRows() const
    {
        return std::ranges::subrange(outlineSelection.lower_bound({SceneKind::Road, 0}),
            outlineSelection.upper_bound({SceneKind::Road, std::numeric_limits<int>::max()}));
    }
    bool MultipleRoadsSelected() const
    {
        const auto roads = SelectedRoadRows();
        return !roads.empty() && std::next(roads.begin()) != roads.end();
    }
    void PickElement(SceneKey key, bool add, bool subtract);
    void SelectRows(const std::set<SceneKey>& keys);
    TransformTarget Target() const
    {
        if (RoadGroupSelected()) return {SelectionKind::RoadGroup, -1, -1, outlineSelection};
        if (!outlineSelection.empty()) return {SelectionKind::None, -1, -1, outlineSelection};
        if (selected < 0) return meshSelection;
        return {point < 0 ? SelectionKind::Road : SelectionKind::Point, selected, point, {}, additionalPoints};
    }

    bool PointSelected(int index) const
    {
        return index == point || additionalPoints.contains(index);
    }

    void InspectAnchor(IntersectionAnchor anchor)
    {
        drawing = false;
        SelectPoint(-1, -1);
        meshSelection = {SelectionKind::Anchor, anchor.intersectionId, anchor.side};
    }
    bool InspectingAnchor(IntersectionAnchor anchor) const
    {
        return meshSelection == TransformTarget{SelectionKind::Anchor, anchor.intersectionId, anchor.side};
    }
    bool AnchorHighlighted(IntersectionAnchor anchor) const
    {
        return anchorSelection == anchor || InspectingAnchor(anchor);
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

// Authored inspector values only. Control points, junction links and generated meshes stay in place.
struct RoadProperties
{
    explicit RoadProperties(const Road& road) : style(road.style), names(road.NameList()), paintMarkings(road.paintMarkings) {}
    RoadStyle style;
    std::string names;
    bool paintMarkings;
    bool operator==(const RoadProperties&) const = default;
    void Apply(Road& road) const;
};

struct EditSnapshot
{
    EditSnapshot(const RoadEditorState& editor, const Road* affectedRoad = nullptr);
    EditSnapshot(const RoadEditorState& editor, std::span<const Road> affectedRoads);
    TransformTarget meshSelection;
    std::optional<ObjectTransform> meshTransform;
    std::optional<RoadGroupPose> roadGroup;
    std::map<int, RoadProperties> roadProperties;
    struct TerrainPlacement
    {
        std::string modelPath;
        ObjectTransform transform;
        std::optional<TerrainBoundary> boundary;
        std::string material;
        int folder = 0;
        bool operator==(const TerrainPlacement&) const = default;
    };
    std::optional<TerrainPlacement> addedTerrain;
    struct PlacedMaterial
    {
        SceneKey target;
        std::string path;
        bool operator==(const PlacedMaterial&) const = default;
    };
    std::optional<PlacedMaterial> placedMaterial;
    std::map<int, TerrainBoundary> splitBoundaries; // Perimeters affected by splitting an existing road.
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
    std::optional<std::string> markerText;
    std::optional<Marker::TextColor> markerTextColor;
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

    std::uint64_t WorldRevision() const { return cursor ? actions[cursor - 1].worldRevision : 0; }
    bool CanUndo() const { return cursor != 0; }
    bool CanRedo() const { return cursor < actions.size(); }
    struct ObjectEdit
    {
        int index, folder;
        // Exactly one owner: the world while placed, this record while removed.
        std::unique_ptr<BSCObject> removed;
    };
    bool Record(int roadIndex, EditSnapshot before, EditSnapshot after, std::optional<ObjectEdit> object = {});
    bool ReplaceRoad(WorldDocument& world, RoadEditorState& editor, int index, std::vector<Road> replacements,
                     const Intersection* inserted = nullptr);
    Result Undo(WorldDocument& world, RoadEditorState& editor);
    Result Redo(WorldDocument& world, RoadEditorState& editor);

private:
    struct Action
    {
        int roadIndex; // -1 for a tool or draft edit.
        EditSnapshot before;
        EditSnapshot after;
        std::optional<ObjectEdit> object;
        std::uint64_t worldRevision = 0;
        bool ChangesWorld() const;
    };
    static Result Restore(WorldDocument& world, RoadEditorState& editor, Action& action, bool undo);
    std::vector<Action> actions;
    std::uint64_t lastWorldRevision = 0;
    std::size_t cursor = 0; // Number of applied actions.
};
