#pragma once

#include "edit_history.hpp"
#include <array>

struct OutlineRow
{
    SceneKey key;
    std::string label, type;
    int depth;
    bool branch, expanded;
    std::string DragLabel(std::size_t count) const;
};

// The caller owns the BGRA32 surface. Both editor backends use the source PNG.
SDL_Surface* LoadOutlineMoveIcon(bool allowed);

enum class OutlineOp
{
    Select,
    Expand,
    Create,
    Rename,
    Remove,
    FillTerrain,
    Move
};
struct OutlineCommand
{
    OutlineOp op;
    SceneKey target{};
    std::set<SceneKey> items;
    std::string name;
    FolderPlacement placement = FolderPlacement::Into;
};

struct OutlineDrop
{
    SceneKey target;
    FolderPlacement placement;
    int row; // Destination row for nesting, or the gap immediately before this visible row.
    int depth;
    bool IsInsertion() const { return placement != FolderPlacement::Into; }
    OutlineCommand Move(const std::set<SceneKey>& items) const { return {OutlineOp::Move, target, items, {}, placement}; }
    bool operator==(const OutlineDrop&) const = default;
};

class SceneOutliner
{
  public:
    void Invalidate() { dirty = true; }
    void Update(const WorldDocument& world, const RoadEditorState& state);
    bool Apply(const OutlineCommand& command, WorldDocument& world, RoadEditorState& state, EditHistory& history);
    bool CanDrop(const std::set<SceneKey>& items, SceneKey target, FolderPlacement placement = FolderPlacement::Into) const;
    std::optional<OutlineDrop> DropAt(const std::set<SceneKey>& items, int row, int yInRow, int height, int edge) const;
    int NewFolderParent() const { return parentForNew; }
    std::string NewFolderName() const;
    const auto& Rows() const { return rows; }
    const auto& Selection() const { return selection; }
    unsigned Version() const { return version; }

  private:
    const WorldDocument* document = nullptr; // The live world whose rows this view displays.
    std::vector<OutlineRow> rows;
    std::set<SceneKey> selection;
    std::set<int> collapsed;
    std::set<std::string> siblingNames;
    int parentForNew = 0;
    std::array<std::size_t, 5> counts{};
    bool dirty = true;
    unsigned version = 0;
};
