#pragma once

#include "edit_history.hpp"
#include <array>

struct OutlineRow
{
    SceneKey key;
    std::string label, type;
    int depth;
    bool branch, expanded;
};

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
};

class SceneOutliner
{
  public:
    void Invalidate() { dirty = true; }
    void Update(const WorldDocument& world, const RoadEditorState& state);
    bool Apply(const OutlineCommand& command, WorldDocument& world, RoadEditorState& state, EditHistory& history);
    int NewFolderParent() const { return parentForNew; }
    std::string NewFolderName() const;
    const auto& Rows() const { return rows; }
    const auto& Selection() const { return selection; }
    unsigned Version() const { return version; }

  private:
    std::vector<OutlineRow> rows;
    std::set<SceneKey> selection;
    std::set<int> collapsed;
    std::set<std::string> siblingNames;
    int parentForNew = 0;
    std::array<std::size_t, 5> counts{};
    bool dirty = true;
    unsigned version = 0;
};
