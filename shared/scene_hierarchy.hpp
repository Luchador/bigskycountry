#pragma once

#include <compare>
#include <iosfwd>
#include <map>
#include <set>
#include <string>
#include <vector>

enum class SceneKind
{
    World,
    Folder,
    Road,
    Terrain,
    Object,
    Mesh,
    Intersection
};

struct SceneKey
{
    SceneKind kind = SceneKind::World;
    int index = 0;
    auto operator<=>(const SceneKey&) const = default;
};

enum class FolderPlacement { Into, Before, After };

struct SceneFolder
{
    int parent = 0;
    std::string name;
    std::vector<int> children; // Sibling order in the Outliner.
    bool operator==(const SceneFolder&) const = default;
};

// Folder 0 is the world root. Only non-root assignments need a record.
class SceneHierarchy
{
  public:
    const auto& Folders() const { return folders; }
    const auto& Members() const { return members; }
    int Parent(SceneKey key) const;
    bool HasVisibilityOverrides() const { return !hiddenChildren.empty(); }
    bool EditorHidden(SceneKey key) const;
    bool SetChildrenHidden(int folder, bool hidden);
    std::vector<int> OrderedFolderIds() const;
    int Create(int parent, std::string name);
    bool Rename(int id, std::string name);
    bool Remove(int id);
    bool CanMove(const std::set<SceneKey>& keys, int target, FolderPlacement placement = FolderPlacement::Into) const;
    bool Move(const std::set<SceneKey>& keys, int target, FolderPlacement placement = FolderPlacement::Into);
    void ReplaceRange(SceneKind kind, int first, int removed, int inserted, int folder = 0);
    bool Write(std::ostream& out) const;
    static constexpr unsigned currentVersion = 2;
    bool Read(std::istream& in, unsigned version = currentVersion);
    bool operator==(const SceneHierarchy&) const = default;

  private:
    struct MovePlan
    {
        int parent = 0;
        std::vector<SceneKey> items;
        std::vector<int> siblings; // Final child folder order if any folders move.
    };
    MovePlan PlanMove(const std::set<SceneKey>& keys, int target, FolderPlacement placement) const;
    std::vector<int>& Children(int parent) { return parent ? folders.at(parent).children : rootFolders; }
    bool ValidName(int parent, const std::string& name, int except = 0) const;
    std::vector<int> rootFolders;
    std::map<int, SceneFolder> folders;
    std::map<SceneKey, int> members;
    // Nearest folder override wins; the world root is visible.
    std::map<int, bool> hiddenChildren;
};
