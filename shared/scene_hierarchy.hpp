#pragma once

#include <compare>
#include <iosfwd>
#include <map>
#include <set>
#include <string>

enum class SceneKind
{
    World,
    Folder,
    Road,
    Terrain,
    Object,
    Mesh
};

struct SceneKey
{
    SceneKind kind = SceneKind::World;
    int index = 0;
    auto operator<=>(const SceneKey&) const = default;
};

struct SceneFolder
{
    int parent = 0;
    std::string name;
    bool operator==(const SceneFolder&) const = default;
};

// Folder 0 is the world root. Only non-root assignments need a record.
class SceneHierarchy
{
  public:
    const auto& Folders() const { return folders; }
    const auto& Members() const { return members; }
    int Parent(SceneKey key) const;
    int Create(int parent, std::string name);
    bool Rename(int id, std::string name);
    bool Remove(int id);
    bool Move(const std::set<SceneKey>& keys, int destination);
    void ReplaceRange(SceneKind kind, int first, int removed, int inserted, int folder = 0);
    bool Write(std::ostream& out) const;
    bool Read(std::istream& in);
    bool operator==(const SceneHierarchy&) const = default;

  private:
    bool ValidName(int parent, const std::string& name, int except = 0) const;
    std::map<int, SceneFolder> folders;
    std::map<SceneKey, int> members;
};
