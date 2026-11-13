#include "scene_hierarchy.hpp"
#include <algorithm>
#include <iomanip>
#include <istream>
#include <iterator>
#include <limits>
#include <ostream>
#include <utility>
#include <vector>

namespace
{
    bool NameAllowed(const std::string& name)
    {
        return !name.empty() && name.size() <= 255 && name.find_first_not_of(" ") != name.npos &&
               std::none_of(name.begin(), name.end(), [](unsigned char c) { return c < 32 || c == 127; });
    }
} // namespace

int SceneHierarchy::Parent(SceneKey key) const
{
    if (key.kind == SceneKind::Folder)
    {
        const auto it = folders.find(key.index);
        return it == folders.end() ? 0 : it->second.parent;
    }
    const auto it = members.find(key);
    return it == members.end() ? 0 : it->second;
}

bool SceneHierarchy::EditorHidden(SceneKey key) const
{
    if (hiddenChildren.empty()) return false;
    int id = key.kind == SceneKind::Folder ? key.index : Parent(key);
    while (id)
    {
        const auto setting = hiddenChildren.find(id);
        if (setting != hiddenChildren.end()) return setting->second;
        id = Parent({SceneKind::Folder, id});
    }
    return false;
}

bool SceneHierarchy::SetChildrenHidden(int id, bool hidden)
{
    const auto root = folders.find(id);
    if (root == folders.end()) return false;
    bool changed = EditorHidden({SceneKind::Folder, id}) != hidden;
    if (EditorHidden({SceneKind::Folder, root->second.parent}) == hidden)
    {
        if (hiddenChildren.erase(id)) changed = true;
    }
    else
        hiddenChildren[id] = hidden;

    // A command on the parent also resets any exceptions in its descendants.
    auto pending = root->second.children;
    while (!pending.empty())
    {
        const int child = pending.back();
        pending.pop_back();
        if (hiddenChildren.erase(child)) changed = true;
        const auto& children = folders.at(child).children;
        pending.insert(pending.end(), children.begin(), children.end());
    }
    return changed;
}

std::vector<int> SceneHierarchy::OrderedFolderIds() const
{
    std::vector<int> result;
    result.reserve(folders.size());
    std::vector<int> stack(rootFolders.rbegin(), rootFolders.rend());
    while (!stack.empty())
    {
        const int id = stack.back();
        stack.pop_back();
        result.push_back(id);
        const auto& children = folders.at(id).children;
        stack.insert(stack.end(), children.rbegin(), children.rend());
    }
    return result;
}

bool SceneHierarchy::ValidName(int parent, const std::string& name, int except) const
{
    if (!NameAllowed(name) || (parent != 0 && !folders.contains(parent)))
    {
        return false;
    }
    for (const auto& [id, folder] : folders)
    {
        if (id != except && parent == folder.parent && name == folder.name)
        {
            return false;
        }
    }
    return true;
}

int SceneHierarchy::Create(int parent, std::string name)
{
    if (!ValidName(parent, name) || (!folders.empty() && folders.rbegin()->first == std::numeric_limits<int>::max()))
    {
        return 0;
    }
    const int id = folders.empty() ? 1 : folders.rbegin()->first + 1;
    folders.emplace(id, SceneFolder{parent, std::move(name)});
    Children(parent).push_back(id);
    return id;
}

bool SceneHierarchy::Rename(int id, std::string name)
{
    auto it = folders.find(id);
    if (it == folders.end() || it->second.name == name || !ValidName(it->second.parent, name, id))
    {
        return false;
    }
    it->second.name = std::move(name);
    return true;
}

bool SceneHierarchy::Remove(int id)
{
    const auto it = folders.find(id);
    if (it == folders.end())
    {
        return false;
    }
    const int parent = it->second.parent;
    const auto children = std::move(it->second.children);
    auto& siblings = Children(parent);
    const auto slot = siblings.erase(std::find(siblings.begin(), siblings.end(), id));
    siblings.insert(slot, children.begin(), children.end());
    folders.erase(it);
    hiddenChildren.erase(id);
    for (int child : children)
    {
        auto& folder = folders.at(child);
        // Promoting a child can collide with an existing sibling name.
        const std::string base = folder.name;
        for (int n = 2; !ValidName(parent, folder.name, child); ++n)
        {
            const std::string suffix = " (" + std::to_string(n) + ")";
            auto length = std::min(base.size(), 255 - suffix.size());
            while (length < base.size() && (static_cast<unsigned char>(base[length]) & 0xc0) == 0x80)
            {
                --length;
            }
            folder.name = base.substr(0, length) + suffix;
        }
        folder.parent = parent;
    }
    for (auto member = members.begin(); member != members.end();)
    {
        if (member->second == id && parent == 0)
        {
            member = members.erase(member);
        }
        else
        {
            if (member->second == id)
            {
                member->second = parent;
            }
            ++member;
        }
    }
    return true;
}

SceneHierarchy::MovePlan SceneHierarchy::PlanMove(const std::set<SceneKey>& keys, int target, FolderPlacement placement) const
{
    const bool insert = placement != FolderPlacement::Into;
    if ((target || insert) && !folders.contains(target)) return {};
    if (keys.empty() || keys.contains({SceneKind::World, 0})) return {};
    MovePlan plan;
    plan.parent = insert ? folders.at(target).parent : target;
    for (int p = plan.parent; p; p = folders.at(p).parent)
    {
        if (keys.contains({SceneKind::Folder, p})) return {};
    }

    std::set<std::string> names;
    std::set<int> movingFolders;
    for (SceneKey key : keys)
    {
        const bool folder = key.kind == SceneKind::Folder;
        if ((insert && !folder) || (folder && !folders.contains(key.index))) return {};
        int parent = Parent(key);
        if (!insert && parent == plan.parent) continue;
        // Moving a folder carries its whole subtree, including selected descendants.
        while (parent && !keys.contains({SceneKind::Folder, parent})) parent = folders.at(parent).parent;
        if (parent) continue;
        if (folder)
        {
            const auto& value = folders.at(key.index);
            if ((value.parent != plan.parent && !ValidName(plan.parent, value.name, key.index)) ||
                !names.insert(value.name).second) return {};
            movingFolders.insert(key.index);
        }
        plan.items.push_back(key);
    }
    if (movingFolders.empty()) return plan;

    std::vector<int> block;
    if (movingFolders.size() == 1) block.push_back(*movingFolders.begin());
    else
    {
        for (int id : OrderedFolderIds())
            if (movingFolders.contains(id)) block.push_back(id);
    }
    const auto& current = plan.parent ? folders.at(plan.parent).children : rootFolders;
    auto slot = current.end();
    if (insert)
    {
        slot = std::find(current.begin(), current.end(), target);
        if (placement == FolderPlacement::After) ++slot;
    }
    const auto stays = [&](int id) { return !movingFolders.contains(id); };
    std::copy_if(current.begin(), slot, std::back_inserter(plan.siblings), stays);
    plan.siblings.insert(plan.siblings.end(), block.begin(), block.end());
    std::copy_if(slot, current.end(), std::back_inserter(plan.siblings), stays);
    if (insert && plan.siblings == current) return {};
    return plan;
}

bool SceneHierarchy::CanMove(const std::set<SceneKey>& keys, int target, FolderPlacement placement) const
{
    return !PlanMove(keys, target, placement).items.empty();
}

bool SceneHierarchy::Move(const std::set<SceneKey>& keys, int target, FolderPlacement placement)
{
    auto plan = PlanMove(keys, target, placement);
    for (SceneKey key : plan.items)
    {
        if (key.kind == SceneKind::Folder)
        {
            auto& folder = folders.at(key.index);
            if (folder.parent != plan.parent) std::erase(Children(folder.parent), key.index);
            folder.parent = plan.parent;
        }
        else if (plan.parent)
        {
            members[key] = plan.parent;
        }
        else
        {
            members.erase(key);
        }
    }
    if (!plan.siblings.empty()) Children(plan.parent) = std::move(plan.siblings);
    return !plan.items.empty();
}

void SceneHierarchy::ReplaceRange(SceneKind kind, int first, int removed, int inserted, int parent)
{
    if (members.empty() && parent == 0)
    {
        return;
    }
    std::map<SceneKey, int> shifted;
    for (const auto& [key, folder] : members)
    {
        if (key.kind != kind || key.index < first)
        {
            shifted.emplace(key, folder);
        }
        else if (key.index >= first + removed)
        {
            shifted.emplace(SceneKey{kind, key.index - removed + inserted}, folder);
        }
    }
    if (parent)
    {
        for (int i = 0; i < inserted; ++i)
        {
            shifted.emplace(SceneKey{kind, first + i}, parent);
        }
    }
    members.swap(shifted);
}

bool SceneHierarchy::Write(std::ostream& out) const
{
    // F record order carries sibling order. Older worlds wrote these records in ID order.
    for (int id : OrderedFolderIds())
    {
        const auto& f = folders.at(id);
        out << "F " << id << ' ' << f.parent << ' ' << std::quoted(f.name) << '\n';
    }
    for (const auto& [key, id] : members)
    {
        const char tag = key.kind == SceneKind::Road      ? 'R'
                         : key.kind == SceneKind::Intersection ? 'I'
                         : key.kind == SceneKind::Terrain ? 'T'
                         : key.kind == SceneKind::Object  ? 'O'
                                                          : 'M';
        out << tag << ' ' << key.index << ' ' << id << '\n';
    }
    for (const auto& [id, hidden] : hiddenChildren)
        out << "V " << id << ' ' << static_cast<int>(hidden) << '\n';
    return out.good();
}

bool SceneHierarchy::Read(std::istream& in, unsigned version)
{
    if (version < 1 || version > currentVersion) return false;
    SceneHierarchy parsed;
    std::vector<int> order;
    std::set<std::pair<int, std::string>> names;
    while ((in >> std::ws) && !in.eof())
    {
        char tag;
        int index, parent;
        if (!(in >> tag >> index >> parent) || parent < 0 || index < 0)
        {
            return false;
        }
        if (tag == 'F')
        {
            std::string name;
            if (index == 0 || !(in >> std::quoted(name)) || !NameAllowed(name) || !names.emplace(parent, name).second ||
                !parsed.folders.emplace(index, SceneFolder{parent, name}).second)
            {
                return false;
            }
            order.push_back(index);
        }
        else if (tag == 'V' && version >= 2)
        {
            if (index == 0 || parent > 1 || !parsed.hiddenChildren.emplace(index, parent != 0).second)
                return false;
        }
        else
        {
            SceneKind kind;
            switch (tag)
            {
            case 'I':
                kind = SceneKind::Intersection;
                break;
            case 'R':
                kind = SceneKind::Road;
                break;
            case 'T':
                kind = SceneKind::Terrain;
                break;
            case 'O':
                kind = SceneKind::Object;
                break;
            case 'M':
                kind = SceneKind::Mesh;
                break;
            default:
                return false;
            }
            if (parent == 0 || !parsed.members.emplace(SceneKey{kind, index}, parent).second)
            {
                return false;
            }
        }
    }
    std::set<int> valid{0};
    for (const auto& [id, folder] : parsed.folders)
    {
        std::set<int> path;
        for (int p = id; !valid.contains(p); p = parsed.folders.at(p).parent)
        {
            if (!parsed.folders.contains(p) || !path.insert(p).second)
            {
                return false;
            }
        }
        valid.insert(path.begin(), path.end());
    }
    for (const auto& [key, id] : parsed.members)
    {
        if (!parsed.folders.contains(id))
        {
            return false;
        }
    }
    for (const auto& [id, hidden] : parsed.hiddenChildren)
        if (!parsed.folders.contains(id)) return false;
    for (int id : order) parsed.Children(parsed.folders.at(id).parent).push_back(id);
    *this = std::move(parsed);
    return true;
}
