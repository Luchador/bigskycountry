#include "scene_hierarchy.hpp"
#include <algorithm>
#include <iomanip>
#include <istream>
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
    folders.erase(it);
    for (auto& [child, folder] : folders)
    {
        if (folder.parent == id)
        {
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

std::vector<SceneKey> SceneHierarchy::ItemsToMove(const std::set<SceneKey>& keys, int destination) const
{
    if (destination != 0 && !folders.contains(destination))
    {
        return {};
    }
    if (keys.contains({SceneKind::World, 0}))
    {
        return {};
    }
    for (int p = destination; p != 0; p = folders.at(p).parent)
    {
        if (keys.contains({SceneKind::Folder, p}))
        {
            return {};
        }
    }
    // A selected folder carries its descendants, including any also selected.
    std::set<std::string> names;
    std::vector<SceneKey> moves;
    for (SceneKey key : keys)
    {
        if (key.kind == SceneKind::Folder && !folders.contains(key.index))
        {
            return {};
        }
        int parent = Parent(key);
        if (parent == destination)
        {
            continue;
        }
        while (parent && !keys.contains({SceneKind::Folder, parent}))
        {
            parent = folders.at(parent).parent;
        }
        if (parent)
        {
            continue;
        }
        if (key.kind == SceneKind::Folder)
        {
            const auto& name = folders.at(key.index).name;
            if (!ValidName(destination, name, key.index) || !names.insert(name).second)
            {
                return {};
            }
        }
        moves.push_back(key);
    }
    return moves;
}

bool SceneHierarchy::CanMove(const std::set<SceneKey>& keys, int destination) const
{
    return !ItemsToMove(keys, destination).empty();
}

bool SceneHierarchy::Move(const std::set<SceneKey>& keys, int destination)
{
    const auto moves = ItemsToMove(keys, destination);
    for (SceneKey key : moves)
    {
        if (key.kind == SceneKind::Folder)
        {
            folders.at(key.index).parent = destination;
        }
        else if (destination)
        {
            members[key] = destination;
        }
        else
        {
            members.erase(key);
        }
    }
    return !moves.empty();
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
    for (const auto& [id, f] : folders)
    {
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
    return out.good();
}

bool SceneHierarchy::Read(std::istream& in)
{
    SceneHierarchy parsed;
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
    *this = std::move(parsed);
    return true;
}
